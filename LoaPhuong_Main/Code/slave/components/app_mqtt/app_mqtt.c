#include "app_mqtt.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "network.h"
#include "app_flash.h"
#include "app_ota.h"
#include "app_io.h"
#include "DataDefine.h"
#include "lwip/init.h"
#include "esp_modem.h"
#include "esp_modem_dte.h"
#include "ec2x.h"
#include "utilities.h"
#include "app_audio.h"
#include "main.h"
#include "esp32/rom/rtc.h"

static const char *TAG = "app_mqtt";

static uint8_t m_mqtt_sub_req_tick = 0;
// MQTT
static esp_mqtt_client_handle_t m_mqtt_client;
static esp_mqtt_client_config_t m_mqtt_config;
static char m_slave_sub_topic[64] = {0};
static int m_mqtt_sub_cfg_msg_id = 0;
static char m_ota_http_url[256];
bool m_is_mqtt_subscribed = false;
static int m_mqtt_sub_master_msg_id[MASTER_TOTAL] = {0};
static bool m_mqtt_is_master_subscribed[MASTER_TOTAL] = {false};
static uint8_t m_mqtt_send_resp_config_url_num = 0;
static bool m_is_waiting_http_element_stopped = false;
static app_mqtt_state_t m_mqtt_fsm = APP_MQTT_DISCONNECTED;
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);
static app_mqtt_master_streamming_state_t m_master_streamming_info = 
{
    .Value = 0
};

bool app_mqtt_is_master_streamming(void)
{
    return m_master_streamming_info.Value ? true : false;
}

app_mqtt_master_streamming_state_t *app_mqtt_get_master_streamming_status(void)
{
    return &m_master_streamming_info;
}

bool app_mqtt_is_master_subscribed(uint8_t master_index)
{
    return m_mqtt_is_master_subscribed[master_index];
}

app_mqtt_state_t app_mqtt_get_state(void)
{
    return m_mqtt_fsm;
}

bool app_mqtt_is_connected_to_server(void)
{
    return (m_mqtt_fsm == APP_MQTT_CONNECTED) ? true : false;
}

void app_mqtt_set_state(app_mqtt_state_t state)
{
    m_mqtt_fsm = state;
}

bool app_mqtt_is_waiting_http_element_stopped(void)
{
    return m_is_waiting_http_element_stopped;
}

void app_mqtt_is_reset_http_element_stopped_flag(void)
{
    m_is_waiting_http_element_stopped = false;
}

/**
 * Gửi bản tin sau reset, check từng giây
 * Chỉ gửi khi đã lấy được thông tin IMEI của module GSM hoặc SIM (trường hợp online bằng Ethernet)
 */
void app_mqtt_send_reset_message(void)
{
    static bool is_sent = false;
    static uint8_t timeout_get_sim_imei = 35;

    modem_dce_t *dce = slave_get_modem_dce();

    if (is_sent)
        return;
    if (!dce)
        return;

    // Nếu chưa lấy được SIM IMEI thì chờ thêm...
    if (!IsDigitString(dce->sim_imei))
    {
        if (timeout_get_sim_imei > 0)
        {
            timeout_get_sim_imei--;
            return;
        }
        else
        {
            ESP_LOGW(TAG, "Het thoi gian cho lay SIM IMEI!");
        }
    }

    char *reset_info = malloc(512);
    if (!reset_info)
    {
        ESP_LOGE(TAG, "Can't allocate mem!");
        return;
    }

    uint8_t uid[6];
    esp_efuse_mac_get_default(uid);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);


    sprintf(reset_info, "RESET|MAC:%02X%02X%02X%02X%02X%02X,"
            "GSM:%s-%s,SIM:%s-%s,FW:%s,HW:%d,Reason:%u-%u-%u,Count:%u,IO:%02X,VOL:%d,SpkDetect:%u,",
            uid[0], uid[1], uid[2],
            uid[3], uid[4], uid[5],
            app_flash_get_imei(), dce->name, dce->imsi, dce->sim_imei,
            __FIRMWARE_VERSION__, app_io_get_hardware_version(),
            system_get_software_reset_reason(), rtc_get_reset_reason(0), rtc_get_reset_reason(1),
            app_flash_get_total_reset_time(), app_io_get_io_value()->Value,
            app_audio_get_current_output_vol(),
            app_flash_speaker_detect_is_enable());

    int res = mqtt_publish_message("DBG", reset_info);
    free(reset_info);

    if (res >= 0)
    {
        ESP_LOGI(TAG, "mqtt send 'reset_msg' OK");
        is_sent = true;

        // Clear nguyên nhân reset trong NVS
        app_flash_write_u8(APP_FLASH_SELF_RESET_KEY, 0);
        // ESP_LOGI(TAG, "Clear nguyen nhan reset %s", err == ESP_OK ? "OK" : "ERR");
    }
}


void app_mqtt_initialize(void)
{
    /* =================== End of Init Peripheral ========================== */

    /* =================== Config MQTT ===================================== */
    char client_id[32] = {0};
    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(client_id, "VSSL_%s", app_flash_get_imei());
    }
    else
    {
        uint8_t uid[6];
        esp_efuse_mac_get_default(uid);

        sprintf(client_id, "VSSL_%02X%02X%02X%02X%02X%02X",
                uid[0], uid[1], uid[2], 
                uid[3], uid[4], uid[5]);
    }
    m_mqtt_config.client_id = client_id;
    m_mqtt_config.uri = app_flash_get_mqtt_server_url();
    m_mqtt_config.username = app_flash_get_mqtt_server_username();
    m_mqtt_config.password = app_flash_get_mqtt_server_password();
    m_mqtt_config.keepalive = APP_MQTT_KEEP_ALIVE_INTERVAL; /* seconds */
    m_mqtt_config.event_handle = mqtt_event_handler;
    m_mqtt_client = esp_mqtt_client_init(&m_mqtt_config);
    if (!m_mqtt_client)
    {
        ESP_LOGE(TAG, "esp_mqtt_client_init: ERR!");
    }
}

int mqtt_publish_message(char *header, char *msg)
{
    if (m_mqtt_client == NULL)
        return -1;
    if (m_mqtt_fsm != APP_MQTT_CONNECTED)
        return -1;

    char *topic = malloc(96);
    char *body = malloc(strlen(msg) + 96);
    if (!topic || !body)
    {
        ESP_LOGE(TAG, "Can't allocate mem!");
        if (topic)
        {
            free(topic);
        }

        if (body)
        {
            free(body);
        }
        return -1;
    }

    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, app_flash_get_imei());
    }
    else
        sprintf(topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, "NA");

    int bodylen = sprintf(body, "%s,%s", header, msg);

    int msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, bodylen, 0, 0);
    ESP_LOGI(TAG, "slave publish message OK, msg_id=%d", msg_id);

    free(topic);
    free(body);

    return msg_id;
}

static void build_http_stream_url(uint8_t master_index)
{
    if (master_index >= MASTER_TINH1 && master_index <= APP_FLASH_MASTER_XA3)
    {
        if (strlen(app_flash_get_master(master_index)) >= 15)
        {
            sprintf(app_audio_get_stream_url(), 
                    "%s%s", 
                    app_flash_get_http_stream_header(), 
                    app_flash_get_master(master_index));
        }
    }
}

static bool is_highier_priority_master(uint8_t new_master, uint8_t current_master)
{
    if (new_master < current_master)
        return true;
    return false;
}


bool process_if_higher_stream(int command_master_level, bool force_new_stream)
{
    /* 1. ================= Nếu đang Streaming ================= //
    * => Xét mức ưu tiên của master, nếu master cao hơn hiện tại -> chuyển luồng stream sang higher master
    */
    // audio_element_state_t el_i2s_state = audio_element_get_state(i2s_stream_writer);
    // audio_element_state_t el_opus_state = audio_element_get_state(opus_decoder);
    // audio_element_state_t el_http_state = audio_element_get_state(http_stream_reader);
    // ESP_LOGI(TAG, "process_if_higher_stream i2s %d, opus %d, http %d", el_i2s_state, el_opus_state, el_http_state);
    if (command_master_level != -1)
    {
        if (app_audio_is_i2s_running()
            || app_audio_is_opus_running()
            || app_audio_is_http_audio_stream_running())
        {
            // Nhận lệnh từ master cao hơn -> Stop stream master hiện tại, chuyển link sang master cao hơn
            if (force_new_stream || is_highier_priority_master(command_master_level, app_flash_get_current_streamming_master_index()))
            {
                ESP_LOGI(TAG, "[ * ] Start streaming by higher level master: %u", command_master_level);

                if (slave_get_received_running_state_timeout() == 0)
                {
                    ESP_LOGI(TAG, "Start counting running state timeout...");
                    slave_set_received_running_state_timeout(45); /* Sau 45s mà không start stream được thì reset cho nhanh */
                }
                slave_reset_stream_monitor_data();

                /* Dừng tất cả các chế độ khác -> chuyển sang chế độ codec decoder */
                app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                // Switch Relay to Codec output
                app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                // Nếu PA đang OFF thì ON
                if (app_io_is_pa_off())
                {
                    app_io_control_pa(APP_IO_PA_ON);
                }

                if (force_new_stream == false)
                {
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Start streaming by higher level master");
                    /* 2. Save the current Master */
                    app_flash_set_current_streamming_master(command_master_level);
                    app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, command_master_level);
                    build_http_stream_url(command_master_level);
                }
                else
                {
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Start streaming directly by URL");
                }
                
                app_io_opto_control_all(APP_IO_OPTO_ON);

                /* 1. Terminate pipeline */
                // audio_pipeline_pause(pipeline);
                // audio_pipeline_stop(pipeline);
                // audio_pipeline_wait_for_stop(pipeline);
                // audio_element_reset_state(opus_decoder);
                // audio_element_reset_state(mp3_decoder);
                // audio_element_reset_state(aac_decoder);
                // audio_element_reset_state(raw_decoder);
                // audio_element_reset_state(i2s_stream_writer);
                // audio_element_reset_state(http_stream_reader);
                // audio_pipeline_reset_ringbuffer(pipeline);
                // audio_pipeline_reset_items_state(pipeline);
                app_audio_simple_terminate_pipeline();
                app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_STOPPED);

                // Check if HttpElement is stopped -> try to restart new stream...
                m_is_waiting_http_element_stopped = true;
                slave_reset_counter_wait_for_http_element_stop();

                /* Thời gian chờ chuyển sang higher master streaming, bao gồm cả 5s delay sau khi stop_http_element
                    * Trong thời gian nếu nhận được lệnh Stream của master khác thì không thực thi!
                    */
                slave_set_timeout_number_of_waiting_master_streamming(30);
                slave_allow_http_element_monitor_downloaded_data_in_streamming_state(false); /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
            }
            else
            {
                ESP_LOGW(TAG, "[ * ] Streaming from lower master: %u <= %u. Ignore!", 
                            command_master_level, 
                            app_flash_get_current_streamming_master_index());
            }
            return true;
        }
        else
        {
            ESP_LOGI(TAG, "Already streaming\r\n");
        }
    }
    else
    {
        if (slave_get_received_running_state_timeout() == 0)
        {
            ESP_LOGI(TAG, "Start counting running state timeout...");
            slave_set_received_running_state_timeout(45); /* Sau 45s mà không start stream được thì reset cho nhanh */
        }
        slave_reset_stream_monitor_data();

        /* Dừng tất cả các chế độ khác -> chuyển sang chế độ codec decoder */
        app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

        // Switch Relay to Codec output
        app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

        // Nếu PA đang OFF thì ON
        if (app_io_is_pa_off())
        {
            app_io_control_pa(APP_IO_PA_ON);
        }

        mqtt_publish_message("DBG", "app_io_opto_control_all(): Start streaming directly by URL");
        
        app_io_opto_control_all(APP_IO_OPTO_ON);

        /* 1. Terminate pipeline */
        // audio_pipeline_pause(pipeline);
        // audio_pipeline_stop(pipeline);
        // audio_pipeline_wait_for_stop(pipeline);
        // audio_element_reset_state(opus_decoder);
        // audio_element_reset_state(mp3_decoder);
        // audio_element_reset_state(aac_decoder);
        // audio_element_reset_state(raw_decoder);
        // audio_element_reset_state(i2s_stream_writer);
        // audio_element_reset_state(http_stream_reader);
        // audio_pipeline_reset_ringbuffer(pipeline);
        // audio_pipeline_reset_items_state(pipeline);
        app_audio_simple_terminate_pipeline();
        app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_STOPPED);

        // Check if HttpElement is stopped -> try to restart new stream...
        m_is_waiting_http_element_stopped = true;
        slave_reset_counter_wait_for_http_element_stop();

        /* Thời gian chờ chuyển sang higher master streaming, bao gồm cả 5s delay sau khi stop_http_element
            * Trong thời gian nếu nhận được lệnh Stream của master khác thì không thực thi!
            */
        slave_set_timeout_number_of_waiting_master_streamming(30);
        slave_allow_http_element_monitor_downloaded_data_in_streamming_state(false); /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
    }
    return false;
}

bool app_mqtt_is_subscribed(void)
{
    return m_is_mqtt_subscribed;
}

esp_err_t app_mqtt_start(void)
{
    return esp_mqtt_client_start(m_mqtt_client);
}


/******************************************************************************************/
/**
 * @brief   : Lưu cấu hình WiFi từ lệnh MQTT
 * @param   : data: WIFINAME(name),PASS(pass),ENABLE:1
 * @retval  :
 * @author  :
 * @created :
 */
bool do_reconnect_wifi = false;
static void mqtt_write_wifi_config(char *data)
{
    // WIFINAME(name),PASS(pass),ENABLE:1

    char wName[28] = {0};
    char wPass[28] = {0};
    uint8_t wEnable = 0;
    char *name = strstr(data, "WIFINAME(");
    uint32_t same_wifi = 0;
    if (name != NULL)
    {
        if (CopyParameter(name, wName, '(', ')'))
        {
            if (strcmp(app_flash_get_wifi_name(), wName))
            {
                ESP_LOGI(TAG, "New wiFi name: %s", wName);
                sprintf(app_flash_get_wifi_name(), "%s", wName);
                do_reconnect_wifi = true;
            }
            else
            {
                same_wifi++;
            }
        }
    }
    char *pass = strstr(data, "PASS(");
    if (pass != NULL)
    {
        if (CopyParameter(pass, wPass, '(', ')'))
        {
            if (strcmp(app_flash_get_wifi_pass(), wPass))
            {
                ESP_LOGI(TAG, "New wiFi pass: %s", wPass);
                sprintf(app_flash_get_wifi_pass(), "%s", wPass);
                do_reconnect_wifi = true;
            }
            else
            {
                same_wifi++;
            }
        }
    }
    char *enable = strstr(data, "ENABLE:");
    if (enable != NULL)
    {
        wEnable = GetNumberFromString(7, enable);
        ESP_LOGI(TAG, "WiFi enable: %d", wEnable);
        if (app_flash_is_wifi_enable() != wEnable)
        {
            same_wifi = 0;
        }

        app_flash_wifi_enable(wEnable);
        if (!app_flash_is_wifi_enable())
        {
            do_reconnect_wifi = false;
        }
    }
    esp_err_t err = ESP_OK;
    if (!same_wifi)
    {
        app_flash_write_wifi_info(app_flash_get_wifi_name(),
                                app_flash_get_wifi_pass(), wEnable);
    }
    ESP_LOGI(TAG, "Write wifi config to NVS: %s", err == ESP_OK ? "OK" : "ERR");

    if (err == ESP_OK && same_wifi != 2)
    {
        ESP_LOGI(TAG, "Reset system...");
        app_audio_change_codec_vol(0);
        app_audio_simple_terminate_pipeline();
        vTaskDelay(1000 / portTICK_RATE_MS);
        system_software_reset(SW_RESET_REASON_CONFIG_WIFI);
    }
    else if (err == ESP_OK && same_wifi == 2)
    {
        ESP_LOGW(TAG, "Save WiFi");
        mqtt_publish_message("DBG", "Same wifi username and password, no need to change");
    }
    else
    {
        ESP_LOGE(TAG, "Save WiFi config ERROR!");
        mqtt_publish_message("DBG", "Config WiFi ERR");
    }
}

/******************************************************************************************/
/**
 * @brief   : Xử lý lệnh SET cấu hình
 * @param   : data: SET,Lệnh 1,Lệnh 2,....,Lệnh N
 * @retval  :
 * @author  :
 * @created :
 */
static void mqttProcessSetConfig(char *cfgData)
{
    char tempStr[80] = {0};
    char *pResponse = malloc(256);
    uint16_t index = 0;
    uint8_t hasNewUrl = 0;

    ESP_LOGI(TAG, "Config: %s", cfgData);

    if (!pResponse)
    {
        ESP_LOGE(TAG, "Can't alocate mem pResponse!");
        return;
    }

#if 1
    // Lệnh cấu hình http stream url
    char *streamUrl = strstr(cfgData, "STREAM_URL(");
    if (streamUrl != NULL)
    {
        memset(tempStr, 0, sizeof(tempStr));
        if (CopyParameter(streamUrl, tempStr, '(', ')'))
        {
            ESP_LOGI(TAG, "Set URL header: %s", tempStr);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_http_stream_header(), tempStr) != 0)
            {
                sprintf(app_flash_get_http_stream_header(), "%s", tempStr);

                ESP_LOGI(TAG, "New URL header: %s", app_flash_get_http_stream_header());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_HTTP_URL_HEADER_KEY, app_flash_get_http_stream_header());
                index += sprintf(&pResponse[index], "STREAM_URL=%s,", app_flash_get_http_stream_header());

                hasNewUrl = 1;
            }
            else
            {
                ESP_LOGI(TAG, "Same URL header: %s", app_flash_get_http_stream_header());
                index += sprintf(&pResponse[index], "STREAM_URL=%s,", app_flash_get_http_stream_header());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }
#endif

    // SET,MQTT_URL(mqtt://smart.radiotech.vn:1883),MQTT_USER(village-speaker),MQTT_PASS(vs.bytech@2019)
    char *mqtt_url = strstr(cfgData, "MQTT_URL(");
    if (mqtt_url != NULL)
    {
        memset(tempStr, 0, sizeof(tempStr));
        if (CopyParameter(mqtt_url, tempStr, '(', ')'))
        {
            ESP_LOGI(TAG, "Set URL header: %s", tempStr);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_mqtt_server_url(), tempStr) != 0)
            {
                memset(app_flash_get_mqtt_server_url(), 0, APP_FLASH_MQTT_URL_SIZE);
                snprintf(app_flash_get_mqtt_server_url(), APP_FLASH_MQTT_URL_SIZE, "%s", tempStr);

                ESP_LOGI(TAG, "New mqtt url: %s", app_flash_get_mqtt_server_url());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_URL, app_flash_get_mqtt_server_url());
                index += sprintf(&pResponse[index], "MQTT_URL=%s,", app_flash_get_mqtt_server_url());
            }
            else
            {
                ESP_LOGI(TAG, "Same URL header: %s", app_flash_get_mqtt_server_url());
                index += sprintf(&pResponse[index], "MQTT_URL=%s,", app_flash_get_mqtt_server_url());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }


    char *mqtt_username = strstr(cfgData, "MQTT_USER(");
    if (mqtt_username != NULL)
    {
        memset(tempStr, 0, sizeof(tempStr));
        if (CopyParameter(mqtt_username, tempStr, '(', ')'))
        {
            ESP_LOGI(TAG, "Set URL header: %s", tempStr);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_mqtt_server_username(), tempStr) != 0)
            {
                memset(app_flash_get_mqtt_server_username(), 0, APP_FLASH_MQTT_USERNAME_SIZE);
                snprintf(app_flash_get_mqtt_server_username(), APP_FLASH_MQTT_USERNAME_SIZE, "%s", tempStr);

                ESP_LOGI(TAG, "New mqtt username: %s", app_flash_get_mqtt_server_username());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_USERNAME, app_flash_get_mqtt_server_username());
                index += sprintf(&pResponse[index], "MQTT_USER=%s,", app_flash_get_mqtt_server_username());
            }
            else
            {
                ESP_LOGI(TAG, "Same URL username: %s", app_flash_get_mqtt_server_username());
                index += sprintf(&pResponse[index], "MQTT_USER=%s,", app_flash_get_mqtt_server_username());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }

    char *mqtt_pass = strstr(cfgData, "MQTT_PASS(");
    if (mqtt_pass != NULL)
    {
        memset(tempStr, 0, sizeof(tempStr));
        if (CopyParameter(mqtt_pass, tempStr, '(', ')'))
        {
            ESP_LOGI(TAG, "Set URL header: %s", tempStr);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_mqtt_server_password(), tempStr) != 0)
            {
                memset(app_flash_get_mqtt_server_password(), 0, APP_FLASH_MQTT_PASSWORD_SIZE);
                snprintf(app_flash_get_mqtt_server_password(), APP_FLASH_MQTT_PASSWORD_SIZE, "%s", tempStr);

                ESP_LOGI(TAG, "New mqtt pass: %s", app_flash_get_mqtt_server_password());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_PASSWORD, app_flash_get_mqtt_server_password());
                index += sprintf(&pResponse[index], "MQTT_PASS=%s,", app_flash_get_mqtt_server_password());
            }
            else
            {
                ESP_LOGI(TAG, "Same mqtt pass: %s", app_flash_get_mqtt_server_password());
                index += sprintf(&pResponse[index], "MQTT_PASS=%s,", app_flash_get_mqtt_server_password());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }

    // Mod: Lệnh cấu hình Master sẽ gửi nhóm IMEI: MASTER1(imei1|imei2|imei3)
    char *master1 = strstr(cfgData, "MASTER1(");
    if (master1 != NULL)
    {
        memset(tempStr, 0, sizeof(tempStr));

        if (CopyParameter(master1, tempStr, '(', ')'))
        {
            ESP_LOGI(TAG, "Set MASTER1: %s", tempStr);

            char listImeiT[4][25];
            uint8_t listImeiIndex = 0;
            char *mToken = strtok(tempStr, "|");
            while (mToken != NULL)
            {
                snprintf(listImeiT[listImeiIndex++], 25, "%s", mToken);
                mToken = strtok(NULL, "|");

                if (listImeiIndex >= 3)
                    break;
            }

            for (uint8_t i = 0; i < 3; i++)
            {
                if (IsDigitString(listImeiT[i]) && strlen(listImeiT[i]) >= 3)
                { /* Cho phép clear imei master = "000" */
                    memset(app_flash_get_master(MASTER_TINH1 + i), 0, APP_FLASH_MASTER_IMEI_SIZE);
                    memcpy(app_flash_get_master(MASTER_TINH1 + i), listImeiT[i], strlen(listImeiT[i]));

                    // Write to NVS
                    if (i == 0)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_T1_KEY, app_flash_get_master(MASTER_TINH1));
                        index += sprintf(&pResponse[index], "MASTER1=%s,", app_flash_get_master(MASTER_TINH1));
                    }
                    else if (i == 1)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_T2_KEY, app_flash_get_master(MASTER_TINH2));
                        index += sprintf(&pResponse[index], "MASTER1_2=%s,", app_flash_get_master(MASTER_TINH2));
                    }
                    else if (i == 2)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_T3_KEY, app_flash_get_master(MASTER_TINH3));
                        index += sprintf(&pResponse[index], "MASTER1_3=%s,", app_flash_get_master(MASTER_TINH3));
                    }

                    // Send subscribe master1
                    app_mqtt_send_subscribe_command_topic(MASTER_TINH1 + i);
                }
            }
        }
    }
    char *master2 = strstr(cfgData, "MASTER2(");
    if (master2 != NULL)
    {
        memset(tempStr, 0, sizeof(tempStr));

        if (CopyParameter(master2, tempStr, '(', ')'))
        {
            ESP_LOGI(TAG, "Set MASTER2: %s", tempStr);

            char listImeiH[4][25];
            uint8_t listImeiIndex = 0;
            char *mToken = strtok(tempStr, "|");
            while (mToken != NULL)
            {
                snprintf(listImeiH[listImeiIndex++], 25, "%s", mToken);
                mToken = strtok(NULL, "|");

                if (listImeiIndex >= 3)
                    break;
            }

            for (uint8_t i = 0; i < 3; i++)
            {
                if (IsDigitString(listImeiH[i]) && strlen(listImeiH[i]) >= 3)
                { /* Cho phép clear imei master = "000" */
                    memset(app_flash_get_master(APP_FLASH_MASTER_HUYEN1 + i), 0, APP_FLASH_MASTER_IMEI_SIZE);
                    memcpy(app_flash_get_master(APP_FLASH_MASTER_HUYEN1 + i), listImeiH[i], strlen(listImeiH[i]));

                    // Write to NVS
                    if (i == 0)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_H1_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
                        index += sprintf(&pResponse[index], "MASTER2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
                    }
                    else if (i == 1)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_H2_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN2));
                        index += sprintf(&pResponse[index], "MASTER2_2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN2));
                    }
                    else if (i == 2)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_H3_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN3));
                        index += sprintf(&pResponse[index], "MASTER2_3=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN3));
                    }

                    // Send subscribe master1
                    app_mqtt_send_subscribe_command_topic(APP_FLASH_MASTER_HUYEN1 + i);
                }
            }
        }
    }
    char *master3 = strstr(cfgData, "MASTER3(");
    if (master3 != NULL)
    {
        memset(tempStr, 0, sizeof(tempStr));

        if (CopyParameter(master3, tempStr, '(', ')'))
        {
            ESP_LOGI(TAG, "Set MASTER3: %s", tempStr);

            char listImeiX[4][25];
            uint8_t listImeiIndex = 0;
            char *mToken = strtok(tempStr, "|");
            while (mToken != NULL)
            {
                snprintf(listImeiX[listImeiIndex++], 25, "%s", mToken);
                mToken = strtok(NULL, "|");

                if (listImeiIndex >= 3)
                    break;
            }

            for (uint8_t i = 0; i < 3; i++)
            {
                if (IsDigitString(listImeiX[i]) && strlen(listImeiX[i]) >= 3)
                { /* Cho phép clear imei master = "000" */
                    memset(app_flash_get_master(APP_FLASH_MASTER_XA1 + i), 0, APP_FLASH_MASTER_IMEI_SIZE);
                    memcpy(app_flash_get_master(APP_FLASH_MASTER_XA1 + i), listImeiX[i], strlen(listImeiX[i]));

                    // Write to NVS
                    if (i == 0)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_X1_KEY, app_flash_get_master(APP_FLASH_MASTER_XA1));
                        index += sprintf(&pResponse[index], "MASTER3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA1));
                    }
                    else if (i == 1)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_X2_KEY, app_flash_get_master(APP_FLASH_MASTER_XA2));
                        index += sprintf(&pResponse[index], "MASTER3_2=%s,", app_flash_get_master(APP_FLASH_MASTER_XA2));
                    }
                    else if (i == 2)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_X3_KEY, app_flash_get_master(APP_FLASH_MASTER_XA3));
                        index += sprintf(&pResponse[index], "MASTER3_3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA3));
                    }

                    // Send subscribe master1
                    app_mqtt_send_subscribe_command_topic(APP_FLASH_MASTER_XA1 + i);
                }
            }
        }
    }

    char *fmFreq1 = strstr(cfgData, "FM_FREQ1("); /** Tần số đài Tỉnh/Trung ương */
    if (fmFreq1 != NULL)
    {
        uint32_t freq = GetNumberFromString(9, fmFreq1);
        if (freq > 0)
        {
            ESP_LOGI(TAG, "Set FM_FREQ1: %u", freq);
            app_flash_set_fm_freq1(freq); /* KHz */

            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ1_KEY, freq);
        }
    }
    char *fmFreq2 = strstr(cfgData, "FM_FREQ2("); /** Tần số đài huyện */
    if (fmFreq2 != NULL)
    {
        uint32_t freq = GetNumberFromString(9, fmFreq2);
        if (freq > 0)
        {
            ESP_LOGI(TAG, "Set FM_FREQ2: %u", freq);
            app_flash_set_fm_freq2(freq); /* KHz */

            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ2_KEY, freq);
        }
    }
    char *fmFreq3 = strstr(cfgData, "FM_FREQ3("); /** Tần số đài xã */
    if (fmFreq3 != NULL)
    {
        uint32_t freq = GetNumberFromString(9, fmFreq3);
        if (freq > 0)
        {
            ESP_LOGI(TAG, "Set FM_FREQ3: %u", freq);
            app_flash_set_fm_freq3(freq); /* KHz */

            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ3_KEY, freq);
        }
    }

    char *volume = strstr(cfgData, "VOLUME(");
    if (volume != NULL)
    {
        uint8_t vol = GetNumberFromString(7, volume);
        if (vol > 100)
            vol = 100;

        ESP_LOGI(TAG, "Set VOLUME: %u", vol);
        app_flash_set_volume(vol); /* 0 - 100 */

        // Write to NVS
        app_flash_write_u8(APP_FLASH_VOLUME_KEY, vol);

        // Set volume cho audio codec
        uint8_t retVol = app_audio_change_codec_vol(app_flash_get_volume());

        // Add to response
        index += sprintf(&pResponse[index], "VOLUME=%u,VOLUME_CODEC=%u,", app_flash_get_volume(), retVol);
    }

    /* Relay delay time */
    char *relayDelayOn = strstr(cfgData, "DELAY_ON(");
    if (relayDelayOn != NULL)
    {
        uint8_t delayOn = GetNumberFromString(9, relayDelayOn);

        ESP_LOGI(TAG, "Set Relay delay on: %u", delayOn);

        if (delayOn > 0 && delayOn <= APP_FLASH_RELAY_DELAY_MAX_TIME)
        { /* 1 - 250 */
            app_flash_set_delay_turn_on_relay(delayOn);

            // Write to NVS
            app_flash_write_u8(APP_FLASH_RELAY_DELAY_ON_KEY, delayOn);

            // Add to response
            if (pResponse)
            {
                index += sprintf(&pResponse[index], "DELAY_ON=%u,", app_flash_get_delay_turn_on_relay());
            }
        }
    }
    char *relay1DelayOff = strstr(cfgData, "DELAY_OFF1(");
    if (relay1DelayOff != NULL)
    {
        uint8_t delayOff1 = GetNumberFromString(11, relay1DelayOff);

        ESP_LOGI(TAG, "Set Relay1 delay off: %u", delayOff1);

        if (delayOff1 > 0 && delayOff1 <= APP_FLASH_RELAY_DELAY_MAX_TIME)
        { /* 1 - 250 */
            app_flash_set_relay1_turn_off_delay(delayOff1);

            // Write to NVS
            app_flash_write_u8(APP_FLASH_RELAY1_DELAY_OFF_KEY, delayOff1);

            // Add to response
            if (pResponse)
            {
                index += sprintf(&pResponse[index], "DELAY_OFF1=%u,", app_flash_get_relay1_turn_off_delay());
            }
        }
    }
    char *relay2DelayOff = strstr(cfgData, "DELAY_OFF2(");
    if (relay2DelayOff != NULL)
    {
        uint16_t delayOff2 = GetNumberFromString(11, relay2DelayOff);

        ESP_LOGI(TAG, "Set Relay2 delay off: %u", delayOff2);

        if (delayOff2 > 0 && delayOff2 <= APP_FLASH_RELAY2_DELAY_MAX_TIME)
        { /* 1 - 250 */
            app_flash_set_relay2_turn_off_delay(delayOff2);

            // Write to NVS
            app_flash_write_u16(APP_FLASH_RELAY2_DELAY_OFF_KEY, delayOff2);

            // Add to response
            if (pResponse)
            {
                index += sprintf(&pResponse[index], "DELAY_OFF2=%u,", app_flash_get_relay2_turn_off_delay());
            }
        }
    }

    char *opMode = strstr(cfgData, "MODE(");
    if (opMode != NULL)
    {
        uint8_t mode = GetNumberFromString(5, opMode);

        ESP_LOGI(TAG, "Set MODE: %u", mode);

        /* self limit mode range */
        if (mode >= APP_AUDIO_OPERATION_MODE_INTERNET && mode <= APP_AUDIO_OPERATION_MODE_NO_OPERATION)
        {
            
            /* Xử lý chuyển audio mode */
            if (mode == APP_AUDIO_OPERATION_MODE_INTERNET)
            {
                app_flash_set_operate_mode(mode); /* 1 - 4 */
                // Write to NVS
                app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);
                // Add to response
                index += sprintf(&pResponse[index], "MODE=%u,", app_flash_get_operate_mode());
                index += sprintf(&pResponse[index], "MASTER=%s,", app_flash_get_master(app_flash_get_current_streamming_master_index()));

                // Change audio codec mode
                app_audio_change_codec_to_internet_mode();

                // Tạm thời tắt PA luôn, nếu sau có lệnh stream thì tự bật
                app_io_control_pa(APP_IO_PA_OFF);

                // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                ESP_LOGI(TAG, "app_io_opto_control_all OFF...");
                app_io_opto_control_all(APP_IO_OPTO_OFF);

                // Khi chuyển sang mode INTERNET thì FM_BUSY
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                // Chuyển sang Internet -> Gửi phản hồi mạch FM vài lần để về Idle
                app_io_set_number_of_retries_counter(3);

                if (app_audio_get_streamming_logic_step() == APP_AUDIO_STREAM_STOPPED)
                {
                    app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_RESTART);
                    build_http_stream_url(app_flash_get_current_streamming_master_index());
                    mqtt_publish_message("DBG", "Mode change, start stream");
                    ESP_LOGI(TAG, "streamingStep Restart...");
                }
            }
            else if (mode == APP_AUDIO_OPERATION_MODE_MIC || mode == APP_AUDIO_OPERATION_MODE_LINE)
            {
                /* Nếu đang không stream internet hoặc phát FM thì mới chuyển sang MIC/LINE IN */
                if (!app_audio_is_i2s_running()
                    && !app_audio_is_opus_running()
                    && !app_audio_is_http_audio_stream_running())
                {
                    ESP_LOGI(TAG, "Switch to MIC/LINE mode");

                    app_flash_set_operate_mode(mode);   /* 1 - 5 */
                    app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY); // Không cho thay đổi Volume từ bên module FM

                    // Write to NVS
                    app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);
                    // Add to response
                    index += sprintf(&pResponse[index], "MODE=%u,", app_flash_get_operate_mode());

                    /** Change codec mode and input/output relay */
                    if (mode == APP_AUDIO_OPERATION_MODE_MIC)
                    {
                        /* Change mode to MIC, PA ON */
                        app_audio_change_to_local_mic_mode();
                    }
                    else if (mode == APP_AUDIO_OPERATION_MODE_LINE)
                    {
                        /* Change mode to LINE IN */
                        app_audio_change_to_local_line_in_mode();
                    }

                    // Bật 2 Relay ngoài
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to MIC/LINE mode");
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
                else
                {
                    ESP_LOGI(TAG, "HTTP is running, can't run MIC/LINE mode!");
                }
            }
            else if (mode == APP_AUDIO_OPERATION_MODE_FM)
            {
                // Lấy tần số kênh FM được chọn
                char *fmChannelSelected = strstr(cfgData, "FM_FREQ(");

                if (fmChannelSelected != NULL)
                {
                    uint32_t curFMChannel = GetNumberFromString(8, fmChannelSelected);
                    if (curFMChannel >= 1 && curFMChannel <= 3)
                    {
                        if (curFMChannel == 1)
                            app_io_set_current_fm_freq(app_flash_get_fm_freq1());
                        if (curFMChannel == 2)
                            app_io_set_current_fm_freq(app_flash_get_fm_freq2());
                        if (curFMChannel == 3)
                            app_io_set_current_fm_freq(app_flash_get_fm_freq3());

                        /* Nếu đang không stream internet thì mới cho chạy FM */
                        if (!app_audio_is_i2s_running()
                            && !app_audio_is_opus_running()
                            && !app_audio_is_http_audio_stream_running())
                        {
                            ESP_LOGI(TAG, "Switch to FM mode, freq: %u", app_io_get_current_fm_freq());

                            /** Change to FM mode, PA ON */
                            app_audio_change_to_local_fm_mode();

                            // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                            ESP_LOGI(TAG, "app_io_opto_control_all OFF...");
                            app_io_opto_control_all(APP_IO_OPTO_OFF);

                            app_flash_set_operate_mode(mode); /* 1 - 5 */
                            app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                            // Write mode to NVS
                            app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);

                            // Write current freq to NVS
                            app_flash_node_nvs_write_u32(APP_FLASH_CURRENT_FREQ_KEY, app_io_get_current_fm_freq());

                            // Add to response: Gửi giá trị tần số thực luôn
                            index += sprintf(&pResponse[index], "FM_FREQ=%.1f,", (float)app_io_get_current_fm_freq()/ 1000); // MHz
                        }
                        else
                        {
                            ESP_LOGI(TAG, "HTTP is running, can't run FM mode!");
                        }
                    }
                    // Add to response
                    index += sprintf(&pResponse[index], "MODE=%u,", app_flash_get_operate_mode());
                }
            }
            else if (mode == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
            {
                ESP_LOGI(TAG, "Stop audio by set 'APP_AUDIO_OPERATION_MODE_NO_OPERATION'");

                app_flash_set_operate_mode(mode); /* 1 - 4 */
                // Write to NVS
                app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);
                app_audio_pause();
                app_audio_complete_terminate();

                app_audio_set_http_stream_stopped_flag();

                // Change codec to INTERNET mode
                app_audio_change_codec_to_internet_mode();

                // Turn off PA
                ESP_LOGI(TAG, "app_io_opto_control_all OFF...");
                app_io_control_pa(APP_IO_PA_OFF);

                // Tắt Relay (nếu được bật ở chế độ LINE IN)
                app_io_opto_control_all(APP_IO_OPTO_OFF);

                // Add to response
                index += sprintf(&pResponse[index], "MODE=%u,", app_flash_get_operate_mode());
            }
        }
    }

    bool isIOConfig = false;
    if (strstr(cfgData, "IO1(ON)"))
    {
        isIOConfig = true;
        app_io_get_io_value()->Name.IO1 = IO_ON;

        // Add to response
        index += sprintf(&pResponse[index], "%s", "IO1=ON,");

        // Control relay
        app_io_control_opto_output1(APP_IO_OPTO_ON);
    }
    if (strstr(cfgData, "IO1(OFF)"))
    {
        isIOConfig = true;
        app_io_get_io_value()->Name.IO1 = IO_OFF;

        // Add to response
        index += sprintf(&pResponse[index], "%s", "IO1=OFF,");

        // Control relay
        app_io_control_opto_output1(APP_IO_OPTO_OFF);
    }

    if (strstr(cfgData, "SPK_DETECT(OFF)"))
    {
        ESP_LOGI(TAG, "Speaker off");
        if (app_flash_speaker_detect_is_enable())
        {
            app_flash_write_u8(APP_FLASH_SPK_DETECT, 0);
            app_flash_speaker_detect_set_enable(0);
        }
    }
    else if (strstr(cfgData, "SPK_DETECT(ON)"))
    {
        ESP_LOGI(TAG, "Speaker on");
        if (!app_flash_speaker_detect_is_enable())
        {
            app_flash_write_u8(APP_FLASH_SPK_DETECT, 1);
            app_flash_speaker_detect_set_enable(1);
        }
    }
    if (strstr(cfgData, "IO2(ON)"))
    {
        isIOConfig = true;
        app_io_get_io_value()->Name.IO2 = IO_ON;

        // Add to response
        index += sprintf(&pResponse[index], "%s", "IO2=ON,");

        // Control relay
        app_io_control_opto_output2(APP_IO_OPTO_ON);
    }
    if (strstr(cfgData, "IO2(OFF)"))
    {
        isIOConfig = true;
        app_io_get_io_value()->Name.IO2 = IO_OFF;

        // Add to response
        index += sprintf(&pResponse[index], "%s", "IO2=OFF,");

        // Control relay
        app_io_control_opto_output2(APP_IO_OPTO_OFF);
    }
    if (isIOConfig)
    {
        // Write to NVS
        app_flash_write_u8(APP_FLASH_IO_STATE_KEY, app_io_get_io_value()->Value);
    }

    // TCP console
    if (strstr(cfgData, "CONSOLE(ON)"))
    {
        ESP_LOGI(TAG, "TCP console enable");
    }
    else if (strstr(cfgData, "CONSOLE(OFF)"))
    {
        ESP_LOGI(TAG, "TCP console disable");
    }

    /** Mặc định phản hồi các tần số cấu hình để trên web có danh sách tần số */
    index += sprintf(&pResponse[index], "FM_FREQ1=%u,FM_FREQ2=%u,FM_FREQ3=%u,SpeakerDetect:%u,",
                     app_flash_get_fm_freq1(), 
                     app_flash_get_fm_freq2(), 
                     app_flash_get_fm_freq3(), 
                     app_flash_speaker_detect_is_enable());

    /* Gửi phản hồi kết quả cấu hình */
    if (index > 0)
    {
        // Nếu là bản tin config url -> chỉ gửi vài lần mỗi khi kết nối
        if (streamUrl == NULL || (streamUrl != NULL && m_mqtt_send_resp_config_url_num <= 3))
        {
            ESP_LOGI(TAG, "SET reply: %s", pResponse);
            mqtt_publish_message("CFG", pResponse);
        }
    }

    /* Nếu có cấu hình url mới */
    if (hasNewUrl > 0)
    {
        /** Lấy trạng thái các audio_element (nếu đã khởi tạo, vì lúc mới khởi động có thể chưa khởi tạo) */
        /* 1. ================= Nếu đang Streaming ================= //
            * stop stream, sau đó nhận được lệnh stream từ master nào thì play với link mới
            */
        if (app_audio_is_i2s_running() 
            || app_audio_is_opus_running()
            || app_audio_is_http_audio_stream_running()
            )
        {
            ESP_LOGW(TAG, "[ * ] Stop streaming old url now...");
            slave_reset_stream_monitor_data();

            /* 1. Init new app_audio_get_stream_url() */
            build_http_stream_url(app_flash_get_current_streamming_master_index());

            /* 2. Terminate pipeline */
            app_audio_simple_terminate_pipeline();
            app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_STOPPED);

            // check if HttpElement is stopped -> try to restart new stream...
            m_is_waiting_http_element_stopped = true;
            slave_reset_counter_wait_for_http_element_stop();

            /* Thời gian chờ chuyển sang higher master streaming, bao gồm cả 5s delay sau khi stop_http_element
                * Trong thời gian nếu nhận được lệnh Stream của master khác thì không thực thi!
                */
            slave_set_timeout_number_of_waiting_master_streamming(30);
            slave_allow_http_element_monitor_downloaded_data_in_streamming_state(false); /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
        }

        // reboot cho mát
        // ESP_LOGI(TAG, "Reboot to apply new URL...");
        // system_software_reset(SW_RESET_REASON_CHANGE_STREAM_URL);
    }

    /* Free mem */
    free(pResponse);
}

static void process_stream_by_url(char *stream_url)
{
    if (strstr(stream_url, app_audio_get_stream_url()))
    {
        ESP_LOGW(TAG, "Same URL\r\n");
        mqtt_publish_message("STREAM_BY_URL", "Same url");
    }
    else
    {
        char *p = strtok(stream_url, ")");
        strcpy(app_audio_get_stream_url(), p);
        ESP_LOGI(TAG, "Test stream reply: %s", app_audio_get_stream_url());
        mqtt_publish_message("STREAM_BY_URL", app_audio_get_stream_url());
        process_if_higher_stream(-1, true);
    }
}

/******************************************************************************************/
/**
 * @brief   : Thực hiện đóng relay theo thứ tự, thời gian delay đồng bộ theo delay của master
 * @param   :   STREAM_PREPAIR(delayTime)
 * @retval  : None
 * @author  :
 * @created : 19/12/20
 */
void slave_process_prepair_onair(char *prepairMsg)
{
    // // Lấy thời gian Prepair delay từ master (nếu có)
    // char *prepairMsg = strstr(msg, "STREAM_PREPAIR");
    if (prepairMsg != NULL)
    {
        char delayString[10] = {0};
        if (CopyParameter(prepairMsg, delayString, '(', ')'))
        {
            slave_set_delay_turn_on_relay_prepare_stream((uint8_t)GetNumberFromString(0, delayString));
            ESP_LOGI(TAG, "Prepair delay from master = %d", slave_get_delay_turn_on_relay_prepare_stream());
        }
        else
        {
            slave_set_delay_turn_on_relay_prepare_stream(app_flash_get_delay_turn_on_relay());
            ESP_LOGI(TAG, "Not found prepair delay fom master, use default = %d", slave_get_delay_turn_on_relay_prepare_stream());
        }
    }

    // Thêm điều khiển ON relay theo thứ tự relay1 -> delay -> relay2
    if (!app_audio_is_http_audio_stream_running())
    {
        mqtt_publish_message("DBG", "app_io_control_opto_output1(): slave_process_prepair_onair");
    }
    app_io_control_opto_output1(APP_IO_OPTO_ON);
}

/******************************************************************************************/
/**
 * @brief   : Xử lý lệnh GET cấu hình
 * @param   : data: GET,Lệnh 1,Lệnh 2,....,Lệnh N
 * @retval  :
 * @author  :
 * @created :
 */
static void process_get_config(char *cfgData)
{
    char *pResponse = malloc(512+256);
    uint16_t index = 0;

    if (!pResponse)
    {
        ESP_LOGE(TAG, "Can't alocate mem pResponse!");
        return;
    }

    bool isGetAll = false;
    if (strstr(cfgData, "ALL"))
    {
        isGetAll = true;
    }

    if (strstr(cfgData, "MASTER") || isGetAll)
    {
        if (strlen(app_flash_get_master(MASTER_TINH1)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER1=%s,", app_flash_get_master(MASTER_TINH1));
        }
        if (strlen(app_flash_get_master(MASTER_TINH2)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER1_2=%s,", app_flash_get_master(MASTER_TINH2));
        }
        if (strlen(app_flash_get_master(MASTER_TINH3)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER1_3=%s,", app_flash_get_master(MASTER_TINH3));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN1)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN2)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER2_2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN3)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER2_3=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN3));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA1)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA2)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER3_2=%s,", app_flash_get_master(APP_FLASH_MASTER_XA2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA3)) >= 3)
        {
            index += sprintf(&pResponse[index], "MASTER3_3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA3));
        }
    }
    if (strstr(cfgData, "FM_FREQ") || isGetAll)
    {
        index += sprintf(&pResponse[index], "FM_FREQ=%u,", app_io_get_current_fm_freq());
    }
    if (strstr(cfgData, "FM_FREQ1") || isGetAll)
    {
        index += sprintf(&pResponse[index], "FM_FREQ1=%u,", app_flash_get_fm_freq1());
    }
    if (strstr(cfgData, "FM_FREQ2") || isGetAll)
    {
        index += sprintf(&pResponse[index], "FM_FREQ2=%u,", app_flash_get_fm_freq2());
    }
    if (strstr(cfgData, "FM_FREQ3") || isGetAll)
    {
        index += sprintf(&pResponse[index], "FM_FREQ3=%u,", app_flash_get_fm_freq3());
    }

    if (strstr(cfgData, "VOLUME") || isGetAll)
    {
        index += sprintf(&pResponse[index], "VOLUME=%u,VOLUME_CODEC=%u,", 
                        app_flash_get_volume(), 
                        app_audio_get_current_output_vol());
    }
    if (strstr(cfgData, "MODE") || isGetAll)
    {
        index += sprintf(&pResponse[index], "MODE=%u,", app_flash_get_operate_mode());
    }
    if (strstr(cfgData, "IO1") || isGetAll)
    {
        index += sprintf(&pResponse[index], "IO1=%s,", app_io_get_io_value()->Name.IO1 ? "ON" : "OFF");
    }
    if (strstr(cfgData, "IO2") || isGetAll)
    {
        index += sprintf(&pResponse[index], "IO2=%s,", app_io_get_io_value()->Name.IO2 ? "ON" : "OFF");
    }
    if (strstr(cfgData, "DELAY_ON") || isGetAll)
    {
        index += sprintf(&pResponse[index], "DELAY_ON=%u,", app_flash_get_delay_turn_on_relay());
    }
    if (strstr(cfgData, "DELAY_OFF") || isGetAll)
    {
        index += sprintf(&pResponse[index], "DELAY_OFF1=%u,DELAY_OFF2=%u,", 
                        app_flash_get_relay1_turn_off_delay(), 
                        app_flash_get_relay2_turn_off_delay());
    }
    if (strstr(cfgData, "WIFI") || isGetAll)
    {
        index += sprintf(&pResponse[index], "WIFI=%s:%s,", app_flash_get_wifi_name(), app_flash_get_wifi_pass());
    }

    // last config
    if (strstr(cfgData, "STREAM") || isGetAll)
    {
        index += sprintf(&pResponse[index], "STREAM_URL=%s,", app_flash_get_http_stream_header());
    }

    if (strstr(cfgData, "mqttInfo") || isGetAll)
    {
        index += sprintf(&pResponse[index], "MQTT_SERVER=%s,", app_flash_get_mqtt_server_url());
        index += sprintf(&pResponse[index], "MQTT_USERNAME=%s,", app_flash_get_mqtt_server_username());
        index += sprintf(&pResponse[index], "MQTT_PASSWORD=%s,", app_flash_get_mqtt_server_password());
    }

    /* Gửi phản hồi kết quả lấy cấu hình */
    if (index > 0)
    {
        ESP_LOGI(TAG, "GET reply: %s", pResponse);
        mqtt_publish_message("CFG", pResponse);
    }

    /* Free mem */
    free(pResponse);
}


static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    // esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        app_mqtt_set_state(APP_MQTT_CONNECTED);
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        ESP_LOGI(TAG, "[ZIG] SERVER: CONNECTED\r\n");

        // Send subscribe 'config_topic'
        app_mqtt_send_subscribe_config_topic(app_flash_get_imei());

        slave_set_mqtt_state_timeout(56); /* Publish info sau khi connected 5s */
        break;

    case MQTT_EVENT_DISCONNECTED:
        app_mqtt_set_state(APP_MQTT_DISCONNECTED);
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        ESP_LOGI(TAG, "[ZIG] SERVER: DISCONNECTED\r\n");
        m_mqtt_sub_req_tick = 0;
        m_is_mqtt_subscribed = false;
        m_mqtt_send_resp_config_url_num = 0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        // ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        if (event->msg_id == m_mqtt_sub_cfg_msg_id)
        {
            // ESP_LOGI(TAG, "MQTT subscribe CONFIG topic: OK");
            m_is_mqtt_subscribed = true;

            // Send subscribe 'command_topic'
            app_mqtt_send_subscribe_command_topic(MASTER_TINH1);
        }
        else
        {
            for (uint8_t i = MASTER_TINH1; i < MASTER_TOTAL; i++)
            {
                if (event->msg_id == m_mqtt_sub_master_msg_id[i])
                {
                    // ESP_LOGI(TAG, "MQTT subscribe MASTER%d: OK", i);
                    m_mqtt_is_master_subscribed[i] = true;

                    // Send sub master tiếp theo
                    uint8_t next_master = i + 1;
                    if (next_master < MASTER_TOTAL && !m_mqtt_is_master_subscribed[next_master])
                    {
                    }
                        app_mqtt_send_subscribe_command_topic(next_master);
                    break;
                }
            }
        }
        m_mqtt_sub_req_tick = 0;
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        m_is_mqtt_subscribed = false;
        m_mqtt_send_resp_config_url_num = 0;
        for (uint8_t i = 0; i < MASTER_TOTAL; i++)
        {
            m_mqtt_is_master_subscribed[i] = false;
        }
        m_mqtt_sub_req_tick++;
        if (m_mqtt_sub_req_tick > 10)
        {
            m_mqtt_sub_req_tick = 0;
            esp_mqtt_client_stop(m_mqtt_client);
        }
        else
        {
            // Send subscribe 'config_topic'
            app_mqtt_send_subscribe_config_topic(app_flash_get_imei());
        }
        break;

    case MQTT_EVENT_PUBLISHED:
        // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGD(TAG, "MQTT_EVENT_DATA");

        // get topic name
        char topicName[100] = {0};
        uint8_t topicLen = event->topic_len;
        if (topicLen > 99)
            topicLen = 99;
        memcpy(topicName, event->topic, topicLen);

        // clear last byte of received data
        event->data[event->data_len] = 0;

        ESP_LOGD(TAG, "TOPIC=%s, leng=%u", /*event->topic*/ topicName, event->topic_len);
        ESP_LOGD(TAG, "DATA=%s, leng=%u-%u", event->data, event->data_len, strlen(event->data));

        /* ==================================== From slave 'config_topic' ======================================= */
        if (strlen(app_flash_get_imei()) >= 15 && strstr(topicName, app_flash_get_imei()))
        {
            ESP_LOGD(TAG, "From 'config_topic'");
            // event->data[event->data_len] = 0;

            /* ================ Lệnh RESET thiết bị ===================*/
            if (strstr(event->data, "REBOOT"))
            {
                ESP_LOGI(TAG, "\t--- Reset System ---");
                mqtt_publish_message("DBG", "Reset by reboot cmd");
                app_audio_change_codec_vol(0);
                vTaskDelay(1000 / portTICK_RATE_MS);
                app_audio_simple_terminate_pipeline();
                vTaskDelay(1000 / portTICK_RATE_MS);

                system_software_reset(SW_RESET_REASON_REBOOT_CMD);
            }

            /* ================ Lệnh Cấu hình WiFi ===================*/
            if (strstr(event->data, "WIFINAME("))
            {
                ESP_LOGI(TAG, "\t--- Set WiFi info ---");

                // Write config to NVS
                mqtt_write_wifi_config(event->data);
            }

            /* ================ Lệnh OTA Firmware ===================*/
            char *ota_for_esp32 = strstr(event->data, "UDFW(");
            char *ota_for_gd32 = strstr(event->data, "UDFW_GD32(");
            char *ota_for_fm = strstr(event->data, "UDFW_FM(");
            char *force_ota = strstr(event->data, "FORCE_UDFW(");
            if (ota_for_esp32 || ota_for_gd32 || ota_for_fm)
            { /* UDFW(http link) */

                ESP_LOGI(TAG, "\t--- Update firmware ---");

                /* Không nhận lệnh liên tiếp, hoặc khi đang chạy task OTA rồi */
                if (app_ota_is_running())
                {
                    ESP_LOGI(TAG, "WAR: OTA is running...");
                    return ESP_OK;
                }

                /* Không OTA khi đang streaming */
                if (app_audio_is_http_audio_stream_running() && !force_ota)
                {
                    mqtt_publish_message("DBG", "It's STREAMING. Don't OTA! Try later");
                    return ESP_OK;
                }

                if (CopyParameter(event->data, m_ota_http_url, '(', ')'))
                {
                    ESP_LOGI(TAG, "OTA link: %s", m_ota_http_url);
                    static app_ota_info_t ota_url;
                    ota_url.url = m_ota_http_url;
                    ota_url.type = APP_OTA_DEVICE_INVALID;

                    if (!ota_for_esp32)
                    {
                        ESP_LOGW(TAG, "OTA for GD32");
                        if (ota_for_gd32 && app_io_get_gd32_protocol_method() == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
                        {
                            ota_url.type = APP_OTA_DEVICE_GD32;
                        }

                        if (ota_for_fm 
                            && app_io_get_gd32_protocol_method() == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL
                            && app_io_get_fm_protocol_method() == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
                        {
                            ota_url.type = APP_OTA_DEVICE_FM;
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "OTA for esp32");
                        ota_url.type = APP_OTA_DEVICE_ESP32;
                    }

                    if (ota_url.type != APP_OTA_DEVICE_INVALID)
                    {
                        // Create ota task
                        xTaskCreate(app_ota_download_task, "ota_task", 8192, (void *)&ota_url, 5, NULL);
                    }
                    else
                    {
                        mqtt_publish_message("DBG", "Invalid OTA device type");
                    }
                }
                break;
            }

            /* ================ Lệnh SET cấu hình ===================*/
            if (strstr(event->data, "SET,"))
            {
                ESP_LOGI(TAG, "\t--- Set Parameter ---");
                mqttProcessSetConfig(event->data);
                break;
            }

            /* ================ Lệnh GET cấu hình ===================*/
            if (strstr(event->data, "GET,"))
            {
                ESP_LOGI(TAG, "\t--- Get Parameter ---");
                process_get_config(event->data);
                break;
            }
            char *p = strstr(event->data, "STREAM_BY_URL(");
            if (p)
            {
                p += strlen("STREAM_BY_URL(");
                ESP_LOGI(TAG, "\tStream by URL");
                if (memcmp(p, "http://", 7) == 0 && strstr(p, ")"))
                {
                    process_stream_by_url(p);
                }
                else
                {
                    ESP_LOGI(TAG, "\tInvalid stream");
                }
                break;
            }
        }

        /* ====================================== Nếu là message từ topic 'master command' ===================================//
         * Ưu tiên xử lý theo thứ tự từng group: (T1 -> T2 -> T3) => (H1 -> H2 -> H3) => (X1 -> X2 -> X3)
         * 07/02/21: WARNING: chỗ này xác định command_master_level bị sai!!!
         * ====================================================================================================================//
         */
        uint8_t command_master_level = 0;
        for (uint8_t master_index = MASTER_TINH1; master_index < MASTER_TOTAL; master_index++)
        {
            ESP_LOGD(TAG, "TOPIC=%s,MASTER%d=%s", topicName, master_index, app_flash_get_master(master_index));
            if ((strlen(app_flash_get_master(master_index)) >= 15) && strstr(topicName, app_flash_get_master(master_index)))
            {
                command_master_level = master_index;
                break;
            }
        }

        ESP_LOGD(TAG, "CMD master level %u", command_master_level);
        // Message đến từ topic lạ (giả sử)!
        if (command_master_level == 0)
        {
            ESP_LOGE(TAG, "Topic name is not allow: %s", topicName);
            esp_mqtt_client_unsubscribe(m_mqtt_client, topicName);
            return ESP_OK;
        }

        // ESP_LOGI(TAG, "From Master topic level: %u, current level: %u", 
                    // command_master_level, 
                    // app_flash_get_current_streamming_master_index());

        /* Nếu đang OTA không nhận lệnh stream */
        if (app_ota_is_running())
            return ESP_OK;

        /* Nếu chế độ hoạt động không cho phép -> return */
        if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
        {
            ESP_LOGI(TAG, "Mode no operation");
            // Clear các trạng thái của pipeline để khi cho phép chế độ INTERNET trở lại thì STREAM được
            slave_set_http_finish_track_timeout(0);
            m_is_waiting_http_element_stopped = false;
            app_audio_reset_stream_retry();
            return ESP_OK;
        }
        // 20/12/20: Thêm lệnh "STREAM_PREPAIR" để điều khiển bật Relay theo thứ tự: relay1 -> delay xx giây -> relay2
        char *prepairMsg = strstr(event->data, "STREAM_PREPAIR");
        if (prepairMsg)
        {
            slave_process_prepair_onair(prepairMsg);
        }

        /**
         * ==================== Lệnh STREAM_STOP ===========================//
         * Ghi nhớ trạng thái STOP streaming của 3 đài.
         * Mục đích: Khi đài cấp cao hơn STOP stream, kiểm tra xem các đài cấp thấp hơn có đang phát không,
         * nếu đang phát thì chuyển luồng stream sang luôn
         * Chỉ STOP khi đang stream từ chính các Master Device! (Bỏ Schedule stream!)
         */
        if (strstr(event->data, "STREAM_STOP"))
        {
            switch (command_master_level)
            {
            case MASTER_TINH1:
                m_master_streamming_info.Name.MasterTINH1 = NO_STREAM;
                break;
            case MASTER_TINH2:
                m_master_streamming_info.Name.MasterTINH2 = NO_STREAM;
                break;
            case MASTER_TINH3:
                m_master_streamming_info.Name.MasterTINH3 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_HUYEN1:
                m_master_streamming_info.Name.MasterHUYEN1 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_HUYEN2:
                m_master_streamming_info.Name.MasterHUYEN2 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_HUYEN3:
                m_master_streamming_info.Name.MasterHUYEN3 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_XA1:
                m_master_streamming_info.Name.MasterXA1 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_XA2:
                m_master_streamming_info.Name.MasterXA2 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_XA3:
                m_master_streamming_info.Name.MasterXA3 = NO_STREAM;
                break;
            default:
                break;
            }
        }

        /** ==================== Lệnh APP_AUDIO_STREAM_START or APP_AUDIO_STREAM_RUNNING =========================== */
        if (strstr(event->data, "STREAM_START") 
            || strstr(event->data, "STREAM_RUNNING"))
        {
            slave_set_timeout_when_received_stream_running_command();
            ESP_LOGI(TAG, "STREAM_RUNNING");
            /* Nếu đang PHÁT -> Chỉ xử lý lệnh từ Master cấp bằng hoặc cao hơn
             * Ví dụ: Đang phát ở cấp xã nhận được lệnh cấp Huyện/Tỉnh -> Stop xã chuyển sang Huyện/Tỉnh
             * Nếu đang phát cấp tỉnh mà nhận được START cấp xã/huyện thì bỏ qua
             * Nếu đang DỪNG -> nhận lệnh từ cả 3 cấp!
             */

            /** Ghi nhớ trạng thái streaming của 3 đài.
             * Mục đích: Khi đài cấp cao hơn STOP stream, kiểm tra xem các đài cấp thấp hơn có đang phát không,
             * nếu đang phát thì chuyển luồng stream sang luôn
             */
            switch (command_master_level)
            {
            case MASTER_TINH1:
                m_master_streamming_info.Name.MasterTINH1 = LIVE_MASTER;
                break;
            case MASTER_TINH2:
                m_master_streamming_info.Name.MasterTINH2 = LIVE_MASTER;
                break;
            case MASTER_TINH3:
                m_master_streamming_info.Name.MasterTINH3 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_HUYEN1:
                m_master_streamming_info.Name.MasterHUYEN1 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_HUYEN2:
                m_master_streamming_info.Name.MasterHUYEN2 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_HUYEN3:
                m_master_streamming_info.Name.MasterHUYEN3 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_XA1:
                m_master_streamming_info.Name.MasterXA1 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_XA2:
                m_master_streamming_info.Name.MasterXA2 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_XA3:
                m_master_streamming_info.Name.MasterXA3 = LIVE_MASTER;
                break;
            default:
                ESP_LOGE(TAG, "Unknown master %u\r\n", command_master_level);
                break;
            }

            if (process_if_higher_stream(command_master_level, false))
            {
                return ESP_OK;
            }
            /* 2. ================== Nếu đang Không Streaming -> Nhận lệnh từ tất cả các master ============== */
            /* Nếu đang trong quá trình chuyển sang stream higher master -> không nhận lệnh nữa */
            if (slave_get_number_of_second_timeout_waiting_master_streamming())
            {
                ESP_LOGI(TAG, "Dang start higher master stream, khong nhan lenh: %u", slave_get_number_of_second_timeout_waiting_master_streamming());
                return ESP_OK;
            }

            if (slave_get_received_running_state_timeout() == 0)
            {
                ESP_LOGI(TAG, "Start counting running state timeout...");
                slave_set_received_running_state_timeout(35); /* Sau 35s mà không start stream được thì reset cho nhanh */
            }

            /* Đánh dấu dung lượng stream khi nhận lệnh -> so sánh sau 35 giây sau */
            if (strstr(event->data, "STREAM_START"))
            {
                slave_reset_stream_monitor_data();
            }
            // #warning "HuyTV comment out, not sure about it"
            // last_total_streaming_received = total_streaming_received;
            slave_update_last_stream_data();

            // DEBUG
            ESP_LOGI(TAG, "HTTP finish track timeout: %u, m_is_waiting_http_element_stopped: %u, Stream retries number: %u",
                     slave_get_http_finish_track_timeout(), m_is_waiting_http_element_stopped, app_audio_get_reset_stream_retry_number());

#if 0
			/** TEST: Nếu nhiều lệnh START/RUNNING đến liền nhau -> thay đổi url theo mức ưu tiên cao hơn */
			if(command_master_level <= app_flash_get_current_streamming_master_index()) {
				ESP_LOGI(TAG, "Chuyen url sang master uu tien cao hon: %u -> %u", app_flash_get_current_streamming_master_index(), command_master_level);
				
				/* 1. Save the current Master */
				app_flash_set_current_streamming_master(command_master_level);
				app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, command_master_level);

				/* 2. Init new app_audio_get_stream_url() */
				build_http_stream_url(command_master_level);
			}
			/* ============================ END TEST ==============================*/
#endif

            /** Hiện tượng: Đang khởi động stream với Higher Master, http_element vừa closed, đang delay 5s chuẩn bị START_STREAM
             * -> Các element vẫn chưa ở STATE_RUNNING -> nhận được lệnh APP_AUDIO_STREAM_RUNNING khác -> khởi tạo 1 STREAM mới -> xung đột
             * gây treo task -> watchdog reset
             * ==> Giải pháp: Nếu đang chờ chạy 1 luồng stream thì không khởi tạo luồng khác!
             */
            if (slave_get_http_finish_track_timeout() == 0 && /* Nếu nhận được sự kiện HTTP_FINISH_TRACK -> Đang chờ http_finished thì không START STREAM! */
                !m_is_waiting_http_element_stopped &&        /* Nếu đang chờ http_element stopped sau khi lệnh terminate -> Không start STREAM */
                app_audio_get_reset_stream_retry_number() == 0)        /* Đang retry start STREAM -> tạm thời chưa nhận lệnh STREAM */
            {
                
                if (!app_audio_is_i2s_running()
                    || !app_audio_is_opus_running()
                    || !app_audio_is_http_audio_stream_running())
                {
                    /* Dừng tất cả các chế độ khác -> chuyển sang chế độ codec decoder */
                    app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
                    app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                    // Change codec to DECODE mode
                    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                    // Set volume to volume setup
                    app_audio_change_codec_vol(app_flash_get_volume());

                    // Switch Relay to Codec output
                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                    // Nếu PA đang OFF thì ON
                    if (app_io_is_pa_off())
                    {
                        app_io_control_pa(APP_IO_PA_ON);
                    }

                    /* 2. Init new app_audio_get_stream_url() */
                    char *p_url = strstr(event->data, "STREAM_RUNNING(");
                    if (p_url && strstr(p_url, ")"))
                    {
                        memset(app_audio_get_stream_url(), 0, APP_AUDIO_HTTP_URL_SIZE);
                        CopyParameter(p_url, app_audio_get_stream_url(), '(', ')');
                        ESP_LOGI(TAG, "Stream by url %s", app_audio_get_stream_url());
                    }
                    else
                    {
                        build_http_stream_url(command_master_level);
                    }
                    ESP_LOGI(TAG, "HTTP stream URL %s\r\n", app_audio_get_stream_url());

                    // Nếu Relays đang OFF thì ON
                    // if(app_io_is_iso_relays_off()) {
                    char msg[96];
                    snprintf(msg, 96, "Start streaming by 'STREAM_START/STREAM_RUNNING' on master %s", 
                                    app_flash_get_master(command_master_level));
                    mqtt_publish_message("DBG", msg);
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                    //}

#if 1 // TEST: Chuyển khởi tạo url ra ngoài if
                    /** Lưu thông tin của Master ra lệnh START và khởi tạo URL tương ứng -> có thể đưa ra ngoài điều kiện check if
                     * Mục đích: Khi chưa khởi tạo stream xong mà có lệnh từ master cao hơn -> thay đổi url luôn, đến lúc stream sẽ lấy
                     * theo url cao nhất luôn -> phải check điều kiện isMasterHigherLevel
                     */
                    /* 1. Save the current Master */
                    app_flash_set_current_streamming_master(command_master_level);
                    app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, command_master_level);

#endif

                    /** Nếu http_stream_reader đang STOPPED hoặc INIT(sau khi khởi động request mà k có link) thì Start stream được luôn
                     * Trường hợp: Sau lệnh 'STREAM_STOP' -> terminate -> nhận lệnh 'STREAM_START' nhưng http_stream_reader
                     * vẫn chưa stopped xong (vẫn đang state RUNNING) -> terminate cho phát nữa và bật cờ 'm_is_waiting_http_element_stopped'
                     */
                    audio_element_state_t http_state = app_audio_get_http_state();
                    ESP_LOGI(TAG, "Http state %d\r\n", http_state);
                    if (http_state == AEL_STATE_STOPPED 
                        || http_state == AEL_STATE_INIT)
                    {
                        if (!slave_get_start_stream_command_timeout())
                        { /* Case: nhận được nhiều lệnh đến gần nhau -> tránh restart stream liên tục! */
                            slave_reset_total_streaming_received();
                            slave_set_auto_restart_stream_timeout(0);
                            slave_set_start_stream_command_timeout(15);
                            slave_set_timeout_turn_off_opto_output(0);

                            slave_set_timeout_number_of_waiting_master_streamming(30); /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */

                            /**01/05/20: delay before start streaming, nếu start stream luôn thì bị hiện tượng HTTP báo
                             * HTTP_STREAM_FINISH_TRACK -> Có thể do server đặt Latency Buffer lớn -> Stream ngay thì chưa có
                             * nội dung -> báo FINISH_TRACK. Để delay 5s thì không thấy bị
                             */
                            ESP_LOGI(TAG, "STREAM: Delay 5s before start streaming!");
                            vTaskDelay(5000 / portTICK_RATE_MS);
                            app_audio_restart_pipeline(app_audio_get_stream_url());
                            
                            app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_RUNNING);
                            char *dbg = malloc(strlen(app_audio_get_stream_url()) + 32);
                            if (dbg)
                            {
                                sprintf(dbg, "STREAM_START %s", app_audio_get_stream_url());
                                mqtt_publish_message("DBG", dbg);
                                free(dbg);
                            }
                            else
                            {
                                mqtt_publish_message("DBG", "STREAM_START");
                            }
                        }
                    }
                    else
                    {
                        ESP_LOGI(TAG, "'http_element' is not stopped, terminate_pipeline first and waiting for stoped...");
                        app_audio_simple_terminate_pipeline();
                        m_is_waiting_http_element_stopped = true;
                        slave_reset_counter_wait_for_http_element_stop();

                        /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */
                        slave_set_timeout_number_of_waiting_master_streamming(30); 
                        slave_allow_http_element_monitor_downloaded_data_in_streamming_state(false);  /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                    }
                }
            }
            else
            {
                ESP_LOGW(TAG, "Another stream is opening");
            }
        }
        /** ==================== Lệnh STREAM_STOP =========================== */
        else if (strstr(event->data, "STREAM_STOP") 
                && (slave_get_http_finish_track_timeout() == 0)) /** Đang không chờ timeout sau sự kiện HTTP_FINISH_TRACK */
        {
            ESP_LOGI(TAG, "MQTT: Stop streaming by 'STREAM_STOP' command...");

            /* ====================== Chỉ nhận lệnh STOP từ đúng master đang chạy ======================== */
            if (app_flash_get_current_streamming_master_index() == command_master_level)
            {
                slave_set_received_running_state_timeout(0);

                /* Nếu có element nào đang running -> terminate pipeline */
                if (app_audio_is_i2s_running()
                    || app_audio_is_opus_running()
                    || app_audio_is_http_audio_stream_running())
                {
                    ESP_LOGI(TAG, "[ * ] Stop streaming by owner master: %u", command_master_level);

                    /** 1. --------- Xét chuyển luồng stream sang đài cấp thấp hơn nếu đang streaming ---------------
                     * Nếu đài TỈNH dừng phát -> xét đài HUYỆN, XÃ...
                     */
                    uint8_t nextMasterLevel = 0;
                    switch (command_master_level)
                    {
                    case MASTER_TINH1:
                        if (m_master_streamming_info.Name.MasterTINH2)
                            nextMasterLevel = MASTER_TINH2;
                        else if (m_master_streamming_info.Name.MasterTINH3)
                            nextMasterLevel = MASTER_TINH3;
                        else if (m_master_streamming_info.Name.MasterHUYEN1)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN1;
                        else if (m_master_streamming_info.Name.MasterHUYEN2)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streamming_info.Name.MasterHUYEN3)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streamming_info.Name.MasterXA1)
                            nextMasterLevel = APP_FLASH_MASTER_XA1;
                        else if (m_master_streamming_info.Name.MasterXA2)
                            nextMasterLevel = APP_FLASH_MASTER_XA2;
                        else if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case MASTER_TINH2:
                        if (m_master_streamming_info.Name.MasterTINH3)
                            nextMasterLevel = MASTER_TINH3;
                        else if (m_master_streamming_info.Name.MasterHUYEN1)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN1;
                        else if (m_master_streamming_info.Name.MasterHUYEN2)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streamming_info.Name.MasterHUYEN3)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streamming_info.Name.MasterXA1)
                            nextMasterLevel = APP_FLASH_MASTER_XA1;
                        else if (m_master_streamming_info.Name.MasterXA2)
                            nextMasterLevel = APP_FLASH_MASTER_XA2;
                        else if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case MASTER_TINH3:
                        if (m_master_streamming_info.Name.MasterHUYEN1)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN1;
                        else if (m_master_streamming_info.Name.MasterHUYEN2)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streamming_info.Name.MasterHUYEN3)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streamming_info.Name.MasterXA1)
                            nextMasterLevel = APP_FLASH_MASTER_XA1;
                        else if (m_master_streamming_info.Name.MasterXA2)
                            nextMasterLevel = APP_FLASH_MASTER_XA2;
                        else if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_HUYEN1:
                        if (m_master_streamming_info.Name.MasterHUYEN2)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streamming_info.Name.MasterHUYEN3)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streamming_info.Name.MasterXA1)
                            nextMasterLevel = APP_FLASH_MASTER_XA1;
                        else if (m_master_streamming_info.Name.MasterXA2)
                            nextMasterLevel = APP_FLASH_MASTER_XA2;
                        else if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_HUYEN2:
                        if (m_master_streamming_info.Name.MasterHUYEN3)
                            nextMasterLevel = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streamming_info.Name.MasterXA1)
                            nextMasterLevel = APP_FLASH_MASTER_XA1;
                        else if (m_master_streamming_info.Name.MasterXA2)
                            nextMasterLevel = APP_FLASH_MASTER_XA2;
                        else if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_HUYEN3:
                        if (m_master_streamming_info.Name.MasterXA1)
                            nextMasterLevel = APP_FLASH_MASTER_XA1;
                        else if (m_master_streamming_info.Name.MasterXA2)
                            nextMasterLevel = APP_FLASH_MASTER_XA2;
                        else if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_XA1:
                        if (m_master_streamming_info.Name.MasterXA2)
                            nextMasterLevel = APP_FLASH_MASTER_XA2;
                        else if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_XA2:
                        if (m_master_streamming_info.Name.MasterXA3)
                            nextMasterLevel = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_XA3:
                        break;
                    default:
                        break;
                    }

                    /** Nếu tìm thấy master cấp thấp hơn đang stream -> chuyển về listen master đó */
                    if (nextMasterLevel)
                    {
                        ESP_LOGI(TAG, "[ * ] MASTER %u is streaming, let's listen to him", nextMasterLevel);

                        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                        // Set flag HttpElement, check when it's stopped later -> try to restart new stream...
                        m_is_waiting_http_element_stopped = true;
                        slave_reset_counter_wait_for_http_element_stop();
                        slave_reset_timeout_http_stream_monitor();
                        /* Don't turn off PA */
                        slave_set_timeout_turn_off_opto_output(0);

                        /* 1.1. Save the current Master */
                        app_flash_set_current_streamming_master(nextMasterLevel);
                        app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, nextMasterLevel);

                        /* 1.2. Init new app_audio_get_stream_url() */
                        build_http_stream_url(nextMasterLevel);
                        mqtt_publish_message("DBG", "Found next master, listen to him");

                        /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */
                        slave_set_timeout_number_of_waiting_master_streamming(30); 
                    }
                    else
                    {
                        ESP_LOGI(TAG, "[ * ] Have NO any master are streaming, let's relaxing!");

                        /* 1.3. Turn off PA after 30s */
                        slave_set_timeout_turn_off_opto_output(15);
                        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_IDLE);
                        slave_allow_http_element_monitor_downloaded_data_in_streamming_state(false);
                        slave_reset_timeout_http_stream_monitor();
                        app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, 0);
                        // Test: chuyển currentMaster về master nhỏ nhất
                        // app_flash_set_current_streamming_master(APP_FLASH_MASTER_XA3);
                    }
                    /** --------- END of chuyển luồng stream sang đài cấp thấp hơn nếu đang streaming ---------------*/

                    /** 2. Terminate pipeline */
                    ESP_LOGW(TAG, "Stop stream from master cmd -> terminate pipeline");
                    app_audio_pause();
                    app_audio_complete_terminate();
                    app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_STOPPED);
                    slave_allow_http_element_monitor_downloaded_data_in_streamming_state(false); /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                    app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                    ESP_LOGW(TAG, "Monitor stream =>> disable\r\n");
                }
                else
                {
                    ESP_LOGI(TAG, "[ * ] There isn't any audio_element's running. Ignore terminate!");
                }
            }
            else
            {
                ESP_LOGW(TAG, "[OH NO] You're not my master, DON'T STOP!!!");
            }
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;

    default:
        ESP_LOGI(TAG, "MQTT other event id: %d", event->event_id);
        break;
    }
    return ESP_OK;
}

void app_mqtt_send_subscribe_config_topic(char *imei)
{
    /* Nếu đã có GSM_IMEI -> Sub luôn topic Slave config */
    if (strlen(imei) >= 15)
    {
        memset(m_slave_sub_topic, 0, sizeof(m_slave_sub_topic));
        sprintf(m_slave_sub_topic, "%s%s", SLAVE_SUB_TOPIC_CONF_HEADER, imei);
        m_mqtt_sub_cfg_msg_id = esp_mqtt_client_subscribe(m_mqtt_client, m_slave_sub_topic, 1);

        // ESP_LOGI(TAG, "sent subscribe %s OK, msg_id=%d", m_slave_sub_topic, m_mqtt_sub_cfg_msg_id);
        m_mqtt_sub_req_tick = 1;
    }
    else
    {
        m_mqtt_sub_cfg_msg_id = 0;
        ESP_LOGE(TAG, "GSM_IMEI is empty!\r\n");
    }
}

uint8_t app_mqtt_send_subscribe_command_topic(uint8_t masterID)
{
    if (masterID < MASTER_TINH1 || masterID > APP_FLASH_MASTER_XA3)
        return 0;

    /* Đã có cấu hình MASTER -> Sub topic 'master_command' */
    if (strlen(app_flash_get_master(masterID)) >= 15)
    {
        memset(m_slave_sub_topic, 0, sizeof(m_slave_sub_topic));
        sprintf(m_slave_sub_topic, "%s%s", SLAVE_SUB_TOPIC_CMD_HEADER, app_flash_get_master(masterID));
        m_mqtt_sub_master_msg_id[masterID] = esp_mqtt_client_subscribe(m_mqtt_client, m_slave_sub_topic, 1);

        // ESP_LOGI(TAG, "sent subscribe 'master%d_cmd' OK, msg_id=%d", masterID, m_mqtt_sub_master_msg_id[masterID));
        m_mqtt_sub_req_tick = 1;
        return 1;
    }
    return 0;
}

void app_mqtt_publish_slave_info(void)
{
    if (m_mqtt_client == NULL)
        return;
    if (app_mqtt_get_state() != APP_MQTT_CONNECTED)
        return;

    char *pub_topic = malloc(64);
    char *body = malloc(256);
    if (!pub_topic || !body)
    {
        if (pub_topic)
            free(pub_topic);
        if (body)
            free(body);
        ESP_LOGE(TAG, "Can't allocate mem!");
        return;
    }
    slave_set_mqtt_state_timeout(0);

    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(pub_topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, app_flash_get_imei());
    }
    else
    {
        sprintf(pub_topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, "NA");
    }

    /* Nội dung info: INF,<IMEI>,<FW version>,<Stream State>,<Network Interface>,<Stream time in second>,<Stream data usage in byte>,
     *		<Running Mode>,<FM Freq>,<Volume>,<MIC state>,<IO1 state>,<IO2 state>,<FM SNR>,<RSSI>,<RSSI in dBm>,<GPS lat>,<GPS lng>,
     *		<GSM CSQ>,<GSM network name>,<Tech>,<Band>,<Speaker state>,<MCU FwVersion>,PA level
     */
    const char *state_str = app_audio_get_http_state_description();
    if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
    {
        state_str = "STOP";
    }

    int bodylen = sprintf(body, "INF,%s,%s,%s,%s,%u,%u,", (strlen(app_flash_get_imei()) >= 15) ? app_flash_get_imei() : "NA",
                          __FIRMWARE_VERSION__, state_str,
                          NET_IF_TAB[network_get_current_interface()], slave_get_stream_time(), 
                          slave_get_total_streaming_received());

    // Chế độ đang chạy
    if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET)
    {
        char *master_str = "000";
        // uint8_t isFoundCurMaster = 0;
        for (uint8_t i = MASTER_TINH1; i < MASTER_TOTAL; i++)
        {
            if (strlen(master_str) < 5 && strlen(app_flash_get_master(i)) > 5)
            {
                master_str = app_flash_get_master(i);
            }
            if (app_flash_get_current_streamming_master_index() == i 
                && strlen(app_flash_get_master(i)) > 5)
            {
                // bodylen += sprintf(&body[bodylen], "%s,", app_flash_get_master(i));
                // isFoundCurMaster = 1;
                master_str = app_flash_get_master(i);
                break;
            }
        }
        // // Nếu chưa có curMaster -> hiển thị "unknow"
        // if (!isFoundCurMaster)
        // {
        //     bodylen += sprintf(&body[bodylen], "%s,", "000");
        // }
        bodylen += sprintf(&body[bodylen], "%s,", master_str);
    }
    else if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_FM)
    {
        bodylen += sprintf(&body[bodylen], "%s,", "FM");
    }
    else if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_MIC)
    {
        bodylen += sprintf(&body[bodylen], "%s,", "MIC");
    }
    else if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
    {
        bodylen += sprintf(&body[bodylen], "%s,", "NONE");
    }

    // FM Freq - Tần số FM mà module FM đang thu
    if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_FM 
        || app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_FM)
    {
        bodylen += sprintf(&body[bodylen], "%.1f,", (float)app_io_get_freq_at_slave() / 1000);
    }
    else
    {
        bodylen += sprintf(&body[bodylen], "%u,", 0);
    }

    if (app_io_get_hardware_version() == 2)
    {
        // Volume hiện tại, MIC state, IO1, IO2
        bodylen += sprintf(&body[bodylen], "%u,%s,%s,%s,",
                           app_audio_get_current_output_vol(),
                           app_io_is_mic_plugged() ? "MIC" : "NA",
                           (app_io_get_i2c_exp_value()->BitName.ISO_OUT1 == APP_IO_OPTO_ON) ? "ON" : "OFF",
                           (app_io_get_i2c_exp_value()->BitName.ISO_OUT2 == APP_IO_OPTO_ON) ? "ON" : "OFF");
    }
    else if (app_io_get_hardware_version() == 3)
    {
        // Volume hiện tại, MIC state, IO1, IO2
        bodylen += sprintf(&body[bodylen], "%u,%s,%s,%s,",
                           app_audio_get_current_output_vol(),
                           app_io_is_mic_plugged() ? "MIC" : "NA",
                           (app_io_get_mcu_exp_value()->BitName.ISO_OUT1 == APP_IO_STM_OPTO_ON) ? "ON" : "OFF",
                           (app_io_get_mcu_exp_value()->BitName.ISO_OUT2 == APP_IO_STM_OPTO_ON) ? "ON" : "OFF");
    }

    char *gps_info = app_io_get_last_gps_info();
    if (strlen(gps_info) < 3)
    {
        gps_info = "0.0000,0.0000";
    }  
    uint32_t snr, rssi, dbm;
    app_io_get_fm_snr_info(&snr, &rssi, &dbm);
    // FM SNR, RSSI, RSSI in dBm, GPS lat, GPS lng
    bodylen += sprintf(&body[bodylen], "%d,%d,%d,%s,",
                       snr, rssi, dbm, gps_info);

    // GSM CSQ
    bodylen += sprintf(&body[bodylen], "CSQ:%d,", slave_get_gsm_csq());

    // OperatorNetwork, Technology access, band, channel
    modem_dce_t *dce = slave_get_modem_dce();
    if (dce)
    {
        bodylen += sprintf(&body[bodylen], "%s,%s,%s,%s,",
                           dce->oper, dce->accessTech, dce->networkBand, dce->networkChannel);
    }
    else
    {
        bodylen += sprintf(&body[bodylen], "%s,%s,%s,%s,", "NA", "NA", "NA", "NA");
    }
#if 1
    // Speaker state
    if (app_flash_speaker_detect_is_enable())
    {
        bodylen += sprintf(&body[bodylen], "%d,", app_io_get_speaker_detect_value().Value);
    }
    else
    {
        bodylen += sprintf(&body[bodylen], "%d,", 7);       // 7 hoac 15 the hien trang thai khong xac dinh
    }
#else
    bodylen += sprintf(&body[bodylen], "%d,", 7);       // 7 hoac 15 the hien trang thai khong xac dinh
#endif
    // MCU firmware version
    bodylen += sprintf(&body[bodylen], "%d,", app_io_get_worker_version());
    bodylen += sprintf(&body[bodylen], "%d", app_io_get_pa_state());
    // Publish mqtt
    int msg_id = esp_mqtt_client_publish(m_mqtt_client, pub_topic, body, bodylen, 0, 0);
    ESP_LOGI(TAG, "slave publish 'info' OK, msg_id=%d", msg_id);

    free(pub_topic);
    free(body);
}
