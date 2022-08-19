/* Play M3U HTTP Living stream

   ============
   Sửa menuconfig
   ESP32 Specific:
       - Tăng các Main task stack size 3584 lên 100KB (102400)

//	   - Nếu chạy song song cả WiFi/PPP -> PPP bị lỗi: assertion "conn->write_offset < conn->current_msg->msg.w.len" failed, api_msg.c", line 1674, function: lwip_netconn_do_writemore
//	   - Thử tăng Main task stack size lên 112KB để chạy song song PPPoS vs Wifi -> vẫn lỗi -> tăng lên 116KB -> Khởi động bị reset liên tục!
//	   - Thử giảm Main task stack size còn 65536 -> chạy được song song, nhưng lỗi ngẫu nhiên -> thử tăng TCP send buffer size?
//   LwIP : Tăng TCP/IP stack size 4096 -> 8192 -> chạy riêng PPP ổn!
//   LwIP -> TCP : tăng send buffer size 5744 -> tăng lên gấp đôi 11488 chạy vài giờ k thấy bị!
//   Lý do là nhiều khi các gói tin gửi đi liên tiếp, kích thước lớn -> over size of buffer
//   Ví dụ:
//   pppos_netif_output[1]: proto=0x21, len = 1064
//   pppos_netif_output[1]: proto=0x21, len = 1440
//   pppos_netif_output[1]: proto=0x21, len = 1440
//   or
//   pppos_netif_output[1]: proto=0x21, len = 1064
//   pppos_netif_output[1]: proto=0x21, len = 1390
//   pppos_netif_output[1]: proto=0x21, len = 1390
//   pppos_netif_output[1]: proto=0x21, len = 1390
//   assertion "last_unsent->oversize_left >= oversize_used" failed: file "../esp/esp-adf/esp-idf/components/lwip/lwip/src/core/tcp_out.c", line 686, function: tcp_write
//   abort() was called at PC 0x400da247 on core 0

- Bật/tắt debug PPP: Component config -> LWIP -> Enable PPP support -> Enable PPP debug log output
*/

/******************************************************************************
                                   INCLUDES
******************************************************************************/
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "http_stream.h"
#include "i2s_stream.h"
#include "aac_decoder.h"
// #include "esp32/rom/rtc.h"
#include "rom/rtc.h"
#include "wav_decoder.h"
#include "mp3_decoder.h"
#include "opus_decoder.h"

#include "esp_peripherals.h"
#include "periph_wifi.h"
#include "board.h"

#include "mqtt_client.h"
#include "esp_modem.h"
#include "ec2x.h"

// For Ethernet //
#include "lwip/netif.h"
// #include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "rom/gpio.h"
// #include "tcpip_adapter.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "mdns.h"
#include "esp_ping.h"
#include "ping/ping.h"
#include "netdb.h"

// SNTP
#include "esp_sntp.h"

// For OTA
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

// User define
#include "pcf8575.h"
#include "DataDefine.h"
#include "utilities.h"
#include "audio_hal.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "http_stream.h"
#include "network.h"
#include "app_ota.h"
#include "app_io.h"
#include "app_flash.h"
#include "app_mqtt.h"
#include "app_sntp.h"
#include "esp32/rom/rtc.h"
#include "tcp_console.h"

#define SPEAKER_DETECT_ENABLE   1

/******************************************************************************
                                   GLOBAL VARIABLES
******************************************************************************/

/******************************************************************************
                                   CONSTANT VALUE
******************************************************************************/
static const char *TAG = "LIVEDOWN";

// ADF LOG customize
/**
 * @brief Log level
 *
 */
// typedef enum {
//	ESP_LOG_NONE,		/*!< No log output */
//	ESP_LOG_ERROR,		/*!< Critical errors, software module can not recover on its own */
//	ESP_LOG_WARN,		/*!< Error conditions from which recovery measures have been taken */
//	ESP_LOG_INFO,		/*!< Information messages which describe normal flow of events */
//	ESP_LOG_DEBUG,		/*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
//	ESP_LOG_VERBOSE 	/*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
// } esp_log_level_t;

#ifndef CONFIG_ADF_LOG_LEVEL
#define CONFIG_ADF_LOG_LEVEL ESP_LOG_INFO
#endif /**< CONFIG_ADF_LOG_LEVEL */

static uint8_t ADF_LOG_LEVEL = CONFIG_ADF_LOG_LEVEL;

#define ADF_LOG_FORMAT(letter, format) LOG_COLOR_##letter #letter " (%d) [%s, %d]: " format LOG_RESET_COLOR "\n"

#define ADF_LOGE(format, ...)                                                                                                \
    do                                                                                                                       \
    {                                                                                                                        \
        if (ADF_LOG_LEVEL >= ESP_LOG_ERROR)                                                                                  \
        {                                                                                                                    \
            esp_log_write(ESP_LOG_ERROR, TAG, ADF_LOG_FORMAT(E, format), esp_log_timestamp(), TAG, __LINE__, ##__VA_ARGS__); \
        }                                                                                                                    \
    } while (0)

#define ADF_LOGW(format, ...)                                                                                               \
    do                                                                                                                      \
    {                                                                                                                       \
        if (ADF_LOG_LEVEL >= ESP_LOG_WARN)                                                                                  \
        {                                                                                                                   \
            esp_log_write(ESP_LOG_WARN, TAG, ADF_LOG_FORMAT(W, format), esp_log_timestamp(), TAG, __LINE__, ##__VA_ARGS__); \
        }                                                                                                                   \
    } while (0)

#define ADF_LOGI(format, ...)                                                                                               \
    do                                                                                                                      \
    {                                                                                                                       \
        if (ADF_LOG_LEVEL >= ESP_LOG_INFO)                                                                                  \
        {                                                                                                                   \
            esp_log_write(ESP_LOG_INFO, TAG, ADF_LOG_FORMAT(I, format), esp_log_timestamp(), TAG, __LINE__, ##__VA_ARGS__); \
        }                                                                                                                   \
    } while (0)

#define ADF_LOGD(format, ...)                                                                                                \
    do                                                                                                                       \
    {                                                                                                                        \
        if (ADF_LOG_LEVEL >= ESP_LOG_DEBUG)                                                                                  \
        {                                                                                                                    \
            esp_log_write(ESP_LOG_DEBUG, TAG, ADF_LOG_FORMAT(D, format), esp_log_timestamp(), TAG, __LINE__, ##__VA_ARGS__); \
        }                                                                                                                    \
    } while (0)

#define ADF_LOGV(format, ...)                                                                                                  \
    do                                                                                                                         \
    {                                                                                                                          \
        if (ADF_LOG_LEVEL >= ESP_LOG_VERBOSE)                                                                                  \
        {                                                                                                                      \
            esp_log_write(ESP_LOG_VERBOSE, TAG, ADF_LOG_FORMAT(V, format), esp_log_timestamp(), TAG, __LINE__, ##__VA_ARGS__); \
        }                                                                                                                      \
    } while (0)

// Log cho ZIG
#define ADF_LOGZ(format, ...)                                       \
    do                                                              \
    {                                                               \
        esp_log_write(ESP_LOG_VERBOSE, "ZIG", format, ##__VA_ARGS__); \
    } while (0)

/******************************************************************************
                                   GLOBAL FUNCTIONS
******************************************************************************/

/******************************************************************************
                                   DATA TYPE DEFINE
******************************************************************************/
/**
 * LED chỉ chị loa phường:
- LED1: nháy xanh: chưa online; sáng xanh: online SIM
- LED2: nháy đỏ: chưa online; sáng đỏ: online ETH; sáng xanh: đang phát hoặc thu
* Version history
https://bytechvn-my.sharepoint.com/:w:/g/personal/biennb_bytech_vn/EdhCqa98dRxBoXqDesn9TLoBmjnM4wAN6pg9UFn-WXRI9A?e=m4J0X6
*/



// Power pin for Ethernet PHY chip (define if used gpio pin)
#ifndef CONFIG_PHY_USE_POWER_PIN
#define CONFIG_PHY_USE_POWER_PIN
#endif

#define TEST_VBAT_GSM 0
#define AAC_STREAM 0
#define WAV_STREAM 0
#define MP3_STREAM 0
#define OPUS_STREAM 1

#define RECONNECT_WIFI_IN_STREAM 30000
#define RECONNECT_WIFI_IN_IDLE 3000


#define MULTI_ELEMENT   0

// Network error timeout
#define NETWORK_ERROR_TIMEOUT_SEC 180 // 180
#define RTC_MEMORY_VALID_INFO_KEY		0x55AABB

// url stream header
static uint8_t timeoutHttpStreamCheck = 0;


typedef struct
{
    uint32_t flag0;
    uint8_t reason;
    uint32_t flag1;
} software_reset_reason_on_rtc_t;

/******************************************************************************
                                   PRIVATE VARIABLES
******************************************************************************/
/* Các trạng thái của audio_element
 *    AEL_STATE_NONE = 0,
 *    AEL_STATE_INIT,
 *    AEL_STATE_RUNNING,
 *    AEL_STATE_PAUSED,
 *    AEL_STATE_STOPPED,
 *    AEL_STATE_FINISHED,
 *    AEL_STATE_ERROR
 */
const char AEL_STATE_TAB[8][16] = 
{
    "NONE",
    "INIT",
    "INITIALIZING",
    "RUNNING",
    "PAUSED",
    "STOP",
    "FINISH",
    "ERROR"
};

// System_t xSystem;
uint32_t systemTickCount = 0;
uint32_t lastSystemTickCount = 0;
static uint8_t m_main_task_hangout = 0;
// uint8_t HW_VERSION = 3;
static uint32_t m_terminate_pipeline_in_stream_running = 0;
bool isStoppedByClient = false;
static uint32_t total_streaming_received = 0;
static uint32_t total_stream_time_in_session = 0;

static uint8_t m_auto_restart_stream_timeout = 0;
static uint8_t m_timeout_turn_off_opto_output = 0;
static uint8_t m_start_stream_cmd_timeout = 0;

static bool m_allow_monitor_stream_downloaded = false;

static uint8_t m_waiting_http_element_stopped_timeout = 0;
static uint8_t waitingHigherMasterStartStreaming = 0;
uint8_t m_timeout_wating_next_master_streamming = 0;

// Case: Bên phát bị rớt mạng -> mất data -> FINISH_TRACK event
//-> http_element chưa kịp nhận FINISHED_STATUS thì bên phát lại lệnh APP_AUDIO_STREAM_START lại
//-> trạng thái http_element vẫn đang RUNNING -> terminate_pipeline và chờ http_stopped
//-> tuy nhiên khi đã xảy ra FINISH_TRACK thì terminate_pipeline sẽ không làm cho http_element
//  về stopped -> chờ mãi cũng không stopped để restart stream được -> Không thu được gì!
//=> Khi nhận được FINISH_TRACK thì không nhận APP_AUDIO_STREAM_START hay STREAM_STOP nữa, chờ xảy ra
//  FINISHED_STATUS và restart lại stream
static uint8_t httpGetFinishTrackEventTimeout = 0;

// Kiểm soát trạng thái streaming, nếu giữa 2 bản tin APP_AUDIO_STREAM_RUNNING của master mà
// dung lượng stream bên thu không thay đổi -> không stream được -> reset system luôn
uint32_t last_total_streaming_received = 0;
uint8_t receiveRunningStateTimeout = 0;
static int8_t mqttSendStreamStateTimeout = 0;
static uint8_t mqtt_fast_subscribe_timeout = 0;
uint8_t mqttSendSlowSubscribeTimeout = 0;
uint8_t mqttSendInfoTimeout = 0;
uint16_t mqttDisconnectedTimeout = 0;


uint8_t delayTurnOnRelayPrepairStream = 0; // On relay1 -> delay -> on relay2
// delay2 -> off relay2 -> delay1 -> off relay1
uint8_t delayTurnOffRelay1StopStream = 0;
uint16_t delayTurnOffRelay2StopStream = 0;
static uint8_t m_http_received_data_timeout;

// Chọn về Internet từ web thì chế độ Local về Idle
uint32_t auto_switch_codec_mode_in_test_mode = 0;

void SoftResetSystem(uint8_t reason);


static modem_dce_t *m_dce = NULL;
RTC_NOINIT_ATTR software_reset_reason_on_rtc_t m_reset_reason;

void SoftResetSystem(uint8_t reason)
{
    app_flash_write_u8(APP_FLASH_SELF_RESET_KEY, reason);
    vTaskDelay(500 / portTICK_RATE_MS);
    esp_restart();
}

void system_software_reset(uint8_t reason)
{
    m_reset_reason.flag0 = RTC_MEMORY_VALID_INFO_KEY;
    m_reset_reason.reason = reason;
    m_reset_reason.flag1 = RTC_MEMORY_VALID_INFO_KEY;
    vTaskDelay(50 / portTICK_RATE_MS);
    esp_restart();
}

uint8_t system_get_software_reset_reason(void)
{
    if (m_reset_reason.flag0 == RTC_MEMORY_VALID_INFO_KEY
        && m_reset_reason.flag1 == RTC_MEMORY_VALID_INFO_KEY)
    {
        return m_reset_reason.reason;
    }
    return SW_RESET_REASON_UNKNOWN;
}

void slave_reset_delay_turn_off_relay1_when_stop_stream(void)
{
    delayTurnOffRelay1StopStream = 0;
}

void slave_reset_delay_turn_off_relay2_when_stop_stream(void)
{
    delayTurnOffRelay2StopStream = 0;
}

void slave_reset_timeout_http_stream_monitor(void)
{
    timeoutHttpStreamCheck = 0;
}

void slave_allow_http_element_monitor_downloaded_data_in_streamming_state(bool state)
{
    m_allow_monitor_stream_downloaded = state;
}

void slave_reset_counter_wait_for_http_element_stop(void)
{
    m_waiting_http_element_stopped_timeout = 0;
}

void slave_set_mqtt_state_timeout(uint32_t timeout)
{
    mqttSendStreamStateTimeout = timeout;
}

void slave_set_auto_restart_stream_timeout(uint8_t timeout)
{
    m_auto_restart_stream_timeout = timeout;
}

void slave_set_timeout_when_received_stream_running_command(void)
{
    m_terminate_pipeline_in_stream_running = 70;
}

void slave_set_audio_switch_audio_in_test_mode(void)
{
    auto_switch_codec_mode_in_test_mode = 15;
}

uint8_t slave_get_http_finish_track_timeout(void)
{
    return httpGetFinishTrackEventTimeout;
}

void slave_set_http_finish_track_timeout(uint8_t timeout)
{
    httpGetFinishTrackEventTimeout = timeout;
}

void slave_set_received_running_state_timeout(uint32_t timeout)
{
    receiveRunningStateTimeout = timeout;
}

uint8_t slave_get_received_running_state_timeout(void)
{
    return receiveRunningStateTimeout;
}

uint32_t slave_get_stream_time(void)
{
    return total_stream_time_in_session;
}

void slave_increase_stream_data_downloaded(uint32_t number_of_bytes)
{
    total_streaming_received += number_of_bytes;
    m_http_received_data_timeout = 60;    
}

uint8_t slave_get_http_received_data_timeout(void)
{
    return m_http_received_data_timeout;
}

void slave_reset_stream_monitor_data(void)
{
    last_total_streaming_received = 0;
    total_streaming_received = 0;
}

uint32_t slave_get_total_streaming_received(void)
{
    return total_streaming_received;
}

void slave_reset_total_streaming_received(void)
{
    total_streaming_received = 0;
}

void slave_update_last_stream_data(void)
{
    last_total_streaming_received = total_streaming_received;
}

void slave_set_timeout_number_of_waiting_master_streamming(uint8_t counter)
{
    m_timeout_wating_next_master_streamming = counter;
}

uint8_t slave_get_number_of_second_timeout_waiting_master_streamming(void)
{
    return m_timeout_wating_next_master_streamming;
}

void slave_process_stop_onair(void)
{
    // Thêm điều khiển OFF relay theo thứ tự delay2 -> off relay2 -> delay1 -> off relay1
    delayTurnOffRelay2StopStream = app_flash_get_relay2_turn_off_delay();
}

void slave_set_start_stream_command_timeout(uint8_t timeout)
{
    m_start_stream_cmd_timeout = timeout;
}

uint8_t slave_get_start_stream_command_timeout(void)
{
    return m_start_stream_cmd_timeout;
}


uint8_t slave_get_timeout_turn_off_opto_output(void)
{
    return m_timeout_turn_off_opto_output;
}

void slave_set_timeout_turn_off_opto_output(uint8_t timeout)
{
    m_timeout_turn_off_opto_output = timeout;
    
}

static uint8_t m_csq = 99;
uint8_t slave_get_gsm_csq(void)
{
    return m_csq;
}

void *slave_get_modem_dce(void)
{
    return m_dce;
}

static char m_ppp_ip_str[24] = {"0.0.0.0"};

char *slave_get_ppp_ip(void)
{
    return m_ppp_ip_str;
}

void network_event_cb(void *src, void *evt, void *data, uint32_t size)
{
    network_interface_src_t *if_src = (network_interface_src_t*)src;
    if (*if_src == NETWORK_INTERFACE_SRC_ETH)
    {
        uint32_t eth_evt = (uint32_t)evt;
        if (eth_evt == SYSTEM_EVENT_ETH_DISCONNECTED)
        {
            /* Test LED Ethernet ON */
            app_io_control_led_eth_state(0);
        }
        else if (eth_evt == SYSTEM_EVENT_ETH_GOT_IP)
        {
            ADF_LOGZ("[ZIG] ETHERNET: Link Up - Got IP\r\n");
            app_io_control_led_eth_state(1);
        }
    }
    else if (*if_src == NETWORK_INTERFACE_SRC_WIFI)
    {
        uint32_t wifi_evt = (uint32_t)evt;
        if (wifi_evt == SYSTEM_EVENT_STA_GOT_IP)
        {
            app_io_control_led_wifi_state(1);
        }
        else if (wifi_evt == SYSTEM_EVENT_STA_DISCONNECTED)
        {
            app_io_control_led_wifi_state(0);
        }
    }
    else if (*if_src == NETWORK_INTERFACE_SRC_SMART_CFG)
    {
        app_flash_write_wifi_info(((network_wifi_info_t*)data)->ssid, ((network_wifi_info_t*)data)->password, 1);
    }
    else if (*if_src == NETWORK_INTERFACE_SRC_PPP)
    {
        esp_modem_event_t modem_event = (esp_modem_event_t)evt;
        switch (modem_event)
        {
            case MODEM_EVENT_PPP_START:
            break;

        case MODEM_EVENT_PPP_CONNECT:
        {
            ESP_LOGI(TAG,  "Modem connected to PPP Server");
            ppp_client_ip_info_t *ipinfo = (ppp_client_ip_info_t *)(data);
            sprintf(m_ppp_ip_str, IPSTR, IP2STR(&ipinfo->ip));
            app_io_control_led_4g_state(1);
        }
            break;

        case MODEM_EVENT_PPP_DISCONNECT:
            ESP_LOGI(TAG,  "Modem Disconnect from PPP Server");
            app_io_control_led_4g_state(0);
            break;

        case MODEM_EVENT_PPP_STOP:
            ESP_LOGI(TAG,  "Modem PPP Stopped");
            app_io_control_led_4g_state(0);
            m_csq = 99;
            break;

        case MODEM_EVENT_INIT_DONE: /* Khởi tạo xong module 4G */
        {
            modem_dce_t *dce = (modem_dce_t*)data;
            m_dce = dce;
            /* Print Module ID, Operator, IMEI, IMSI */
            ESP_LOGI(TAG,  "Module version: %s, IMEI: %s", dce->name, dce->imei);
            ESP_LOGI(TAG,  "SIM IMEI: %s, IMSI: %s", dce->sim_imei, dce->imsi);
            ESP_LOGI(TAG,  "Operator: %s", dce->oper);
            ESP_LOGI(TAG,  "RSSI: %d", dce->rssi);
            ESP_LOGI(TAG,  "Tech: %s, band: %s", dce->accessTech, dce->networkBand);
            m_csq = dce->rssi;
            ADF_LOGZ("[ZIG] IMEI: GSM: %s, SIM: %s\r\n", dce->imei, dce->imsi);

            // Lưu thông tin IMEI module vào bộ nhớ, dùng cho lần sau
            if (IsDigitString(dce->imei))
            {
                if (strlen(app_flash_get_imei()) < 15 || (strcmp(dce->imei, app_flash_get_imei()) != 0))
                {
                    sprintf(app_flash_get_imei(), "%s", dce->imei);
                    app_flash_node_nvs_write_string(APP_FLASH_GSM_IMEI_KEY, app_flash_get_imei());
                    ESP_LOGI(TAG,  "Coppied GSM_IMEI: %s", app_flash_get_imei());
                }
                else
                {
                    ESP_LOGI(TAG,  "GSM_IMEI is existed and matched!");
                }
            }
        }
            break;

        case MODEM_EVENT_UNKNOWN:
            break;
        default:
            break;
        }
    }
}


/******************************************************************************************/
/**
 * @brief   : system Timer callback, 10ms
 * @param   :
 * @retval  :
 * @author  :
 * @created :
 */
static void xSystem_timercb(void *timer)
{
    static uint8_t timeout1000ms = 0;

    timeout1000ms++;
    if (timeout1000ms >= 100)
    {
        timeout1000ms = 0;

        /* Check system tick count */
        if (systemTickCount != lastSystemTickCount)
        {
            lastSystemTickCount = systemTickCount;
            m_main_task_hangout = 0;
        }
        else
        {
            if (m_main_task_hangout++ >= 20)
            {
                ets_printf("!!! OOOOPS... MAIN TASK HANGUP. REBOOT !!!");
                system_software_reset(SW_RESET_REASON_MAIN_TASK_HANG);
            }
        }
    }
}

void slave_set_delay_turn_on_relay_prepare_stream(uint8_t value)
{
    delayTurnOnRelayPrepairStream = value;
}

uint8_t slave_get_delay_turn_on_relay_prepare_stream(uint8_t value)
{
    return delayTurnOnRelayPrepairStream;
}

void slave_reset_delay_turn_on_relay_process_on_air(void)
{
    delayTurnOnRelayPrepairStream = 0;
}

/******************************************************************************************/
/**
 * @brief   : task quản lý hoạt động của module gsm
 * @param   :
 * @retval  :
 * @author  :
 * @created :
 */
static void main_manager_task(void *arg)
{
    uint8_t mqttTick = 0;
    uint8_t task_timeout10s = 0;
    //	uint8_t task_tick3s = 0;

    uint8_t lastStreamState = 0;
    // uint8_t lastMICDetectState = 1;
    uint8_t ledState = 0;
    //	uint8_t testOnOffGSMVbat = 0;
    uint8_t masterSubIndex = MASTER_TINH1;
    esp_err_t err;

    ESP_LOGI(TAG,  "\t\t--- main_manager_task is running ---");

    while (1)
    {
        systemTickCount++;

        network_manager_poll();

        /* ==================================== Quản lý MQTT connection ==========================================*/
        if (network_is_connected())
        {
            switch (app_mqtt_get_state())
            {
            case APP_MQTT_DISCONNECTED:
                {
                    err = app_mqtt_start();
                    if (err == ESP_OK)
                    {
                        mqttTick = 0;
                    }
                    else
                    {
                        mqttTick = 8;
                    }
                    app_mqtt_set_state(APP_MQTT_CONNECTING);
                    mqtt_fast_subscribe_timeout = 0;
                }
                break;
            case APP_MQTT_CONNECTING:
                if (mqttTick++ >= 10)
                {
                    err = app_mqtt_start();
                    if (err == ESP_OK)
                    {
                        mqttTick = 0;
                    }
                    else
                    {
                        mqttTick = 8;
                    }
                }
                break;
            case APP_MQTT_CONNECTED:
                mqttDisconnectedTimeout = 0;

                /* Gửi bản tin infor định kỳ mỗi 60s */
                if (++mqttSendStreamStateTimeout >= 60)
                {
                    mqttSendStreamStateTimeout = 0;

                    app_mqtt_publish_slave_info();
                }

                /* Gửi yêu cầu subscribe nhanh nếu chưa được sub */
                if (++mqtt_fast_subscribe_timeout >= 10)
                { // 10
                    mqtt_fast_subscribe_timeout = 0;

                    if (!app_mqtt_is_subscribed())
                    {
                        app_mqtt_send_subscribe_config_topic(app_flash_get_imei());
                    }
                    /* subscribe master topic nào đã được cấu hình */
                    for (uint8_t master_index = MASTER_TINH1; master_index < MASTER_TOTAL; master_index++)
                    {
                        if (!app_mqtt_is_master_subscribed(master_index) 
                            && strlen(app_flash_get_master(master_index)) >= 15)
                        {
                            app_mqtt_send_subscribe_command_topic(master_index);
                            mqtt_fast_subscribe_timeout = 9;
                            break;
                        }
                    }
                }

                /* Gửi yêu cầu subscribe định kỳ nếu đã được sub -> tránh trường hợp mất subcribe không nhận được lệnh */
                mqttSendSlowSubscribeTimeout++;
                if (mqttSendSlowSubscribeTimeout == 63)
                {
                    app_mqtt_send_subscribe_config_topic(app_flash_get_imei());
                }
                if (mqttSendSlowSubscribeTimeout > 63)
                {
                    for (uint8_t i = masterSubIndex; i < MASTER_TOTAL; i++)
                    {
                        if (strlen(app_flash_get_master(i)) >= 15)
                        {
                            app_mqtt_send_subscribe_command_topic(i);
                            masterSubIndex = i;
                            break;
                        }
                    }
                    masterSubIndex++;
                    if (masterSubIndex >= MASTER_TOTAL)
                    {
                        mqttSendSlowSubscribeTimeout = 1;
                        masterSubIndex = MASTER_TINH1;
                    }
                }

                static uint32_t send_proto_des = 0;
                if (send_proto_des++ == 20)
                {
                    static bool send = false;
                    if (send == false)
                    {
                        static char *protocol_des[] = {"NA", "string", "min"};
                        send = true;
                        char buffer[48];
                        sprintf(buffer, "GD32:%s,FM:%s", 
                                protocol_des[app_io_get_gd32_protocol_method()], 
                                protocol_des[app_io_get_fm_protocol_method()]);
                        mqtt_publish_message("DBG", buffer);
                    }
                }

                /* Gửi bản tin sau reset */
                app_mqtt_send_reset_message();
                if (app_flash_is_tcp_console_enable())
                {
                    tcp_console_start();
                }
                break;

            default:
                break;
            }
        }
        /* ============================ Quản lý ngoại vi ================================= */
        if (m_timeout_turn_off_opto_output)
        {
            m_timeout_turn_off_opto_output--;
            if (m_timeout_turn_off_opto_output == 0)
            {
                /* Nếu đang không chạy các mode module Codec MIC/FM thì mới tắt PA */
                if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
                    && app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_IDLE)
                {
                    app_io_control_pa(APP_IO_PA_OFF);
                }

                /* Nếu đang không chạy mode MIC/FM thì tắt luôn relay */
                if (app_flash_get_operate_mode() != APP_AUDIO_OPERATION_MODE_MIC 
                    && app_flash_get_operate_mode() != APP_AUDIO_OPERATION_MODE_LINE 
                    && app_io_get_current_fm_module() != APP_AUDIO_OPERATION_MODE_MIC 
                    && app_io_get_current_fm_module() != APP_AUDIO_OPERATION_MODE_LINE)
                {
                    // Turn off luôn Relays
                    // app_io_opto_control_all(APP_IO_OPTO_OFF);
                    // 19/12/20: Tắt relay theo thứ tự relay2 -> delay 5s -> relay1
                    ESP_LOGI(TAG,  "m_timeout_turn_off_opto_output = 0, call slave_process_stop_onair...");
                    slave_process_stop_onair();
                }
                m_http_received_data_timeout = 0;
            }
        }
        if (++task_timeout10s >= 200)
        {
            task_timeout10s = 200;
            ESP_LOGI(TAG, "Free heap: %u/%u, streaming: %u, netif: %s", esp_get_free_heap_size(), esp_get_free_internal_heap_size(),
                     total_streaming_received, (netif_default != NULL) ? netif_default->name : "NULL");
            
        }
        if (app_io_is_in_test_mode() && auto_switch_codec_mode_in_test_mode)
        {
            auto_switch_codec_mode_in_test_mode--;
            ESP_LOGI(TAG,  "Auto switch mode %d", auto_switch_codec_mode_in_test_mode);
            if (auto_switch_codec_mode_in_test_mode == 0)
            {
                auto_switch_codec_mode_in_test_mode = 10;
                if (app_audio_get_codec_mode() != APP_AUDIO_CODEC_MODE_LINE_IN)
                {
                    ESP_LOGI(TAG,  "Change to linein");
                    app_audio_change_to_local_line_in_mode();
                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);
                }
                else
                {
                    ESP_LOGI(TAG,  "Change to FM");
                    app_flash_set_current_fm_freq(app_flash_get_fm_freq3());
                    /* Nếu đang không stream internet thì mới cho chạy FM */
                    if (!app_audio_is_http_audio_stream_running() 
                        && !app_audio_is_opus_running()
                        && !app_audio_is_i2s_running())
                    {
                        ESP_LOGI(TAG,  "Switch to FM mode, freq: %u", app_flash_get_current_fm_freq());

                        /** Change to FM mode, PA ON */
                        app_audio_change_to_local_fm_mode();

                        // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                        ESP_LOGI(TAG,  "app_io_opto_control_all OFF...");
                        app_io_opto_control_all(APP_IO_OPTO_OFF);

                        app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_FM); /* 1 - 5 */
                        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);
                    }
                    else
                    {
                        ESP_LOGI(TAG,  "HTTP is running, can't run FM mode!");
                    }
                }
            }
        }

        /* Giám sát trạng thái Streaming của http element */
        if (m_allow_monitor_stream_downloaded)
        {
            if (timeoutHttpStreamCheck++ >= 10)
            {
                timeoutHttpStreamCheck = 0;

                if (total_streaming_received > last_total_streaming_received)
                {
                    ESP_LOGI(TAG,  "HTTP's streaming GOOD!");
                    last_total_streaming_received = total_streaming_received;
                }
                else
                {
                    ESP_LOGI(TAG,  "ERR: HTTP's streaming BAD! Reboot...");
                    if (!app_ota_is_running())
                    {
                        app_audio_change_codec_vol(0);
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        system_software_reset(SW_RESET_REASON_STREAM_START_TIMEOUT);
                    }
                }
            }
        }
        else
        {
            timeoutHttpStreamCheck = 1;
            total_streaming_received = last_total_streaming_received = 0;
        }

        // Timeout lệnh APP_AUDIO_STREAM_START qua MQTT
        if (m_start_stream_cmd_timeout)
        {
            m_start_stream_cmd_timeout--;
            ESP_LOGI(TAG,  "STREAM_START_CMD timeout: %d", m_start_stream_cmd_timeout);
        }

        // Timeout từ lúc nhận được FINISH_TRACK
        if (httpGetFinishTrackEventTimeout)
        {
            httpGetFinishTrackEventTimeout--;
            ESP_LOGI(TAG,  "HTTP_FINISH_TRACK timeout: %u", httpGetFinishTrackEventTimeout);
        }

        if (m_terminate_pipeline_in_stream_running)
        {
            m_terminate_pipeline_in_stream_running--;
            if (m_terminate_pipeline_in_stream_running == 0 && app_audio_get_http_state() == AEL_STATE_RUNNING)
            {
                /* 1.3. Turn off PA after 30s */
                m_timeout_turn_off_opto_output = 15;
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_IDLE);
                m_allow_monitor_stream_downloaded = 0;
                timeoutHttpStreamCheck = 0;

                /** 2. Terminate pipeline */
                ADF_LOGW("Stop stream when no received stream_running");
                app_audio_pause();
                app_audio_complete_terminate();

                app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_STOPPED);
                m_allow_monitor_stream_downloaded = false; /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                m_allow_monitor_stream_downloaded = false;
                ADF_LOGW("Monitor stream =>> disable\r\n");
                mqtt_publish_message("DBG", "Auto stop streamming after 60s");
            }
            if (m_terminate_pipeline_in_stream_running == 0)
            {
                app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, 0);
            }
        }
        // Timeout từ thời điểm nhận lệnh APP_AUDIO_STREAM_START/APP_AUDIO_STREAM_RUNNING
        if (receiveRunningStateTimeout)
        {
            ESP_LOGI(TAG,  "receiveRunningStateTimeout = %d", receiveRunningStateTimeout);
            if (app_audio_is_http_audio_stream_running() 
                && app_audio_is_opus_running()
                && (app_audio_is_i2s_running())
                && (total_streaming_received > last_total_streaming_received))
            {
                /* Sau 2s mà trạng thái vẫn RUNNING và dung lượng stream tăng thì OK */
                if (receiveRunningStateTimeout < 34)
                {
                    ESP_LOGI(TAG,  "Streaming's GOOD!");
                    receiveRunningStateTimeout = 0;
                }
                else
                {
                    receiveRunningStateTimeout--;
                }
            }
            else
            {
                receiveRunningStateTimeout--;
                if (receiveRunningStateTimeout == 0)
                {
                    ADF_LOGE("ERR! Qua lau khong start stream duoc, reset...");
                    mqtt_publish_message("DBG", "Stream timeout, reset");
                    if (!app_ota_is_running())
                    {
                        app_audio_change_codec_vol(0);
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        system_software_reset(SW_RESET_REASON_STREAM_START_TIMEOUT);
                    }
                }
            }
        }

        /* Đếm thời gian stream ở trạng thái RUNNING */
        if (app_audio_is_http_audio_stream_running())
        {
            total_stream_time_in_session++;
            if (lastStreamState == AEL_STATE_STOPPED)
                mqttSendStreamStateTimeout = 55; /* Chuyển trạng thái sang RUNNING 5s thì gửi info */
            lastStreamState = AEL_STATE_RUNNING;

            // Nếu LED Stream không sáng -> ON
            if (!app_io_is_led_streamming_on())
            {
                app_io_control_led_stream(APP_AUDIO_STREAM_RUNNING);
                m_allow_monitor_stream_downloaded = true;
            }
        }
        else
        {
            if (app_io_is_led_streamming_on())
            {
                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                m_allow_monitor_stream_downloaded = false;
            }
            total_stream_time_in_session = 0;
            total_streaming_received = 0;
            lastStreamState = AEL_STATE_STOPPED;
        }

        /* Timeout thời gian khởi tạo stream từ higher master */
        if (waitingHigherMasterStartStreaming)
            waitingHigherMasterStartStreaming--;
        if (m_timeout_wating_next_master_streamming)
            m_timeout_wating_next_master_streamming--;

        /* Kiểm soát chế độ của codec khi đang streaming internet, nếu đang có 1 master streaming -> slave luôn ở mode DECODE */
        if (app_mqtt_get_master_streamming_status()->Value)
        {
            if (app_audio_get_codec_mode() != APP_AUDIO_CODEC_MODE_DECODE)
            {
                ESP_LOGI(TAG,  "CODEC: Force return to INTERNET mode");
                app_audio_change_codec_to_internet_mode();
                break;
            }
        }

        // /** Trạng thái cắm MIC: HW v1.0.2 chỉ detect được khi Relay input đóng vào đường MIC!
        //  * Không detect được đường LINE IN
        //  */
        // uint8_t micState = gpio_get_level(APP_IO_MIC_DETECT_NUM);
        // if (micState != lastMICDetectState)
        // {
        //     ESP_LOGI(TAG,  "MIC state: %s", micState == 0 ? "PLUGGED" : "REMOVED");
        // }
        // xSystem.Status.isMICPlugged = !micState;
        // lastMICDetectState = micState;

        // Kiểm tra trạng thái điều khiển PA, ISORelay khi đang chạy các chế độ phát thanh
        if (total_stream_time_in_session > 1 
            || app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_MIC 
            || app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_LINE 
            || app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_MIC 
            || app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_LINE)
        {
            // Nếu PA đang OFF -> turn ON
            if (app_io_is_pa_off())
            {
                app_io_control_pa(APP_IO_PA_ON);
            }

            // Nếu ISO relays đang OFF -> turn ON
            if (app_io_is_iso_relays_off())
            {
                mqtt_publish_message("DBG", "All opto turn on");
                app_io_opto_control_all(APP_IO_OPTO_ON);
            }
        }

        /* Timeout mất kết nối MQTT server -> mạng có vấn đề */
        mqttDisconnectedTimeout++;

        // ESP_LOGI(TAG,  "mqtt disconnect timeout: %d", mqttDisconnectedTimeout);
        // Chuyển mạng qua lại mà vẫn không kết nối được -> Reset cho nhanh!
        if (mqttDisconnectedTimeout >= NETWORK_ERROR_TIMEOUT_SEC)
        {
            ADF_LOGE("Network is error, reset system...");
            app_audio_change_codec_vol(0);
            vTaskDelay(1000 / portTICK_RATE_MS);
            system_software_reset(SW_RESET_REASON_SRV_TIMEOUT);
        }

        network_interface_t interface = network_get_current_interface();
        if (mqttDisconnectedTimeout % 60 == 0)
        {
            if (interface == NETWORK_INTERFACE_PPP)
            {
                // Nếu có mạng Ethernet -> thử chuyển sang Ethernet
                if (network_is_eth_got_ip())
                {
                    ESP_LOGI(TAG,  "Broker's disconnected, change to ETH");
                    network_change_interface(NETWORK_INTERFACE_ETH);
                }
                else if (network_is_wifi_got_ip())
                {
                    ESP_LOGI(TAG,  "Broker's disconnected, change to WIFI");
                    network_change_interface(NETWORK_INTERFACE_WIFI);
                }
            }
            else if (interface == NETWORK_INTERFACE_ETH)
            {
                if (network_is_wifi_got_ip())
                {
                    ESP_LOGI(TAG,  "Broker's disconnected, change to WIFI");
                    network_change_interface(NETWORK_INTERFACE_WIFI);
                }
                else if (network_is_ppp_got_ip())
                {
                    ESP_LOGI(TAG,  "Broker's disconnected, change to PPP");
                    network_change_interface(NETWORK_INTERFACE_PPP);
                }
            }
            else if (interface == NETWORK_INTERFACE_WIFI)
            {
                if (network_is_eth_got_ip())
                {
                    ESP_LOGI(TAG,  "Broker's disconnected, change to ETH");
                    network_change_interface(NETWORK_INTERFACE_ETH);
                }
                else if (network_is_ppp_got_ip())
                {
                    ESP_LOGI(TAG,  "Broker's disconnected, change to PPP");
                    network_change_interface(NETWORK_INTERFACE_PPP);
                }
            }
        }

        /** Trạng thái các LED tương ứng từng mạng: Chưa connect server: Nháy 1s, connect: sáng đứng
            - LED1 BLUE: PPP
            - LED1 RED: WIFI
            - LED2 RED: ETH
        */
        switch (interface)
        {
        case NETWORK_INTERFACE_UNKNOWN:
        case NETWORK_INTERFACE_PPP:
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                if (!app_io_is_led_4g_on())
                    app_io_control_led_4g_state(1);
            }
            else
            {
                ledState ^= 1;
                app_io_control_led_4g_state(ledState);
            }
            break;

        case NETWORK_INTERFACE_ETH:
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                if (!app_io_is_led_eth_on())
                    app_io_control_led_eth_state(1);
            }
            else
            {
                ledState ^= 1;
                app_io_control_led_eth_state(ledState);
            }
            break;

        case NETWORK_INTERFACE_WIFI:
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                if (!app_io_is_led_wifi_on())
                    app_io_control_led_wifi_state(1);
            }
            else
            {
                ledState ^= 1;
                app_io_control_led_wifi_state(ledState);
            }
            break;
        }

        // 19/12/20: Delay turn on relay when start stream
        if (delayTurnOnRelayPrepairStream > 0)
        {
            delayTurnOnRelayPrepairStream--;
            if (delayTurnOnRelayPrepairStream % 5 == 0)
            {
                ESP_LOGI(TAG,  "Delay turn on Relay2: %d", delayTurnOnRelayPrepairStream);
            }

            if (delayTurnOnRelayPrepairStream == 0)
            {
                // mqtt_publish_message("DBG", "app_io_control_opto_output2(): delayTurnOnRelayPrepairStream = 0");
                app_io_control_opto_output2(APP_IO_OPTO_ON);
            }
        }
        // Quy trinh turn off relay: Delay2 -> turn off relay2 -> delay1 -> turn off relay1
        // Delay turn off relay1
        if (delayTurnOffRelay1StopStream > 0)
        {
            delayTurnOffRelay1StopStream--;
            if (delayTurnOffRelay1StopStream % 10 == 0)
            {
                ESP_LOGI(TAG,  "Delay turn off Relay1: %d", delayTurnOffRelay1StopStream);
            }
            if (delayTurnOffRelay1StopStream == 0)
            {
                app_io_control_opto_output1(APP_IO_OPTO_OFF);
            }
        }
        // Delay turn off relay2
        if (delayTurnOffRelay2StopStream > 0)
        {
            delayTurnOffRelay2StopStream--;
            if (delayTurnOffRelay2StopStream % 5 == 0)
            {
                ESP_LOGI(TAG,  "Delay turn off Relay2: %d", delayTurnOffRelay2StopStream);
            }

            if (delayTurnOffRelay2StopStream == 0)
            {
                app_io_control_opto_output2(APP_IO_OPTO_OFF);
                delayTurnOffRelay1StopStream = app_flash_get_relay1_turn_off_delay();
            }
        }

#if 0
		/* Set lại volume nếu không đúng so với mức cấu hình, chỉ áp dụng khi codec ở chế độ DECODE, 
		* k xét chế độ LINE/MIC vì chế độ này âm lượng thay đổi theo volume ngoài
		*/
		if(++task_tick3s >= 3) {
			task_tick3s = 0;
			if (app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_DECODE) {
				if(abs(app_flash_get_volume() - app_audio_get_current_output_vol()) > 3) {
					app_audio_change_codec_vol(app_flash_get_volume());
				}
			}
		}
#endif

        /* Timeout nhận dữ liệu http khi stream -> chữa bệnh hiển thị "INET" khi master đã stop stream */
        if (m_http_received_data_timeout > 0)
        {
            m_http_received_data_timeout--;
            if (m_http_received_data_timeout == 0)
            {
                m_timeout_turn_off_opto_output = 15;
                mqtt_publish_message("DBG", "httpReceivedDataTimeout = 0");
                ESP_LOGI(TAG,  "--> httpReceivedDataTimeout = 0, m_timeout_turn_off_opto_output = 15s...");
            }
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    ESP_LOGI(TAG,  "\t\t--- main_manager_task is exit ---");
    vTaskDelete(NULL);
}

static void live_streaming_down_task(void *arg)
{
    // esp_err_t err;
    ESP_LOGI(TAG,  "\t\t--- live_streaming_down_task is running ---");
    m_http_received_data_timeout = 0;
    app_audio_start();
    bool restart_stream_when_network_disconnect = false;
    uint8_t delay_stream_after_network_connected = 3;
    bool is_network_connected = false;

    while (1)
    {
        if (!network_is_connected())
        {
            ESP_LOGI(TAG,  "[ * ] WiFi/Eth/PPP is not connected");
            vTaskDelay(1000 / portTICK_RATE_MS);

            /* Nếu đang có mạng -> mất mạng => terminate pipeline luôn */
            if (is_network_connected)
            {
                ADF_LOGE("Network is not connected. Terminate pipeline...");
                // if (m_terminate_pipeline_in_stream_running)
                // {
                //     app_audio_simple_terminate_pipeline();

                //     // Nếu có mạng trở lại -> auto retry restart streaming
                //     m_auto_restart_stream_retries_number = 3;
                //     streamingStep = APP_AUDIO_STREAM_RESTART;
                //     m_allow_monitor_stream_downloaded = false; /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                // }
                restart_stream_when_network_disconnect = true;
            }
            is_network_connected = false;
            delay_stream_after_network_connected = 3;
            continue;
        }

        /* Sau khi có mạng chờ vài giây cho ổn định rồi mới connect */
        if (delay_stream_after_network_connected)
        {
            ESP_LOGI(TAG,  "Delay after network connected: %us", delay_stream_after_network_connected);
            delay_stream_after_network_connected--;
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }
        
        if (restart_stream_when_network_disconnect)
        {
            restart_stream_when_network_disconnect = false;
            if (network_debug_and_clear_ppp_disconnect_status())
            {
                mqtt_publish_message("DBG", "Network PPP disconnect, reconnect again");
            }
            else
            {
                mqtt_publish_message("DBG", "Network PPP unknown error");
            }
        }
        is_network_connected = true;

        /** Chờ HttpElement stopped sau lệnh terminate_pipeline khi nhận lệnh 'APP_AUDIO_STREAM_START' hoặc 'APP_AUDIO_STREAM_RUNNING'
         * Khi stopped thì delay vài giây rồi mới restart STREAM
         */
        if (app_mqtt_is_waiting_http_element_stopped())
        {
            audio_element_state_t el_http_state = app_audio_get_http_state();
            if (el_http_state == AEL_STATE_STOPPED || el_http_state == AEL_STATE_INIT)
            {
                ESP_LOGI(TAG,  "el_http_state is already stopped, can restart stream now...");

                app_mqtt_is_reset_http_element_stopped_flag();
                m_waiting_http_element_stopped_timeout = 0;

                /**01/05/20: test delay before start streaming, nếu start stream luôn thì bị hiện tượng HTTP báo
                 * HTTP_STREAM_FINISH_TRACK -> Có thể do server đặt Latency Buffer lớn -> Stream ngay thì chưa có
                 * nội dung -> báo FINISH_TRACK. Để delay 5s thì không thấy bị
                 */
                ESP_LOGI(TAG,  "STREAM: Delay 5s before start streaming!");
                mqtt_publish_message("DBG", "http_state is already stopped, restart stream now");
                vTaskDelay(5000 / portTICK_RATE_MS);
                m_main_task_hangout = 0;
                app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_RESTART);
            }
            else
            {
                vTaskDelay(100 / portTICK_RATE_MS);

                if (m_waiting_http_element_stopped_timeout++ >= 150)
                { /* 15s */
                    // Chờ lâu mà httpElement không stopped được -> reset cho nhanh!
                    ADF_LOGE("m_waiting_http_element_stopped_timeout is too long! Reset system...");
                    if (!app_ota_is_running())
                    {
                        app_audio_change_codec_vol(0);
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        system_software_reset(SW_RESET_REASON_HTTP_EL_STOP_TIMEOUT);
                    }
                }
                else
                {
                    continue;
                }
            }
        }

        if (m_auto_restart_stream_timeout)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            m_auto_restart_stream_timeout--;
            if (m_auto_restart_stream_timeout == 0)
            {
                ESP_LOGI(TAG,  "[ * ] Auto Restart pipeline -> %s", app_audio_get_stream_url());
                app_audio_run_pipeline();
                app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_RUNNING);
            }
        }

        /* ============================ Chỉ chạy stream internet khi chế độ hoạt động cho phép ============================== */
        if (app_flash_get_operate_mode() != APP_AUDIO_OPERATION_MODE_NO_OPERATION)
        {
            /* Sau khi khởi động thử stream luôn, nếu không có link -> http_err_open */
            if (app_audio_get_streamming_logic_step() == APP_AUDIO_STREAM_NOT_INIT)
            {
                ESP_LOGI(TAG,  "[ * ] Start stream the 1st time -> %s", app_audio_get_stream_url());

                // audio_element_set_uri(m_http_stream_reader, app_audio_get_stream_url());
                // audio_pipeline_run(pipeline);
                app_audio_run_new_url(app_audio_get_stream_url());

                app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_RUNNING);
                mqtt_publish_message("DBG", "STREAM_ONREBOOT");
            }

            if (app_audio_get_streamming_logic_step() == APP_AUDIO_STREAM_START)
            {
                ESP_LOGI(TAG,  "[ * ] Start pipeline -> %s", app_audio_get_stream_url());

                // audio_element_set_uri(m_http_stream_reader, app_audio_get_stream_url());
                // audio_pipeline_run(pipeline);
                app_audio_run_new_url(app_audio_get_stream_url());

                app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_RUNNING);
                mqtt_publish_message("DBG", "STREAM_START");
            }

            if (app_audio_get_streamming_logic_step() == APP_AUDIO_STREAM_RESTART)
            {
                ESP_LOGI(TAG,  "[ * ] Re-start pipeline -> %s", app_audio_get_stream_url());

                app_audio_pause();
                app_audio_complete_terminate();
                // Neu terminate luon thi bi bug stack no chua terminate xong ma da cau hinh
                vTaskDelay(2000 / portTICK_RATE_MS);

                ESP_LOGI(TAG,  "Stream url %s\r\n", app_audio_get_stream_url());
                app_audio_run_new_url(app_audio_get_stream_url());

                app_audio_set_streamming_logic_step(APP_AUDIO_STREAM_RUNNING);
                mqtt_publish_message("DBG", "STREAM_RESTART");
            }
        }

        app_audio_wait_for_event(1000 / portTICK_RATE_MS);
    }
    ESP_LOGI(TAG,  "\t\t--- app_streaming_task is exit ---");
    esp_restart();
}

/*
 * This function is called by task_wdt_isr function (ISR for when TWDT times out).
 * It can be redefined in user code to handle twdt events.
 * Note: It has the same limitations as the interrupt function.
 *       Do not use ESP_LOGI functions inside.
 */
void esp_task_wdt_isr_user_handler(void)
{
    ets_printf("\n\n\n!!! System is hangup. Restart now...!!!\n\n\n");

    /* reset system */
    system_software_reset(SW_RESET_REASON_TWDT_TIMEOUT);
}

/******************************************************************************************/
/**
 * @brief   : app_main
 * @param   :
 * @retval  : ESP_OK or ESP_FAIL
 * @author  :
 * @created :
 */
void app_main(void)
{
    esp_err_t err;
    // esp_log_level_set(TAG, ESP_LOG_DEBUG | ESP_LOG_INFO | ESP_LOG_ERROR | ESP_LOG_WARN);
    // esp_log_level_set(TAG, ESP_LOG_NONE);
    /* Reconfig Debug UART0 */
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_DEF_OUTPUT);
    uart_set_pin(UART_NUM_0, 1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //	ESP_LOGI*TAG,  "\t\t============ Firmware Version ", __FIRMWARE_VERSION__);
    ADF_LOGD("[ZIG] FW_VERSION: %s\r\n", __FIRMWARE_VERSION__);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ADF_LOGD("ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision %d, %dMB %s flash",
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             chip_info.revision, spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    /* Get Unique ID */
    uint8_t uid[6];
    err = esp_efuse_mac_get_default(uid);
    if (err == ESP_OK)
    {
        //		ESP_LOGI*TAG,  "ESP unique MAC: %02X:%02X:%02X:%02X:%02X:%02X", uid[0],
        //			uid[1], uid[2], uid[3],
        //			uid[4], uid[5]);

        ADF_LOGZ("[ZIG] MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
                uid[0], uid[1], uid[2], 
                uid[3], uid[4], uid[5]);
    }
    else
    {
        //		ADF_LOGE( "ESP unique MAC error!");
        ADF_LOGZ("[ZIG] MAC: ERROR\r\n");
    }

    /* =========================== Init GPIO ============================== */
    // Khởi tạo GPIO & Timer luôn để reset watchdog, nếu k reset kịp sẽ bị wdg reset!
    app_io_initialize();

    /* Timer xử lý các tác vụ phụ */
    TimerHandle_t timer = xTimerCreate("system_timer", 10 / portTICK_RATE_MS, true, NULL, xSystem_timercb);
    xTimerStart(timer, 0);

    app_flash_node_nvs_initialize();

    app_flash_slave_nvs_read_params("ALL");

    /* Get reset reason
     * Tắt nguồn bật lại: 9 - 14 hoặc 16 - 14 hoặc 1 - 14
     * Nhấn nút reset mạch nạp: 1 - 14
     * Reset mềm: 12 - 12 (SW_CPU_RESET)
     */
    uint8_t cpu0_reset_reason = rtc_get_reset_reason(0);
    uint8_t cpu1_reset_reason = rtc_get_reset_reason(1);
    ESP_LOGI(TAG,  "CPU reset reason: %d - %d", cpu0_reset_reason, cpu1_reset_reason);

    /* Init m_http_stream_url */
    if (app_flash_get_current_streamming_master_index() >= MASTER_TINH1 
        && app_flash_get_current_streamming_master_index() <= APP_FLASH_MASTER_XA3 
        && strlen(app_flash_get_master(app_flash_get_current_streamming_master_index())) >= 15)
    {
        sprintf(app_audio_get_stream_url(),
                "%s%s", 
                app_flash_get_http_stream_header(), 
                app_flash_get_master(app_flash_get_current_streamming_master_index()));
    }
    else
    {
#warning "TueTD Lam do an"
        //sprintf(app_audio_get_stream_url(), "%s%s", app_flash_get_http_stream_header(), "NoLink");
        sprintf(app_audio_get_stream_url(), "%s", app_flash_get_http_stream_header());
    }

    network_enable_wifi(1);
    network_wifi_info_t info;
    info.ssid = app_flash_get_wifi_name();
    info.password = app_flash_get_wifi_pass();

    /* Khởi tạo audio codec để khởi tạo I2C điều khiển PCF */
    bool retval = app_audio_board_init();
    /*Known issued:
    (327) psram: This chip is ESP32-D0WD
    (328) spiram: SPI RAM enabled but initialization failed. Bailing out.
    (328) cpu_start: Failed to init external RAM; continuing without it.
    //Case khởi tạo thành công PSRAM:
    (299) spiram: Found 64MBit SPI RAM device
    (299) spiram: SPI RAM mode: flash 80m sram 80m
    (301) spiram: PSRAM initialized, cache is in low/high (2-core) mode.
    ....
    (444) AUDIO_BOARD_BYT_V102: audio_board_init...
    (454) AUDIO_BOARD_BYT_V102: ../esp/esp-adf-v2.0-beta2/components/audio_board/lyrat_v4_3_byt_v102/board.c:47 (audio_board_init): Memory exhausted[1B][0m
    (464) i2c: ../esp/esp-adf-v2.0-beta2/esp-idf/components/driver/i2c.c:1267 (i2c_master_cmd_begin):i2c driver not installed[1B][0m
    (484) [LIVEDOWN, 5857]: PCF8575 init: ERR[1B][0m
    (*) REASON: Do lỗi khởi tạo PSRAM dẫn đến khi khởi tạo audio_board_init() không đủ bộ nhớ để calloc -> board_handle null
        => không chạy được hàm audio_board_codec_init() -> không khởi tạo I2C và audio codec được -> init PCF8575 cũng failed luôn!
    (*) TODO: check board_handle sau khi call audio_board_init(), nếu = NULL là FAILED, lưu lại nguyên nhân và reset!
    */
    if (!retval)
    {
        ADF_LOGE("---------> [ERROR] Cannot init audio_board_init() because PSRAM init failed. Reboot...");
        ADF_LOGZ("[ZIG] PSRAM: Error\r\n");
        system_software_reset(SW_RESET_REASON_PSRAM_FAIL);
    }
    else
    {
        // ESP_LOGI(TAG,  "[ 2 ] Init board_handle: OK!");
        // ADF_LOGZ("[ZIG] PSRAM: OK\r\n");
    }

    app_io_set_hardware_version(3);

    /* ========================= Khởi tạo GPIO mở rộng kèm detect version PCF ====================== */
    /**
     * Nếu khởi động do tắt nguồn và bật lại -> mặc định OFF các relay
     * Sau khi update firmware -> mặc định OFF các relay
     */
    bool save_opto_state = false;
    if (cpu1_reset_reason == 14)
    {
        if (cpu0_reset_reason == 1 || cpu0_reset_reason == 16)
        {
            ESP_LOGI(TAG,  "CPU reset by power on, turn off all relay...");
            app_io_get_io_value()->Name.IO1 = APP_IO_OPTO_OFF;
            app_io_get_io_value()->Name.IO2 = APP_IO_OPTO_OFF;
            save_opto_state = true;
        }
    }
    if (system_get_software_reset_reason() == SW_RESET_REASON_OTA_FINISH)
    {
        ESP_LOGI(TAG,  "CPU reset by OTA, turn off all relay...");
        app_io_get_io_value()->Name.IO1 = APP_IO_OPTO_OFF;
        app_io_get_io_value()->Name.IO2 = APP_IO_OPTO_OFF;
        save_opto_state = true;
    }
    if (save_opto_state)
    {
        app_flash_write_u8(APP_FLASH_IO_STATE_KEY, app_io_get_io_value()->Value);
    }

    app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_OFF;
    app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
    app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;
    app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;

    /* Các chân ISO Ouput khởi tạo theo cấu hình cài đặt */
    /* Mặc định khởi tạo OFF, khi nào chạy stream hoặc MIC/LINE thì bật */
    // app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_OPTO_OFF;
    // app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_OPTO_OFF;
    //-> Điều khiển relay về trạng thái ON/OFF trước đó
    app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = app_io_get_io_value()->Name.IO1;
    app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = app_io_get_io_value()->Name.IO2;

    // GSM OFF
    app_io_get_i2c_exp_value()->BitName.GSM_PWR_EN = 0;
    app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 0;

    // Swith Audio input relay
    app_io_get_i2c_exp_value()->BitName.SW_MIC_AUX = 0; // Default to AUX input

    // Switch Ouput Codec - FM
    app_io_get_i2c_exp_value()->BitName.SW_CODEC_FM = 0; // Relay to Codec ouput

    // PA enable
    app_io_get_i2c_exp_value()->BitName.EN_PA = APP_IO_PA_OFF;

    // Input
    app_io_get_i2c_exp_value()->BitName.ISO_IN1 = 1;
    app_io_get_i2c_exp_value()->BitName.ISO_IN2 = 1;
    app_io_get_i2c_exp_value()->BitName.BTN_RESET = 1;
    app_io_get_i2c_exp_value()->BitName.BTN_SET = 1;

    Int_t ioEx;
    ioEx.value = app_io_get_i2c_exp_value()->Value;
    uint8_t retry_detect_num = 10;
    while (retry_detect_num > 0)
    {
        retry_detect_num--;
        err = pcf_i2c_write(I2C_NUM_0, PCF8575_I2C_ADDR, ioEx.bytes, 2);
        // ESP_LOGI(TAG,  "[ 3 ] Init PCF8575: %s", err == ESP_OK ? "OK" : "ERR");

        if (err != ESP_OK)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        else
        {
            app_io_set_hardware_version(2);
            break;
        }
    }
    ESP_LOGI(TAG,  "[ 3 ] Detect hardware version = %d", app_io_get_hardware_version());
    ADF_LOGZ("[ZIG] IOEXT_VERSION: %s\r\n", app_io_get_hardware_version() == 2 ? "PCF85xx" : "MCU");

    for (int32_t i = 10; i > 0; i--)
    {
        app_audio_change_codec_vol(app_flash_get_volume()/i);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    app_audio_change_codec_vol(app_flash_get_volume());

    // if (HW_VERSION == 2) {
    //	//========================== Khởi tạo PCF8575 ===========================//
    //	app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_OFF;
    //	app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
    //	app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;
    //	app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
    //
    //	/* Các chân ISO Ouput khởi tạo theo cấu hình cài đặt */
    //	/* Mặc định khởi tạo OFF, khi nào chạy stream hoặc MIC/LINE thì bật */
    //	// app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_OPTO_OFF;
    //	// app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_OPTO_OFF;
    //	//-> Điều khiển relay về trạng thái ON/OFF trước đó
    //	app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = app_io_get_io_value()->Name.IO1;
    //	app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = app_io_get_io_value()->Name.IO2;
    //
    //	//GSM OFF
    //	app_io_get_i2c_exp_value()->BitName.GSM_PWR_EN = 0;
    //	app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 0;
    //
    //	//Swith Audio input relay
    //	app_io_get_i2c_exp_value()->BitName.SW_MIC_AUX = 0;		//Default to AUX input
    //
    //	//Switch Ouput Codec - FM
    //	app_io_get_i2c_exp_value()->BitName.SW_CODEC_FM = 0;		//Relay to Codec ouput
    //
    //	//PA enable
    //	app_io_get_i2c_exp_value()->BitName.EN_PA = APP_IO_PA_OFF;
    //
    //	//Input
    //	app_io_get_i2c_exp_value()->BitName.ISO_IN1 = 1;
    //	app_io_get_i2c_exp_value()->BitName.ISO_IN2 = 1;
    //	app_io_get_i2c_exp_value()->BitName.BTN_RESET = 1;
    //	app_io_get_i2c_exp_value()->BitName.BTN_SET = 1;
    //
    //	Int_t ioEx;
    //	ioEx.value = app_io_get_i2c_exp_value()->Value;
    //	err = pcf_i2c_write(I2C_NUM_0, PCF8575_I2C_ADDR, ioEx.bytes, 2);
    //	ESP_LOGI*TAG,  "[ 3 ] Init PCF8575: %s", err == ESP_OK ? "OK" : "ERR");
    //	vTaskDelay(1000 / portTICK_PERIOD_MS);
    //	/* ======================= End of PCF8575 ===================================*/
    // }

    // xSystem.Status.Speaker.Value = 255;

    //===================================================================================================//
    if (app_io_get_hardware_version() == 3) // Khởi tạo UART giao tiếp STM32/GD32 trước để điều khiển IO
    {
        /**
         * @brief uart handle task: Giao tiếp UART module FM
         */
        app_sntp_start();
        app_io_init_min_protocol();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        app_io_set_default_value();
    }
    /* ======================End of Khởi tạo GPIO mở rộng ======================*/
    network_initialize(&info, network_event_cb);
    
    app_mqtt_initialize();
    /**
     * @brief Tạo task quản lý hoạt động của main
     */
    xTaskCreate(main_manager_task, "main_manager_task", 5 * 1024, NULL, 5, NULL);

    if (app_io_get_hardware_version() == 2)
    { // Khởi tạo UART giao tiếp FM module sau khi đã khởi tạo xong audio codec
        /**
         * @brief uart handle task: Giao tiếp UART module FM
         */
        app_io_expander_uart_initialize();
        xTaskCreate(app_io_exp_uart_task, "fm_uart_task", 3 * 1024, NULL, 5, NULL);
    }

    app_io_allow_process_fm_data();

    /* Task http stream */
    live_streaming_down_task(NULL);
}
