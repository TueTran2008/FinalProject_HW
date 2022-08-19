#include "app_flash.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "string.h"
#include "stdlib.h"
#include "app_audio.h"
#include "esp_log.h"
#include "DataDefine.h"
#include "app_io.h"
#include "utilities.h"

static const char *TAG = "app_flash";
static esp_err_t node_nvs_write_u8(char *key, uint8_t value);
static char m_gsm_imei[26] = "NA";
static char m_wifi_name[50];
static char m_wifi_pass[50];
static uint8_t m_wifi_enable;
static app_audio_operation_mode_t m_operate_mode;
static char m_http_url_header[128];	
static char m_mqtt_server_url[APP_FLASH_MQTT_URL_SIZE];
static char m_mqtt_server_user_name[96];
static char m_mqtt_server_password[96];
static uint32_t m_total_reset_time = 0;
static uint8_t m_speaker_detect_enable = 0;
static uint8_t m_volume;
static uint32_t m_current_freq;

static uint32_t m_freq1;
static uint32_t m_freq2;
static uint32_t m_freq3;
static uint8_t m_delay_turn_off_relay1;
static uint16_t m_delay_turn_off_relay2;
static uint8_t m_delay_turn_on_relay;
static char m_master_imei[APP_FLASH_MAX_SUPPORT_MASTER][APP_FLASH_MASTER_IMEI_SIZE];
static uint8_t m_current_master;
static uint8_t m_tcp_console_enable = 0;

uint8_t app_flash_get_current_streamming_master_index(void)
{
    return m_current_master;
}

void app_flash_set_current_streamming_master(uint8_t master)
{
    m_current_master = master;
}

char *app_flash_get_master(uint8_t index)
{
    return m_master_imei[index];
}

uint8_t app_flash_get_delay_turn_on_relay(void)
{
    return m_delay_turn_on_relay;
}

void app_flash_set_delay_turn_on_relay(uint8_t delay)
{
    m_delay_turn_on_relay = delay;
}

uint8_t app_flash_get_relay1_turn_off_delay(void)
{
    return m_delay_turn_off_relay1;
}

uint16_t app_flash_get_relay2_turn_off_delay(void)
{
    return m_delay_turn_off_relay2;
}

void app_flash_set_relay1_turn_off_delay(uint8_t delay)
{
    m_delay_turn_off_relay1 = delay;
}

void app_flash_set_relay2_turn_off_delay(uint16_t delay)
{
    m_delay_turn_off_relay2 = delay;
}

uint32_t app_flash_get_current_fm_freq(void)
{
    return m_current_freq;
}

uint32_t app_flash_get_fm_freq1(void)
{
    return m_freq1;
}

uint32_t app_flash_get_fm_freq2(void)
{
    return m_freq2;
}

uint32_t app_flash_get_fm_freq3(void)
{
    return m_freq3;
}

void app_flash_set_fm_freq1(uint32_t freq)
{
    m_freq1 = freq;
}

void app_flash_set_fm_freq2(uint32_t freq)
{
    m_freq2 = freq;
}

void app_flash_set_fm_freq3(uint32_t freq)
{
    m_freq3 = freq;
}

void app_flash_set_current_fm_freq(uint32_t freq)
{
    m_current_freq = freq;
}

uint8_t app_flash_get_volume(void)
{
    return m_volume;
}

void app_flash_set_volume(uint8_t volume)
{
    m_volume = volume;
}

bool app_flash_speaker_detect_is_enable(void)
{
    return m_speaker_detect_enable ? true : false;
}

void app_flash_speaker_detect_set_enable(bool enable)
{
    m_speaker_detect_enable = enable ? 1 : 0;
}

char *app_flash_get_mqtt_server_url(void)
{
    return m_mqtt_server_url;
}

char *app_flash_get_mqtt_server_username(void)
{
    return m_mqtt_server_user_name;
}

char *app_flash_get_mqtt_server_password(void)
{
    return m_mqtt_server_password;
}

char *app_flash_get_http_stream_header(void)
{
    return m_http_url_header;
}

char *app_flash_get_imei(void)
{
    return m_gsm_imei;
}

void app_flash_set_imei(char *imei)
{
    memset(m_gsm_imei, 0, sizeof(m_gsm_imei));
    snprintf(m_gsm_imei, sizeof(m_gsm_imei)-1, "%s", imei);
    app_flash_node_nvs_write_string(APP_FLASH_GSM_IMEI_KEY, imei);
}

uint32_t app_flash_get_total_reset_time(void)
{
    return m_total_reset_time;
}

uint8_t app_flash_get_operate_mode(void)
{
    return m_operate_mode;
}

void app_flash_set_operate_mode(uint8_t mode)
{
    m_operate_mode = mode;
}

esp_err_t app_flash_node_nvs_write_string(char *key, char *content)
{
    nvs_handle my_nvs_handle;
    esp_err_t ret = ESP_OK;

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    ret |= err;

    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "\tWriting to NVS -> %s", content);

        err = nvs_set_str(my_nvs_handle, key, content);
        ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        ESP_LOGI(TAG, "\tCommitting updates string in NVS ... ");
        err = nvs_commit(my_nvs_handle);
        ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Close NVS
        nvs_close(my_nvs_handle);
    }

    return ret;
}

/******************************************************************************************/
/**
 * @brief 	: Write uint32 value to NVS
 * @param 	: key, str
 * @author	:
 * @return 	: 0 if Failed, 1 if OK
 */
esp_err_t app_flash_node_nvs_write_u32(char *key, uint32_t value)
{
    nvs_handle my_nvs_handle;
    esp_err_t ret = ESP_OK;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    ret |= err;

    if (err != ESP_OK)
    {
        //		ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        //		ESP_LOGI(TAG, "\tWriting to NVS...");

        // Write value
        err = nvs_set_u32(my_nvs_handle, key, value);
        ret |= err;
        //		ESP_LOGI(TAG, "NVS write uint8: %s", (err != ESP_OK) ? "ERR" : "OK");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        //		ESP_LOGI(TAG, "\tCommitting updates uint8 value in NVS ... ");
        err = nvs_commit(my_nvs_handle);
        ret |= err;
        //		ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");

        // Close NVS
        nvs_close(my_nvs_handle);
    }

    return ret;
}


static esp_err_t node_nvs_write_u8(char *key, uint8_t value)
{
    nvs_handle my_nvs_handle;
    esp_err_t ret = ESP_OK;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    ret |= err;

    if (err == ESP_OK)
    {
        //		ESP_LOGI(TAG, "\tWriting to NVS...");

        // Write value
        uint8_t tmp;
        err = nvs_get_u8(my_nvs_handle, key, &tmp);
        if (err != ESP_OK || tmp != value)
        {
            err = nvs_set_u8(my_nvs_handle, key, value);
        }
        ret |= err;
        //		ESP_LOGI(TAG, "NVS write uint8: %s", (err != ESP_OK) ? "ERR" : "OK");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        //		ESP_LOGI(TAG, "\tCommitting updates uint8 value in NVS ... ");
        err = nvs_commit(my_nvs_handle);
        ret |= err;
        //		ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");

        // Close NVS
        nvs_close(my_nvs_handle);
    }
    return ret;
}

void app_flash_write_u8(char *key, uint8_t value)
{
    node_nvs_write_u8(key, value);
}

/******************************************************************************************/
/**
 * @brief 	: Write uint16 value to NVS
 * @param 	: key, str
 * @author	:
 * @return 	: 0 if Failed, 1 if OK
 */
static esp_err_t node_nvs_write_u16(char *key, uint16_t value)
{
    nvs_handle my_nvs_handle;
    esp_err_t ret = ESP_OK;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    ret |= err;

    if (err != ESP_OK)
    {
        //		ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        uint16_t tmp;
        err = nvs_get_u16(my_nvs_handle, key, &tmp);
        if (err != ESP_OK || tmp != value)
        {
            err = nvs_set_u16(my_nvs_handle, key, value);
        }

        ret |= err;
        //		ESP_LOGI(TAG, "NVS write uint8: %s", (err != ESP_OK) ? "ERR" : "OK");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        //		ESP_LOGI(TAG, "\tCommitting updates uint8 value in NVS ... ");
        err = nvs_commit(my_nvs_handle);
        ret |= err;
        //		ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");

        // Close NVS
        nvs_close(my_nvs_handle);
    }

    return ret;
}

void app_flash_write_u16(char *key, uint16_t value)
{
    node_nvs_write_u16(key, value);
}

/******************************************************************************************/
/**
 * @brief 	: Read parameters from VNS
 * @param 	:
 * @author	:
 * @return 	:
 */
void app_flash_slave_nvs_read_params(char *paramName)
{
    //	uint8_t isReadAll = 0;
    //	uint8_t paramCount = 1;

    //	if(strstr(paramName, "ALL")) isReadAll = 1;
    nvs_handle my_nvs_handle;

    ESP_LOGD(TAG, "\tOpening NVS handle...");
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGD(TAG, "\tOpen NVS OK. Reading parameters from NVS...");

        /* ====== WIFI INFOR ==========*/
        size_t wifi_name_len = sizeof(m_wifi_name);
        err = nvs_get_str(my_nvs_handle, APP_FLASH_WIFI_NAME_KEY, m_wifi_name, &wifi_name_len);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tWIFI name: %s", m_wifi_name);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            // Sử dụng wifi mặc định
            sprintf(m_wifi_name, "%s", "BYTECH_T2");
            ESP_LOGI(TAG, "\tNot found WiFi name. Use default!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        size_t wifiPassLength = sizeof(m_wifi_pass);
        err = nvs_get_str(my_nvs_handle, APP_FLASH_WIFI_PASS_KEY, m_wifi_pass, &wifiPassLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tWIFI pass: %s", m_wifi_pass);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            // Sử dụng wifi mặc định
            sprintf(m_wifi_pass, "%s", "bytech@2020");
            ESP_LOGI(TAG, "\tNot found WiFi pass. Use default!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        int8_t wifiEnalbe = 0;
        err = nvs_get_i8(my_nvs_handle, APP_FLASH_WIFI_ENABLE_KEY, &wifiEnalbe);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tWIFI enable: %d", wifiEnalbe);
            if (wifiEnalbe > 1 || wifiEnalbe < 0)
                wifiEnalbe = 0;
            m_wifi_enable = wifiEnalbe;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "\tNot found WiFi enable!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }

        /* ====== GSM IMEI ==========*/
        size_t gsm_imei_len = sizeof(m_gsm_imei);
        err = nvs_get_str(my_nvs_handle, APP_FLASH_GSM_IMEI_KEY, m_gsm_imei, &gsm_imei_len);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tGSM IMEI: %s", m_gsm_imei);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found GSM IMEI!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MASTER T1 ==========*/
        size_t masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_T1_KEY, m_master_imei[MASTER_TINH1], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "\tMaster_T1: %s", m_master_imei[MASTER_TINH1]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_T1!");
            sprintf(m_master_imei[MASTER_TINH1], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MASTER T2 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_T2_KEY, m_master_imei[MASTER_TINH2], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "\tMaster_T2: %s", m_master_imei[MASTER_TINH2]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_T2!");
            sprintf(m_master_imei[MASTER_TINH2], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MASTER T3 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_T3_KEY, m_master_imei[MASTER_TINH3], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "\tMaster_T3: %s", m_master_imei[MASTER_TINH3]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found Master_T3!");
            sprintf(m_master_imei[MASTER_TINH3], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }

        /* ====== MASTER H1 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_H1_KEY, m_master_imei[APP_FLASH_MASTER_HUYEN1], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "\tMaster_H1: %s", m_master_imei[APP_FLASH_MASTER_HUYEN1]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_H1!");
            sprintf(m_master_imei[APP_FLASH_MASTER_HUYEN1], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MASTER H2 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_H2_KEY, m_master_imei[APP_FLASH_MASTER_HUYEN2], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "\tMaster_H2: %s", m_master_imei[APP_FLASH_MASTER_HUYEN2]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_H2!");
            sprintf(m_master_imei[APP_FLASH_MASTER_HUYEN2], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MASTER H3 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_H3_KEY, m_master_imei[APP_FLASH_MASTER_HUYEN3], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGD(TAG, "\tMaster_H3: %s", m_master_imei[APP_FLASH_MASTER_HUYEN3]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_H3!");
            sprintf(m_master_imei[APP_FLASH_MASTER_HUYEN3], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }

        /* ====== MASTER X1 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_X1_KEY, m_master_imei[APP_FLASH_MASTER_XA1], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tMaster_X1: %s", m_master_imei[APP_FLASH_MASTER_XA1]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_X1!");
            sprintf(m_master_imei[APP_FLASH_MASTER_XA1], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MASTER X2 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_X2_KEY, m_master_imei[APP_FLASH_MASTER_XA2], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tMaster_X2: %s", m_master_imei[APP_FLASH_MASTER_XA2]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_X2!");
            sprintf(m_master_imei[APP_FLASH_MASTER_XA2], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MASTER X3 ==========*/
        masterLength = APP_FLASH_MASTER_IMEI_SIZE;
        err = nvs_get_str(my_nvs_handle, APP_FLASH_MASTER_X3_KEY, m_master_imei[APP_FLASH_MASTER_XA3], &masterLength);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tMaster_X3: %s", m_master_imei[APP_FLASH_MASTER_XA3]);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGI(TAG, "\tNot found Master_X3!");
            sprintf(m_master_imei[APP_FLASH_MASTER_XA3], "000");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }

        /* ====== HTTP URL header ==========*/
        size_t url_length = sizeof(m_http_url_header);
        err = nvs_get_str(my_nvs_handle, APP_FLASH_HTTP_URL_HEADER_KEY, m_http_url_header, &url_length);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tUrl header: %s", m_http_url_header);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found Url header!");
            sprintf(m_http_url_header, "%s", APP_FLASH_HTTP_STREAM_HEADER);
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            sprintf(m_http_url_header, "%s", APP_FLASH_HTTP_STREAM_HEADER);
            break;
        }

        /* ====== MQTT server ==========*/
        url_length = sizeof(m_mqtt_server_url);
        err = nvs_get_str(my_nvs_handle, APP_FLASH_KEY_MQTT_SERVER_URL, m_mqtt_server_url, &url_length);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tUrl mqtt header: %s", m_mqtt_server_url);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found mqtt Url header!");
            sprintf(m_mqtt_server_url, "%s", APP_FLASH_BROKER_URL);
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            sprintf(m_mqtt_server_url, "%s", APP_FLASH_BROKER_URL);
            break;
        }

        url_length = sizeof(m_mqtt_server_user_name);
        err = nvs_get_str(my_nvs_handle, APP_FLASH_KEY_MQTT_SERVER_USERNAME, m_mqtt_server_user_name, &url_length);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tUrl mqtt username: %s", m_mqtt_server_user_name);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found mqtt username header!");
            sprintf(m_mqtt_server_user_name, "%s", APP_FLASH_MQTT_USERNAME);
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            sprintf(m_mqtt_server_user_name, "%s", APP_FLASH_MQTT_USERNAME);
            break;
        }

        url_length = sizeof(m_mqtt_server_password);
        err = nvs_get_str(my_nvs_handle, APP_FLASH_KEY_MQTT_SERVER_PASSWORD, m_mqtt_server_password, &url_length);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tUrl mqtt username: %s", m_mqtt_server_password);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found mqtt username header!");
            sprintf(m_mqtt_server_password, "%s", APP_FLASH_MQTT_PASSWORD);
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            sprintf(m_mqtt_server_password, "%s", APP_FLASH_MQTT_PASSWORD);
            break;
        }

        /* ====== FM_FREQ1 ==========*/
        err = nvs_get_u32(my_nvs_handle, APP_FLASH_FM_FREQ1_KEY, &m_freq1);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tFM_FREQ1: %u", m_freq1);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found FM_FREQ1!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== FM_FREQ2 ==========*/
        err = nvs_get_u32(my_nvs_handle, APP_FLASH_FM_FREQ2_KEY, &m_freq2);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tFM_FREQ2: %u", m_freq2);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found FM_FREQ2!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== FM_FREQ3 ==========*/
        err = nvs_get_u32(my_nvs_handle, APP_FLASH_FM_FREQ3_KEY, &m_freq3);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tFM_FREQ3: %u", m_freq3);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found FM_FREQ3!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!", esp_err_to_name(err));
            break;
        }
        /* ====== MODE ==========*/
        uint8_t operate_mode;
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_OPERATE_MODE_KEY, &operate_mode);
        switch (err)
        {
        case ESP_OK:
            m_operate_mode = operate_mode;
            ESP_LOGI(TAG, "\tOperate mode: %u", m_operate_mode);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found Mode!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }
        /* ====== VOLUME chế độ decode ==========*/
        m_volume = APP_FLASH_VOLUME_DEFAULT;
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_VOLUME_KEY, &m_volume);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tVolume: %u", m_volume);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found Volume!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }
        /* ====== IO STATE ==========*/
        app_io_get_io_value()->Value = 0; // mặc định ban đầu là OFF hết
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_IO_STATE_KEY, &app_io_get_io_value()->Value);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tIO control: %u", app_io_get_io_value()->Value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found IOState!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }

        /* ====== Speaker detect ==========*/
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_SPK_DETECT, &m_speaker_detect_enable);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tSPK detect: %u", app_io_get_io_value()->Value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            m_speaker_detect_enable = 0;
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }

        /* ====== Current Master ==========*/
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_CURRENT_MASTER_KEY, &m_current_master);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tCurrent master: %u", m_current_master);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "\tNot found current master!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }
        /* ====== Current Frequency ==========*/
        err = nvs_get_u32(my_nvs_handle, APP_FLASH_CURRENT_FREQ_KEY, &m_current_freq);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tCurrent freq: %u", m_current_freq);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "\tNot found current freq!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }
        /* ====== RELAY DELAY ON TIME ========== */
        m_delay_turn_on_relay = APP_FLASH_RELAY_DELAY_ON_DEFAULT;
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_RELAY_DELAY_ON_KEY, &m_delay_turn_on_relay);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tRelay delay On: %u", m_delay_turn_on_relay);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found DelayOn!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }
        /* ====== RELAY1 DELAY OFF TIME ========== */
        m_delay_turn_off_relay1 = APP_FLASH_RELAY_DELAY_OFF_DEFAULT;
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_RELAY1_DELAY_OFF_KEY, &m_delay_turn_off_relay1);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tRelay1 delay Off: %u", m_delay_turn_off_relay1);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found Relay1 delay Off!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }
        /* ====== RELAY2 DELAY OFF TIME ========== */
        m_delay_turn_off_relay2 = APP_FLASH_RELAY2_DELAY_OFF_DEFAULT;
        err = nvs_get_u16(my_nvs_handle, APP_FLASH_RELAY2_DELAY_OFF_KEY, &m_delay_turn_off_relay2);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tRelay2 delay Off: %u", m_delay_turn_off_relay2);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGD(TAG, "\tNot found Relay2 delay Off!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }

        // /* ====== Nguyên nhân tự reset ======= */
        // uint8_t self_reset_reason
        // err = nvs_get_u8(my_nvs_handle, APP_FLASH_SELF_RESET_KEY, &self_reset_reason());
        // switch (err)
        // {
        // case ESP_OK:
        //     ESP_LOGI(TAG, "\tSelfReset reason: %u", self_reset_reason);
        //     break;
        // case ESP_ERR_NVS_NOT_FOUND:
        //     ESP_LOGE(TAG, "\tNot found SelfReset!");
        //     break;
        // default:
        //     ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
        //     break;
        // }
        // Clear SelfReset reason -> Không clear nguyên nhân, khi nào gửi được lên server thì clear!
        //		err = nvs_set_u8(my_nvs_handle, APP_FLASH_SELF_RESET_KEY, 0);
        //		err = nvs_commit(my_nvs_handle);
        //		ESP_LOGI(TAG, "Clear SelfReset: %s", (err != ESP_OK) ? "Failed!" : "OK");

        /* ====== Mỗi lần khởi động là 1 lần reset -> update số lần reset ======= */
        m_total_reset_time = 0;
        err = nvs_get_u32(my_nvs_handle, APP_FLASH_RESET_COUNTER_KEY, &m_total_reset_time);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tReset counter: %u", m_total_reset_time);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "\tNot found Reset counter!");
            break;
        default:
            ESP_LOGE(TAG, "\rError (%s) reading!\n", esp_err_to_name(err));
            break;
        }
        // Tăng số lần reset và ghi lại
        m_total_reset_time += 1;
        app_flash_node_nvs_write_u32(APP_FLASH_RESET_COUNTER_KEY, m_total_reset_time);

        // Valid lại nội dung đọc được, nếu WiFi enable nhưng name, pass không đúng thì cũng bỏ qua
        bool isWifiNameValid = IsASCIIString(m_wifi_name);
        bool isWifiPassValid = IsASCIIString(m_wifi_pass);
        if (m_wifi_enable)
        {
            if (!isWifiNameValid || !isWifiPassValid)
            {
                ESP_LOGE(TAG, "ERR: WiFi name/password is not valid string!");
                m_wifi_enable = 0;
            }
        }

        // Valid GSM IMEI
        if (!IsDigitString(m_gsm_imei))
        {
            ESP_LOGE(TAG, "GSM_IMEI is invalid!");
            memset(m_gsm_imei, 0, sizeof(m_gsm_imei));
        }
        // Valid Master IMEI
        for (uint8_t i = MASTER_TINH1; i < MASTER_TOTAL; i++)
        {
            if (!IsDigitString(m_master_imei[i]))
            {
                ESP_LOGE(TAG, "MASTER%d is invalid!", i);
                memset(m_master_imei[i], 0, APP_FLASH_MASTER_IMEI_SIZE);
            }
        }

        // TCP console
        err = nvs_get_u8(my_nvs_handle, APP_FLASH_KEY_TCP_CONSOLE, &m_tcp_console_enable);
        switch (err)
        {
        case ESP_OK:
            ESP_LOGI(TAG, "\tTCP console: %u", m_tcp_console_enable);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            m_tcp_console_enable = 0;
            break;
        default:
            m_tcp_console_enable = 0;
            break;
        }


        // Valid các thông số cài đặt, khởi tạo mặc định nếu chưa có cài đặt
        if (m_operate_mode < APP_AUDIO_OPERATION_MODE_INTERNET || m_operate_mode > APP_AUDIO_OPERATION_MODE_NO_OPERATION)
            m_operate_mode = APP_AUDIO_OPERATION_MODE_INTERNET;

        if (m_current_master < MASTER_TINH1 || m_current_master > APP_FLASH_MASTER_XA3)
            m_current_master = APP_FLASH_MASTER_XA3; // Không có thì thu của master thấp nhất

        if (m_volume > 100)
            m_volume = APP_FLASH_VOLUME_DEFAULT;

        /* Đơn vị cấu hình tần số trên Slave là KHz, trên module FM là 10KHz */
        if (m_freq1 > 110000 || m_freq1 < 32000)
            m_freq1 = 100000; // 100MHz - VOV1
        if (m_freq2 > 110000 || m_freq2 < 32000)
            m_freq2 = 102700; // 102.7MHz - VOV2
        if (m_freq3 > 110000 || m_freq3 < 32000)
            m_freq3 = 96500; // 96.5MHz - VOV3
        if (m_current_freq > 110000 || m_current_freq < 32000)
            m_current_freq = 0;

        // Delay relay
        if (m_delay_turn_on_relay == 0 || m_delay_turn_on_relay > APP_FLASH_RELAY_DELAY_MAX_TIME)
            m_delay_turn_on_relay = APP_FLASH_RELAY_DELAY_ON_DEFAULT;
        if (m_delay_turn_off_relay1 == 0 || m_delay_turn_off_relay1 > APP_FLASH_RELAY_DELAY_MAX_TIME)
            m_delay_turn_off_relay1 = APP_FLASH_RELAY_DELAY_OFF_DEFAULT;
        if (m_delay_turn_off_relay2 == 0 || m_delay_turn_off_relay2 > APP_FLASH_RELAY2_DELAY_MAX_TIME)
            m_delay_turn_off_relay2 = APP_FLASH_RELAY2_DELAY_OFF_DEFAULT;

        // IO đọc ra 0xFF -> có thể lỗi
        if (app_io_get_io_value()->Value == 0xFF)
            app_io_get_io_value()->Value = 0;

        ESP_LOGI(TAG, "Current master: %u", m_current_master);
        ESP_LOGI(TAG, "Operation mode: %u", m_operate_mode);
        ESP_LOGI(TAG, "FM Freq: 1.%u 2.%u 3.%u, running: %u",
                 m_freq1, m_freq2, m_freq3, m_current_freq);
        ESP_LOGI(TAG, "Volume: %u", m_volume);
        ESP_LOGI(TAG, "Relay delay time: ON=%u, OFF1=%u, OFF2=%u",
                 m_delay_turn_on_relay, m_delay_turn_off_relay1, m_delay_turn_off_relay2);

        // Close nvs
        nvs_close(my_nvs_handle);
    }
}


esp_err_t app_flash_write_wifi_info(char *name, char *pass, uint8_t enable_wifi)
{
    nvs_handle my_nvs_handle;
    esp_err_t ret = ESP_OK;
    m_wifi_enable = enable_wifi;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_nvs_handle);
    ret |= err;

    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "\tWriting to NVS...");

        err = nvs_set_str(my_nvs_handle, "wifi_name", name);
        ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        ESP_LOGI(TAG, "\tCommitting updates wifi name in NVS ... ");
        err = nvs_commit(my_nvs_handle);
        ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Write password
        err = nvs_set_str(my_nvs_handle, "wifi_pass", pass);
        ESP_LOGI(TAG, "Set WiFi pass: %s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        ESP_LOGI(TAG, "\tCommitting updates wifi pass in NVS ... ");
        err = nvs_commit(my_nvs_handle);
        ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Write enable
        err = nvs_set_u8(my_nvs_handle, "wifi_enable", enable_wifi);
        ESP_LOGI(TAG, "Set WiFi enable: %s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        ESP_LOGI(TAG, "\tCommitting updates wifi enable in NVS ... ");
        err = nvs_commit(my_nvs_handle);
        ESP_LOGI(TAG, "%s", (err != ESP_OK) ? "Failed!" : "Done");
        ret |= err;

        // Close NVS
        nvs_close(my_nvs_handle);
    }

    return ret;
}

void app_flash_node_nvs_initialize(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // Retry nvs_flash_init
        ESP_LOGI(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Init NVS: %s", err == ESP_OK ? "OK" : "FAILED");
}

char *app_flash_get_wifi_name(void)
{
    return m_wifi_name;
}

char *app_flash_get_wifi_pass(void)
{
    return m_wifi_pass;
}

bool app_flash_is_wifi_enable(void)
{
    return m_wifi_enable ? true : false;
}

void app_flash_wifi_enable(bool enable)
{
    m_wifi_enable = (enable ? 1 : 0);
}

bool app_flash_is_tcp_console_enable(void)
{
    return m_tcp_console_enable ? true : false;
}

void app_flash_tcp_console_enable(void)
{
    if (!m_tcp_console_enable)
    {
        m_tcp_console_enable = 1;
        node_nvs_write_u8(APP_FLASH_KEY_TCP_CONSOLE, m_tcp_console_enable);
    }
}

void app_flash_tcp_console_disable(void)
{
    if (m_tcp_console_enable)
    {
        m_tcp_console_enable = 0;
        node_nvs_write_u8(APP_FLASH_KEY_TCP_CONSOLE, m_tcp_console_enable);
    }
}
