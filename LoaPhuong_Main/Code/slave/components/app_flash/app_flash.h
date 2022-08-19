#ifndef APP_FLASH_H
#define APP_FLASH_H

#include "stdint.h"
#include "stdbool.h"
#include "esp_err.h"


/*
 * Thứ tự ưu tiên của các master, nhỏ nhất = ưu tiên cao nhất
 */
enum
{
    MASTER_TINH1 = 1,
    MASTER_TINH2,
    MASTER_TINH3,
    APP_FLASH_MASTER_HUYEN1,
    APP_FLASH_MASTER_HUYEN2,
    APP_FLASH_MASTER_HUYEN3,
    APP_FLASH_MASTER_XA1,
    APP_FLASH_MASTER_XA2,
    APP_FLASH_MASTER_XA3,
    MASTER_TOTAL
} Master_Type_e;

#define APP_FLASH_KEY_MQTT_SERVER_URL "mqttSvr"
#define APP_FLASH_KEY_MQTT_SERVER_USERNAME "mqttUser" // "village-speaker"
#define APP_FLASH_KEY_MQTT_SERVER_PASSWORD "mqttPass"//"vs.bytech@2019"
#define APP_FLASH_KEY_LCD_ENABLE      "lcd_en"
#define APP_FLASH_RESET_COUNTER_KEY "reset_counter"

// NVS
#define APP_FLASH_AUDIO_IN_TYPE_KEY "audio_in"
#define APP_FLASH_GSM_IMEI_KEY "gsm_imei"
#define APP_FLASH_VOLUME_KEY "volume"
#define APP_FLASH_OPERATE_MODE_KEY "op_mode"
#define APP_FLASH_FM_FREQ1_KEY "fm_freq1"
#define APP_FLASH_FM_FREQ2_KEY "fm_freq2"
#define APP_FLASH_FM_FREQ3_KEY "fm_freq3"
#define APP_FLASH_IO_STATE_KEY "io_state"
#define APP_FLASH_RELAY_DELAY_ON_KEY "delay_on"
#define APP_FLASH_RELAY_DELAY_OFF_KEY "delay_off"
#define APP_FLASH_SELF_RESET_KEY "self_reset"
#define APP_FLASH_HTTP_URL_HEADER_KEY "url_header"
#define APP_FLASH_HTTP_STREAM_USERNAME_KEY    "stream_user"
#define APP_FLASH_HTTP_STREAM_PASSWORD_KEY    "stream_pass"
#define APP_FLASH_KEY_ALTERNATIVE_IMEI  "alt_imei"
#define APP_FLASH_IO_MAIN_GD32_PROTCOL_KEY "gd_p"
#define APP_FLASH_BROKER_URL "mqtt://village-speaker.bytechjsc.vn:2591"
#define APP_FLASH_MQTT_USERNAME "village-speaker"
#define APP_FLASH_MQTT_PASSWORD "vs.bytech@2019"
#define APP_FLASH_HTTP_STREAM_URL_SIZE    96
#define APP_FLASH_HTTP_STREAM_HEADER_SIZE 96
#define APP_FLASH_HTTP_STREAM_PASSWORD_SIZE 96
#define APP_HTTP_STREAM_USERNAME_SIZE 96
#define APP_FLASH_MQTT_URL_SIZE   96
#define APP_FLASH_MQTT_USERNAME_SIZE 96
#define APP_FLASH_MQTT_PASSWORD_SIZE 96
#define APP_FLASH_HTTP_STREAM_USERNAME_DEFAULT    "source"
#define APP_FLASH_HTTP_STREAM_PASSWORD_DEFAULT    "bytech"
#define APP_FLASH_WIFI_NAME_KEY "wifi_name"
#define APP_FLASH_WIFI_PASS_KEY "wifi_pass"
#define APP_FLASH_WIFI_ENABLE_KEY "wifi_enable"
#define APP_FLASH_KEY_TCP_CONSOLE   "tcp_console"
#define APP_FLASH_MASTER_T1_KEY "master1"
#define APP_FLASH_MASTER_H1_KEY "master2"
#define APP_FLASH_MASTER_X1_KEY "master3"
#define APP_FLASH_MASTER_T2_KEY "masterT2"
#define APP_FLASH_MASTER_H2_KEY "masterH2"
#define APP_FLASH_MASTER_X2_KEY "masterX2"
#define APP_FLASH_MASTER_T3_KEY "masterT3"
#define APP_FLASH_MASTER_H3_KEY "masterH3"
#define APP_FLASH_MASTER_X3_KEY "masterX3"
#define APP_FLASH_CURRENT_FREQ_KEY "cur_freq"
#define APP_FLASH_CURRENT_MASTER_KEY "cur_master"
#define APP_FLASH_SPK_DETECT "spk_detect"
#define APP_FLASH_RELAY1_DELAY_OFF_KEY "delay1_off"
#define APP_FLASH_RELAY2_DELAY_OFF_KEY "delay2_off"
#define APP_MQTT_KEEP_ALIVE_INTERVAL 120
#define APP_FLASH_HTTP_URL_HEADER_KEY "url_header"
#define APP_FLASH_HTTP_STREAM_HEADER "http://village-speaker.bytechjsc.vn:3000/"
//#define APP_FLASH_HTTP_STREAM_HEADER "http://stream.bytech.vn:3000/"
#define APP_FLASH_HTTP_STREAM_URL_TEST "http://village-speaker.bytechjsc.vn:3000/opustest4g1" // Opus link

// Các thông số thời gian delay RELAY
#define APP_FLASH_RELAY_DELAY_MAX_TIME 250
#define APP_FLASH_RELAY_DELAY_ON_DEFAULT 30  // Relay1 On -> 30s -> Relay2 On
#define APP_FLASH_RELAY_DELAY_OFF_DEFAULT 60 // Relay2 Off -> 60s -> Relay1 Off

#define APP_FLASH_RELAY2_DELAY_MAX_TIME 3600
#define APP_FLASH_RELAY2_DELAY_OFF_DEFAULT 60 // Stream finished -> Delay -> Relay2 Off (tổng time chờ tắt Relay2 = 1'30s)
#define APP_FLASH_VOLUME_DEFAULT 35

#define APP_FLASH_MAX_SUPPORT_MASTER        10
#define APP_FLASH_MASTER_IMEI_SIZE          25
/*
 * Reset system, nguyên nhân
 * 1: Lệnh REBOOT từ xa
 * 2: Config WiFi
 * 3: Timeout mất kn server MQTT
 * 4: OTA
 * 5: Quá lâu không start được stream
 * 6: Timeout chờ http_element stopped
 * 7: Main task hangup
 * 8: Lệnh set stream url mới
 * 100: TWDT times out
 * 101: ec2x_dce = null
 * 102: board_handle = null, PSRAM failed
 */
#define SW_RESET_REASON_REBOOT_CMD 1
#define SW_RESET_REASON_CONFIG_WIFI 2
#define SW_RESET_REASON_SRV_TIMEOUT 3
#define SW_RESET_REASON_OTA_FINISH 4
#define SW_RESET_REASON_STREAM_START_TIMEOUT 5
#define SW_RESET_REASON_HTTP_EL_STOP_TIMEOUT 6
#define SW_RESET_REASON_MAIN_TASK_HANG 7
#define SW_RESET_REASON_CHANGE_STREAM_URL 8
#define SW_RESET_REASON_TWDT_TIMEOUT 100
#define SW_RESET_REASON_DCE_NULL 101
#define SW_RESET_REASON_PSRAM_FAIL 102
#define SW_RESET_REASON_UNKNOWN 103

/**
 * @brief               Write string to nvs module
 * @param[in]           key Flash key
 * @param[in]           content String data
 */
esp_err_t app_flash_node_nvs_write_string(char *key, char *content);

/**
 * @brief               Write 32 bit value to nvs module
 * @param[in]           key Flash key
 * @param[in]           value 32 bit value
 */
esp_err_t app_flash_node_nvs_write_u32(char *key, uint32_t value);


/**
 * @brief               Write 1 byte in to nvs
 * @param[in]           key Flash key
 * @param[in]           value Value
 */
void app_flash_write_u8(char *key, uint8_t value);

/**
 * @brief               Write uint16 value to flash
 * @param[in]           key Flash key
 * @param[in]           value Value
 */
void app_flash_write_u16(char *key, uint16_t value);

/**
 * @brief               Write wifi info to flash
 * @param[in]           name wifi name
 * @param[in]           pass wifi password
 * @param[in]           enable_wifi Enable or disable wifi
 * @retval              esp_err_t
 */
esp_err_t app_flash_write_wifi_info(char *name, char *pass, uint8_t enable_wifi);

/**
 * @brief               Initialize nvs module
 */
void app_flash_node_nvs_initialize(void);

/**
 * @brief               Get operate mode
 * @retval              Operate mode
 */
uint8_t app_flash_get_operate_mode(void);

/**
 * @brief               Set operate mode
 * @retval              Operate mode
 */
void app_flash_set_operate_mode(uint8_t mode);

/**
 * @brief               Get GSM imei
 * @retval              GSM imei
 */
char *app_flash_get_imei(void);

/**
 * @brief               Set GSM imei
 * @param[in]           GSM imei
 */
void app_flash_set_imei(char *imei);

/**
 * @brief               Get Wifi name
 * @retval              Wifi name
 */
char *app_flash_get_wifi_name(void);

/**
 * @brief               Get Wifi pass
 * @retval              Wifi pass
 */
char *app_flash_get_wifi_pass(void);

/**
 * @brief               Get Wifi enable status
 * @retval              Wifi status
 */
bool app_flash_is_wifi_enable(void);

/**
 * @brief               Set Wifi enable status
 * @param[in]           Wifi status
 */
void app_flash_wifi_enable(bool enable);

/**
 * @brief               Get http stream header
 * @retval              http stream header
 */
char *app_flash_get_http_stream_header(void);

/**
 * @brief               Get server url
 * @retval              Server url 
 */
char *app_flash_get_mqtt_server_url(void);

/**
 * @brief               Get server username
 * @retval              Server username 
 */
char *app_flash_get_mqtt_server_username(void);

/**
 * @brief               Get server password
 * @retval              Server password 
 */
char *app_flash_get_mqtt_server_password(void);

/**
 * @brief               Read nvs parameters
 */
void app_flash_slave_nvs_read_params(char *paramName);

/**
 * @brief               App flash get total device reset times
 * @retval              Number of reset times
 */
uint32_t app_flash_get_total_reset_time(void);

/**
 * @brief               Get speaker detect enable status
 */
bool app_flash_speaker_detect_is_enable(void);

/**
 * @brief               Set app flash speaker detect enable status
 */
void app_flash_speaker_detect_set_enable(bool enable);

/**
 * @brief               Get current flash volume
 * @retval              Current flash volume
 */
uint8_t app_flash_get_volume(void);

/**
 * @brief               Set current flash volume
 * @retval              Current flash volume
 */
void app_flash_set_volume(uint8_t volume);

/**
 * @brief               Get current FM freq
 * @retval              freq Current FM freq
 */
uint32_t app_flash_get_current_fm_freq(void);

/**
 * @brief               Set current FM freq
 * @param[in]           freq Current FM freq
 */
void app_flash_set_current_fm_freq(uint32_t freq);

/**
 * @brief               Get FM FREQ1
 * @retval              FM freq
 */
uint32_t app_flash_get_fm_freq1(void);

/**
 * @brief               Get FM FREQ2
 * @retval              FM freq
 */
uint32_t app_flash_get_fm_freq2(void);

/**
 * @brief               Get FM FREQ3
 * @retval              FM freq
 */
uint32_t app_flash_get_fm_freq3(void);

/**
 * @brief               Set FM FMEQ1
 * @param[in]           FM freq
 */
void app_flash_set_fm_freq1(uint32_t freq);

/**
 * @brief               Set FM FMEQ2
 * @param[in]           FM freq
 */
void app_flash_set_fm_freq2(uint32_t freq);

/**
 * @brief               Set FM FMEQ3
 * @param[in]           FM freq
 */
void app_flash_set_fm_freq3(uint32_t freq);

/**
 * @brief               Get delay time to off relay 1
 * @retval              Delay time
 */
uint8_t app_flash_get_relay1_turn_off_delay(void);

/**
 * @brief               Get delay time to off relay 2
 * @retval              Delay time
 */
uint16_t app_flash_get_relay2_turn_off_delay(void);

/**
 * @brief               Set delay time to off relay 1
 * @param[in]           Delay time
 */
void app_flash_set_relay1_turn_off_delay(uint8_t delay);

/**
 * @brief               Set delay time to off relay 2
 * @param[in]           Delay time
 */
void app_flash_set_relay2_turn_off_delay(uint16_t delay);

/**
 * @brief               Get delay time to on relay
 * @retval              Delay time
 */
uint8_t app_flash_get_delay_turn_on_relay(void);

/**
 * @brief               Set delay time to on relay
 * @param[in]           Delay time
 */
void app_flash_set_delay_turn_on_relay(uint8_t delay);

/**
 * @brief               Get current streamming master
 * @retval              Current streamming master
 */
uint8_t app_flash_get_current_streamming_master_index(void);

/**
 * @brief               Set current streamming master
 * @param[in]           Current streamming master
 */
void app_flash_set_current_streamming_master(uint8_t master);

/**
 * @brief               Get master which saved in flash
 * @retval              Master imei
 */
char *app_flash_get_master(uint8_t index);

bool app_flash_is_tcp_console_enable(void);
void app_flash_tcp_console_enable(void);
void app_flash_tcp_console_disable(void);

#endif /* APP_FLASH_H */
