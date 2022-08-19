#ifndef APP_MQTT_H
#define APP_MQTT_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>


#define APP_MQTT_KEEP_ALIVE_INTERVAL 120
#define NETWORK_ERROR_TIMEOUT_SEC 180 // 180
/*
 * Master sub topic config: vs/sub/<Master IMEI>
 * Master pub topic command: vs/pub/<Master IMEI>
 * Master pub topic info: vs/pub/<Master IMEI>/info
 */
#define APP_MQTT_MASTER_PUB_TOPIC_HEADER "vs/pub/"
#define APP_MQTT_MASTER_SUB_TOPIC_HEADER "vs/sub/"

/*
 * Slave sub topic config: vs/sub/<Slave IMEI>
 * Slave sub topic master: vs/pub/<Master IMEI>
 * Slave pub topic: vs/pub/<Slave IMEI>
 */
#define SLAVE_PUB_TOPIC_HEADER "vs/pub/"
#define SLAVE_SUB_TOPIC_CONF_HEADER "vs/sub/" /* sub config from user */
#define SLAVE_SUB_TOPIC_CMD_HEADER "vs/pub/"  /* sub command from master */


typedef enum
{
    APP_MQTT_DISCONNECTED = 0,
    APP_MQTT_CONNECTING,
    APP_MQTT_CONNECTED
} app_mqtt_state_t;

typedef union 
{
	struct Stream_State_t 
    {
		uint16_t MasterTINH1 : 1;
		uint16_t MasterTINH2 : 1;
		uint16_t MasterTINH3 : 1;
		uint16_t MasterHUYEN1 : 1;
		uint16_t MasterHUYEN2 : 1;
		uint16_t MasterHUYEN3 : 1;
		uint16_t MasterXA1 : 1;
		uint16_t MasterXA2 : 1;
		uint16_t MasterXA3 : 1;
		uint16_t NA : 7;
	} __attribute__((packed)) Name;
	uint16_t Value;
} __attribute__((packed)) app_mqtt_master_streamming_state_t;


/**
 * @brief           Get server connection status
 * @retval          TRUE device is connected to server
 *                  FALSE device is not connected to server
 */
bool app_mqtt_is_connected_to_server(void);

static const char NET_IF_TAB[4][5] = 
{
    "NA",
    "WIFI",
    "ETH",
    "4G",
};

/**
 * @brief           Send mqtt message
 * @param[in]       Message header
 * @param[in]       msg Data
 * @retval          Message id, -1 on error
 */
int mqtt_publish_message(char *header, char *msg);

/**
 * @brief           Start mqtt task
 */
void app_mqtt_initialize(void);

/**
 * @brief           Subscribe topic
 * @param[in]       imei GSM imei
 */
void app_mqtt_send_subscribe_config_topic(char *imei);

/**
 * @brief           Subscribe command topic
 */
uint8_t app_mqtt_send_subscribe_command_topic(uint8_t masterID);

/**
 * @brief           Check if client is subscribed all topic
 * @retval          TRUE Client is subscribed
 *                  FALSE Client is not subscribed
 */
bool app_mqtt_is_subscribed(void);

/**
 * @brief           Start mqtt task
 * @retval          mqtt error code
 */
esp_err_t app_mqtt_start(void);

/**
 * @brief           Publish reset message
 */
void app_mqtt_send_reset_message(void);

/**
 * @brief           Get MQTT state
 * @retval          MQTT state
 */
app_mqtt_state_t app_mqtt_get_state(void);

/**
 * @brief           Set MQTT state
 * @param[in]       MQTT state
 */
void app_mqtt_set_state(app_mqtt_state_t state);

/**
 * @brief           Get HTTP waiting flag
 */
bool app_mqtt_is_waiting_http_element_stopped(void);

/**
 * @brief           Reset HTTP waiting flag
 */
void app_mqtt_is_reset_http_element_stopped_flag(void);

/**
 * @brief           Check if device is subscribed to a master
 */
bool app_mqtt_is_master_subscribed(uint8_t master_index);

/**
 * @brief           Publish slave info to server
 */
void app_mqtt_publish_slave_info(void);

/**
 * @brief       Check if at least 1 master is streamming
 * @retval      TRUE At least 1 master is streamming
 *              No master is streamming
 */
bool app_mqtt_is_master_streamming(void);

/**
 * @brief       Get master streamming status
 */
app_mqtt_master_streamming_state_t *app_mqtt_get_master_streamming_status(void);

#endif /* APP_MQTT_H */
