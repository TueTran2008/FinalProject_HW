#include "app_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "app_audio.h"
#include "network.h"
#include "driver/uart.h"
#include "app_flash.h"
#include "DataDefine.h"
#include "min.h"
#include "min_id.h"
#include "string.h"
#include "esp_sntp.h"
#include "app_mqtt.h"
#include "app_ota.h"
#include "esp_log.h"
#include "app_audio.h"
#include "utilities.h"
#include "pcf8575.h"
#include "main.h"
#include "esp_modem.h"
#include "esp_modem_dte.h"
#include "pcf8575.h"

// UART giao tiep mach FM hoac STM32
#define FM_UART_BAUD_RATE 115200
#define FM_UART_RX_BUF_SIZE 256
#define FM_UART_TX_BUF_SIZE 256
// Các chân FM UART
#define FM_UART_PORT_NUM UART_NUM_2
#define FM_UART_RX_IO GPIO_NUM_34 // Input only
#define FM_UART_TX_IO GPIO_NUM_13

static const char *TAG = "app_io";
static SemaphoreHandle_t m_sem_protect_gd32_uart;
static app_io_protocol_type_t m_gd32_protocol_state = APP_IO_MCU_PROTOCOL_STATE_UNKNOWN;
static app_io_protocol_type_t m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_UNKNOWN;
static uint8_t *m_min_gd32_rx_buffer;
static uint8_t *m_min_fm_rx_buffer;
static min_context_t m_min_fm_context;
static min_context_t m_min_gd32_context;
static min_frame_cfg_t m_min_fm_setting = MIN_DEFAULT_CONFIG();
static min_frame_cfg_t m_min_esp32_setting = MIN_DEFAULT_CONFIG();
static uint8_t m_protocol_select;
static app_io_i2c_expander_t IOExpander;
static app_io_mcu_expander_t STM32Expander;
static void on_gd32_frame_callback(void *context, min_msg_t *frame);
static bool gd32_uart_tx(void *ctx, uint8_t data);
static uint8_t m_hw_version = 3;
static void on_fm_frame_callback(void *context, min_msg_t *frame);
static uint8_t resend_mode_to_fm_module;
static app_io_struct_t m_io_control;	/* Điều khiển các IO vào ra */
static uint32_t m_is_in_test_mode;
static app_io_fm_ping_msg_t m_last_fm_msg;
static app_io_esp32_ping_msg_t m_last_gd_msg;
static void process_fm_uart_buffer(char *buffer, uint32_t len);
static uint32_t m_publish_fm_info = 0;
/* FM UART */
static QueueHandle_t fm_uart_queue;
// Receive buffer to collect incoming data
uint8_t fm_uart_rx_buf[FM_UART_RX_BUF_SIZE];
uint32_t fm_uart_rx_len;
static uint8_t m_fm_mode;
static char m_last_gps_info[32] = {"0.000000,0.000000"};
static uint32_t m_worker_version;
static uint32_t m_publish_gd32_info;


static app_io_speaker_detect_state_t m_speaker_detect_state = 
{
    .Value =  255,
};

uint8_t app_io_get_worker_version(void)
{
    return m_worker_version;
}

char *app_io_get_last_gps_info(void)
{
    return m_last_gps_info;
}


bool app_io_is_speaker_error(void)
{
    if (!app_flash_speaker_detect_is_enable())
    {
        return false;
    }
    uint32_t spk_err = ((m_speaker_detect_state.Value == 1)
                        || (m_speaker_detect_state.Value == 6)
                        || (m_speaker_detect_state.Value == 13)
                        || (m_speaker_detect_state.Value == 14));
    return spk_err;
}


app_io_fm_ping_msg_t *app_io_get_last_fm_msg(void)
{
    return &m_last_fm_msg;
}

app_io_esp32_ping_msg_t *app_io_get_last_gd32_msg(void)
{
    return &m_last_gd_msg;
}



void app_io_set_current_fm_module(uint8_t mode)
{
    m_fm_mode = mode;
}

uint8_t app_io_get_current_fm_module(void)
{
    return m_fm_mode;
}

uint32_t app_io_get_current_fm_freq(void)
{
    return app_flash_get_current_fm_freq();
}

void app_io_set_current_fm_freq(uint32_t freq)
{
    app_flash_set_current_fm_freq(freq);
}

static uint32_t m_freq_at_fm_slave;
uint32_t app_io_get_freq_at_slave(void)
{
    return m_freq_at_fm_slave;
}

void app_io_get_fm_snr_info(uint32_t *snr, uint32_t *rssi, uint32_t *dbm)
{
    *snr = m_last_fm_msg.snr;
    *rssi = m_last_fm_msg.rssi; 
    *dbm = m_last_fm_msg.dbm;
}

app_io_speaker_detect_state_t app_io_get_speaker_detect_value(void)
{
    return m_speaker_detect_state;
}

bool app_io_is_in_test_mode(void)
{
    return m_is_in_test_mode ? true : false;
}

app_io_struct_t *app_io_get_io_value(void)
{
    return &m_io_control;
}

void app_io_set_number_of_retries_counter(uint8_t counter)
{
    resend_mode_to_fm_module = counter;
}

app_io_i2c_expander_t *app_io_get_i2c_exp_value(void)
{
    return &IOExpander;
}

app_io_mcu_expander_t *app_io_get_mcu_exp_value(void)
{
    return &STM32Expander;
}

app_io_protocol_type_t app_io_get_gd32_protocol_method(void)
{
    return m_gd32_protocol_state;
}

app_io_protocol_type_t app_io_get_fm_protocol_method(void)
{
    return m_fm_protocol_state;
}


uint32_t app_io_get_pa_state(void)
{
    return IOExpander.BitName.EN_PA;
}

bool app_io_is_pa_off(void)
{
    if (m_hw_version == 2)
    {
        if (IOExpander.BitName.EN_PA != APP_IO_PA_ON)
            return true;
    }
    else if (m_hw_version == 3)
    {
        if (STM32Expander.BitName.EN_PA != APP_IO_STM_PA_ON)
            return true;
    }
    return false;
}

bool app_io_is_iso_relays_off(void)
{
    if (m_hw_version == 2)
    {
        if (IOExpander.BitName.ISO_OUT1 == APP_IO_OPTO_OFF || IOExpander.BitName.ISO_OUT2 == APP_IO_OPTO_OFF)
            return true;
    }
    else if (m_hw_version == 3)
    {
        if (STM32Expander.BitName.ISO_OUT1 == APP_IO_STM_ISO_OUT_OFF || STM32Expander.BitName.ISO_OUT2 == APP_IO_STM_ISO_OUT_OFF)
            return true;
    }
    return false;
}


bool app_io_is_led_4g_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE != APP_IO_IOEX_LED_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (STM32Expander.BitName.LED1_BLUE != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

// LED1 BLUE
void app_io_control_led_4g_state(uint8_t state)
{
    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_ON;

        // LED WiFi, ETH Off
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;

        // STM32
        STM32Expander.BitName.LED1_BLUE = APP_IO_STM_LED_ON;

        // LED WiFi, ETH Off
        STM32Expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
        STM32Expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;
        STM32Expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

bool app_io_is_led_wifi_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDAUX_RED != APP_IO_IOEX_LED_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (STM32Expander.BitName.LED1_RED != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

// LED2 RED
void app_io_control_led_eth_state(uint8_t state)
{
    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_ON;

        // LED PPP, WiFi Off
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;

        // STM32
        STM32Expander.BitName.LED2_RED = APP_IO_STM_LED_ON;

        // LED PPP, WiFi Off
        STM32Expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
        STM32Expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
        STM32Expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

// LED1 RED
void app_io_control_led_wifi_state(uint8_t state)
{
    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_ON;

        // LED ETH, PPP Off
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;

        // STM32
        STM32Expander.BitName.LED1_RED = APP_IO_STM_LED_ON;

        // LED ETH, PPP Off
        STM32Expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;
        STM32Expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
        STM32Expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

bool app_io_is_led_eth_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDMIC_RED != APP_IO_IOEX_LED_ON)
            return false;
    }
    else
    {
        if (STM32Expander.BitName.LED2_RED != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

void app_io_control_opto_output1(uint8_t state)
{
    if (state == APP_IO_OPTO_ON)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_ON;
        STM32Expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_ON;

        // Reset delay timeout relay1
        slave_reset_delay_turn_off_relay1_when_stop_stream();
        m_io_control.Name.IO1 = IO_ON;
    }
    else if (state == APP_IO_OPTO_OFF)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_OFF;
        STM32Expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_OFF;
        m_io_control.Name.IO1 = IO_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }

    // Lưu lại trạng thái output Opto
    app_flash_write_u8(APP_FLASH_IO_STATE_KEY, m_io_control.Value);
}

void app_io_control_opto_output2(uint8_t state)
{
    if (state == APP_IO_OPTO_ON)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_ON;
        STM32Expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_ON;

        // Reset delay timeout relay2
        slave_reset_delay_turn_off_relay2_when_stop_stream();
        m_io_control.Name.IO2 = IO_ON;
    }
    else if (state == APP_IO_OPTO_OFF)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_OFF;
        STM32Expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_OFF;
        m_io_control.Name.IO2 = IO_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }

    // Lưu lại trạng thái output Opto
    app_flash_write_u8(APP_FLASH_IO_STATE_KEY, m_io_control.Value);
}

void app_io_control_gsm_power_en(uint8_t ctrl)
{
    if (ctrl)
    {
        IOExpander.BitName.GSM_PWR_EN = 1;                        // Power ON Vbat +4V2
        STM32Expander.BitName.GSM_PWR_EN = APP_IO_STM_GSM_PWR_ON; // Power ON Vbat +4V2
    }
    else
    {
        IOExpander.BitName.GSM_PWR_EN = 0;                         // Power OFF Vbat +4V2
        STM32Expander.BitName.GSM_PWR_EN = APP_IO_STM_GSM_PWR_OFF; // Power OFF Vbat +4V2
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = IOExpander.Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

void app_io_control_gsm_pwr_key(uint8_t ctrl)
{
    if (ctrl)
    {
        app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 1;            // PowerKey LOW
        STM32Expander.BitName.GSM_PWR_KEY = APP_IO_STM_GSM_PWR_KEY_LOW; // PowerKey LOW
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 0;           // PowerKey HIGH
        STM32Expander.BitName.GSM_PWR_KEY = APP_IO_STM_GSM_PWR_KEY_HI; // PowerKey HIGH
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

void app_io_select_relay_audio_input(app_audio_line_in_type_t mode)
{
    ESP_LOGI(TAG, "Switch relay input to: %s", mode == APP_AUDIO_LINE_IN_AUX ? "AUX" : "MIC");

    if (mode == APP_AUDIO_LINE_IN_MIC)
    {
        app_io_get_i2c_exp_value()->BitName.SW_MIC_AUX = 1;   // Switch Relay to MIC input
        STM32Expander.BitName.SW_MIC_AUX = APP_IO_STM_MIC_ON; // Switch Relay to MIC input
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.SW_MIC_AUX = 0;   // Switch Relay to AUX input
        STM32Expander.BitName.SW_MIC_AUX = APP_IO_STM_AUX_ON; // Switch Relay to AUX input
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

void app_io_control_relay_audio_output(app_audio_operation_mode_t mode)
{
    ESP_LOGI(TAG, "Switch relay output to: %s", mode == APP_AUDIO_OPERATION_MODE_FM ? "FM" : "CODEC");

    if (mode == APP_AUDIO_OPERATION_MODE_FM)
    {
        app_io_get_i2c_exp_value()->BitName.SW_CODEC_FM = 1;  // Switch Relay to FM output
        STM32Expander.BitName.SW_CODEC_FM = APP_IO_STM_FM_ON; // Switch Relay to FM output
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.SW_CODEC_FM = 0;     // Switch Relay to codec output
        STM32Expander.BitName.SW_CODEC_FM = APP_IO_STM_CODEC_ON; // Switch Relay to codec output
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

/******************************************************************************************/
/**
 * @brief   : Khởi tạo UART cho module FM
 * @param   :
 * @retval  : ESP_OK or ESP_FAIL
 * @author  :
 * @created :
 */
uint32_t app_io_expander_uart_initialize(void)
{
    esp_err_t err;

    uart_config_t uart_config =
        {
            .baud_rate = FM_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    // Config UART with any gpio pins
    err = uart_param_config(FM_UART_PORT_NUM, &uart_config);
    ESP_LOGI(TAG, "uart_param_config: %s", err == ESP_OK ? "OK" : "ERR");

    err += uart_set_pin(FM_UART_PORT_NUM, FM_UART_TX_IO, FM_UART_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "uart_set_pin: %s", err == ESP_OK ? "OK" : "ERR");

    err += uart_driver_install(FM_UART_PORT_NUM, 2 * FM_UART_RX_BUF_SIZE, 2 * FM_UART_TX_BUF_SIZE, 3, &fm_uart_queue, 0);
    ESP_LOGI(TAG, "uart_driver_install: %s", err == ESP_OK ? "OK" : "ERR");

    return err;
}

static void on_gd32_frame_callback(void *context, min_msg_t *frame)
{
    ESP_LOGD(TAG, "On GD32 frame callback, id %u", frame->id);
    m_gd32_protocol_state = APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL;
    if (m_protocol_select != m_gd32_protocol_state)
    {
        app_flash_write_u8(APP_FLASH_IO_MAIN_GD32_PROTCOL_KEY, m_protocol_select);
    }
    switch (frame->id)
    {
    case MIN_ID_PING:
    {
        app_io_esp32_ping_msg_t *msg = (app_io_esp32_ping_msg_t *)frame->payload;
        memcpy(&m_last_gd_msg, msg, sizeof(app_io_esp32_ping_msg_t));

        static app_io_iso_input_t input;
        if (input.value != msg->iso_input.value)
        {
            input.value = msg->iso_input.value;
            ESP_LOGI(TAG, "Local vol %u, iso 0x%04X, vin %umV, test mode %u, version %d",
                    msg->local_vol, msg->iso_input.value, msg->vin, msg->test_mode, msg->fw_version);

            // Test: print trạng thái (theo bảng trạng thái)
            switch(input.value) 
            {
                case 7:
                case 15:
                    ESP_LOGI(TAG, "Older version, does not support: %d", input.value);
                    break;
                //Class AB
                case 12:
                    ESP_LOGI(TAG, "Class AB: Normal (%d)", input.value);
                    break;
                case 13:
                    ESP_LOGI(TAG, "Class AB: Short to GND (%d)", input.value);
                    break;
                case 14:
                    ESP_LOGI(TAG, "Class AB: Open circuit (%d)", input.value);
                    break;
                //Class D
                case 0:
                    ESP_LOGI(TAG, "Class D: Normal (%d)", input.value);
                    break;
                case 1:
                    ESP_LOGI(TAG, "Class D: Short to GND (%d)", input.value);
                    break;
                case 6:
                    ESP_LOGI(TAG, "Class D: Open circuit (%d)", input.value);
                    break;
                default:
                    ESP_LOGI(TAG, "Unknown input state: %d", input.value);
                    break;
            }

            if (app_io_get_i2c_exp_value()->BitName.EN_PA == APP_IO_PA_OFF 
                && STM32Expander.BitName.EN_PA == APP_IO_STM_PA_OFF)
            {
                m_speaker_detect_state.Value = input.value;
            }
        }
        app_io_stm32_update_io(&STM32Expander);

        if (m_publish_gd32_info == 1)
        {
            m_publish_gd32_info++;
            if (m_publish_gd32_info >= 10 && app_mqtt_is_connected_to_server())
            {
                m_publish_gd32_info = 0;
                char tmp[48];
                sprintf(tmp, "GD32 reset, version %u", m_worker_version);
                mqtt_publish_message("DBG", tmp);
            }
        }
    }
    break;
    case MIN_ID_FORWARD:
    {
        uint8_t *ptr = frame->payload;
        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
        {
            for (uint32_t i = 0; i < frame->len; i++)
            {
                min_rx_feed(&m_min_fm_context, ptr + i, 1);
            }
        }

        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
        {
            process_fm_uart_buffer((char *)frame->payload, frame->len);
        }
    }
    break;
    case MIN_ID_RESET:
        {
            if (m_publish_gd32_info == 0)
            {
                ESP_LOGE(TAG, "GD32 reset\r\n");
            }
            m_publish_gd32_info = 1;
        }
            break;

    default:
        app_ota_on_slave_frame_callback(frame);
        break;
    }
}

void app_io_initialize(void)
{
    if (!m_sem_protect_gd32_uart)
    {
        m_sem_protect_gd32_uart = xSemaphoreCreateMutex();
    }

    gpio_set_direction(APP_IO_MIC_DETECT_NUM, GPIO_MODE_DEF_INPUT);

    ESP_LOGI(TAG, "[GPIO] User button: %d, MIC detect: %d", gpio_get_level(APP_IO_BUTTON_ON_AIR_NUM),
             gpio_get_level(APP_IO_MIC_DETECT_NUM));

    /* ========================= End of Init 4G modem ===================== */
    /* =================== Init Peripheral ================================ */
    //Đọc logic chân button/IO36 trước xem có đang bấm ko (Input only IO không support pull-up/down -> cần trở pullup ngoài!
    gpio_set_direction(APP_IO_BUTTON_ON_AIR_NUM, GPIO_MODE_DEF_INPUT);
}

void app_io_stm32_update_io(app_io_mcu_expander_t *gpio)
{
    if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
    {
        char stmMessage[256] = {0};
        uint32_t index = 0;

        /* Bản tin phản hồi module FM
         * 4G,GPIO=<value>,CRC=12345#
         */
        index = sprintf(stmMessage, "WORKER,GPIO=%d,", gpio->Value);
        if (m_is_in_test_mode)
        {
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                index += sprintf(stmMessage + index, "%s", "SERVER(OK),");
            }
            else
            {
                index += sprintf(stmMessage + index, "%s", "SERVER(ERROR),");
            }

            index += sprintf(stmMessage + index, "GSM_IMEI(%s),", app_flash_get_imei());
            modem_dce_t *dce = slave_get_modem_dce();

            if (dce && strlen(dce->sim_imei) > 10)
            {
                index += sprintf(stmMessage + index, "SIM_IMEI(%s),", dce->sim_imei);
            }
            else
            {
                index += sprintf(stmMessage + index, "SIM_IMEI(%s),", "000");
            }

            if (network_is_eth_got_ip())
            {
                index += sprintf(stmMessage + index, "%s", "ETH(OK),");
            }
            else
            {
                index += sprintf(stmMessage + index, "%s", "ETH(ERROR),");
            }

            if (!app_audio_is_codec_error())
            {
                index += sprintf(stmMessage + index, "%s", "CODEC(OK),");
            }
            else
            {
                index += sprintf(stmMessage + index, "%s", "CODEC(ERROR),");
            }
        }

        /* CRC and tail */
        uint16_t crc16 = CalculateCRC16((uint8_t *)stmMessage, index);
        index += sprintf(&stmMessage[index], "CRC=%05u#", crc16);

        /* send to UART */
        xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
        uart_write_bytes(FM_UART_PORT_NUM, stmMessage, index);
        xSemaphoreGive(m_sem_protect_gd32_uart);
    }
    else
    {
        min_msg_t msg;
        msg.id = MIN_ID_CONTROL_GPIO;
        msg.len = sizeof(app_io_mcu_expander_t);
        msg.payload = gpio;
        xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
        min_send_frame(&m_min_gd32_context, &msg);
        xSemaphoreGive(m_sem_protect_gd32_uart);

        if (m_is_in_test_mode)
        {
            min_jig_data_t jig_data;
            jig_data.jig_status.name.gsm = 0;
            if (strlen(app_flash_get_imei()) > 10)
            {
                jig_data.jig_status.name.gsm = 1;
                sprintf((char*)jig_data.gsm_imei, "%s", app_flash_get_imei());
            }
            else
            {
                sprintf((char*)jig_data.gsm_imei, "%s", "000");
            }

            modem_dce_t *dce = slave_get_modem_dce();
            if (dce && strlen(dce->sim_imei) > 10)
            {
                sprintf((char*)jig_data.sim_imei, "%s", dce->sim_imei);
            }
            else
            {
                sprintf((char*)jig_data.sim_imei, "%s", "000");
            }

            jig_data.jig_status.name.server = (app_mqtt_get_state() == APP_MQTT_CONNECTED) ? 1 : 0;
            jig_data.jig_status.name.eth = network_is_eth_got_ip() ? 1 : 0;
            jig_data.jig_status.name.codec = app_audio_is_codec_error() ? 0 : 1;

            min_msg_t msg;
            msg.id = MIN_ID_JIG_DATA;
            msg.len = sizeof(min_jig_data_t);
            msg.payload = &jig_data;
            xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
            min_send_frame(&m_min_gd32_context, &msg);
            xSemaphoreGive(m_sem_protect_gd32_uart);
        }
    }
}

void app_io_init_min_protocol()
{
    m_min_gd32_rx_buffer = malloc(MIN_MAX_PAYLOAD);
    m_min_fm_rx_buffer = malloc(MIN_MAX_PAYLOAD);

    m_min_fm_setting.get_ms = xTaskGetTickCount;
    m_min_fm_setting.last_rx_time = 0x00;
    /* Not using the rx_callback then*/
    m_min_fm_setting.rx_callback = on_fm_frame_callback;
    m_min_fm_setting.timeout_not_seen_rx = 5000;
    m_min_fm_setting.tx_byte = NULL;
    m_min_fm_setting.use_timeout_method = 1;
    // m_min_fm_context.cb = &m_min_fm_setting;
    m_min_fm_context.rx_frame_payload_buf = m_min_fm_rx_buffer;
    m_min_fm_context.callback = &m_min_fm_setting;
    min_init_context(&m_min_fm_context);

    m_min_esp32_setting.get_ms = xTaskGetTickCount;
    m_min_esp32_setting.last_rx_time = 0x00;
    /* Not using the rx_callback then*/
    m_min_esp32_setting.rx_callback = on_gd32_frame_callback;
    m_min_esp32_setting.timeout_not_seen_rx = 5000;
    m_min_esp32_setting.tx_byte = gd32_uart_tx;
    m_min_esp32_setting.use_timeout_method = 1;
    // m_min_esp32context.cb = &m_min_esp32setting;
    m_min_gd32_context.rx_frame_payload_buf = m_min_gd32_rx_buffer;
    m_min_gd32_context.callback = &m_min_esp32_setting;
    min_init_context(&m_min_gd32_context);

    app_io_expander_uart_initialize();
    xTaskCreate(app_io_exp_uart_task, "fm_uart_task", 5 * 1024, NULL, 5, NULL);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void app_io_set_default_value(void)
{
    STM32Expander.Value = 0;

    STM32Expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    STM32Expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
    STM32Expander.BitName.LED2_BLUE = APP_IO_STM_LED_OFF;
    STM32Expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;

    /* Các chân ISO Ouput khởi tạo theo cấu hình cài đặt */
    /* Mặc định khởi tạo OFF, khi nào chạy stream hoặc MIC/LINE thì bật */
    STM32Expander.BitName.ISO_OUT1 = m_io_control.Name.IO1;
    STM32Expander.BitName.ISO_OUT2 = m_io_control.Name.IO2;

    // GSM OFF
    STM32Expander.BitName.GSM_PWR_EN = APP_IO_STM_GSM_PWR_OFF;
    STM32Expander.BitName.GSM_PWR_KEY = APP_IO_STM_GSM_PWR_KEY_HI;

    // Swith Audio input relay
    STM32Expander.BitName.SW_MIC_AUX = APP_IO_STM_AUX_ON; // Default to AUX input

    // Switch Ouput Codec - FM
    STM32Expander.BitName.SW_CODEC_FM = APP_IO_STM_CODEC_ON; // Relay to Codec ouput

    // PA enable
    STM32Expander.BitName.EN_PA = APP_IO_STM_PA_OFF;

    // Send lệnh qua UART
    app_io_stm32_update_io(&STM32Expander);
}

bool app_io_is_led_streamming_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE != APP_IO_IOEX_LED_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (STM32Expander.BitName.LED2_BLUE != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

// LED2 - BLUE
void app_io_control_led_stream(uint8_t state)
{
    if (state == APP_AUDIO_STREAM_RUNNING)
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_ON;
        STM32Expander.BitName.LED2_BLUE = APP_IO_STM_LED_ON;
    }
    else if (state == APP_AUDIO_STREAM_STOPPED)
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_OFF;
        STM32Expander.BitName.LED2_BLUE = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

/*
 * Điều khiển mạch PA ngoài
 * 1: ON, 0: OFF
 */
void app_io_control_pa(uint8_t state)
{
    bool spk_err = app_io_is_speaker_error();

    if (state && spk_err)
    {
        // Khi ho mach hoac ngan mach thi ko bat PA
        app_io_get_i2c_exp_value()->BitName.EN_PA = APP_IO_PA_OFF;
        STM32Expander.BitName.EN_PA = APP_IO_STM_PA_OFF;
        return;
    }

    ESP_LOGI(TAG, "Turn PA_EN: %s", state ? "ON" : "OFF");

    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.EN_PA = APP_IO_PA_ON;
        STM32Expander.BitName.EN_PA = APP_IO_STM_PA_ON;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.EN_PA = APP_IO_PA_OFF;
        STM32Expander.BitName.EN_PA = APP_IO_STM_PA_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }
}

static bool m_fm_allowed = false;
void app_io_allow_process_fm_data(void)
{
    m_fm_allowed = true;
}

static void process_fm_uart_buffer(char *buffer, uint32_t size)
{
    /** 
     * Xử lý bản tin FM
     * FM|LOCAL_MODE=<FM/MIC/LINE/IDLE>|FREQ=<FREQ>|VOL=<70>|SNR=<>|RSSI=<>|GPS=<VD,KD>|CRC=12345#
     */
    char pResponse[128];
    uint8_t index = 0;
    if (m_hw_version == 3)
    {
        // Chưa khởi tạo xong audio codec thì không nhận lệnh chuyển mode!
        if (!m_fm_allowed)
            return;
    }

    if (strstr((char *)buffer, "FM|") && strstr((char *)buffer, "#") && size > 10)
    {
        /* Check CRC: |CRC=12345# */
        char *crc = strstr((char *)buffer, "CRC=");
        if (crc)
        {
            uint16_t crc16 = GetNumberFromString(4, crc);

            /* Tinh CRC thuc cua chuoi: Tru 10 ki tu cuoi CRC=12345# */
            uint16_t crcCal = CalculateCRC16((uint8_t *)buffer, size - 10);

            if (crc16 != crcCal)
            {
                ESP_LOGI(TAG, "FM CRC failed: %u - %u", crc16, crcCal);
                return;
            }
        }
        else
        {
            return;
        }

        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
        {
            m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT;
        }
        
        /** Chỉ xét các chế độ đang chạy dưới module FM khi slave đang rảnh (chế độ INTERNET và không streaming) */
        if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
            && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !app_mqtt_get_master_streamming_status()->Value))
        {
            if (strstr((char *)buffer, "MODE=LOCAL_FM"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_FM)
                {
                    ESP_LOGI(TAG, "Switch to Local FM mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_FM;

                    /* Change mode to FM */
                    app_audio_change_to_local_fm_mode();

                    // Tắt Relay (nếu được bật ở chế độ LINE IN)
                    ESP_LOGI(TAG, "call app_io_opto_control_all...");
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to Local FM mode");
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_MIC"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_MIC)
                {
                    ESP_LOGI(TAG, "Switch to Local MIC mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_MIC;

                    /* Change mode to MIC */
                    app_audio_change_to_local_mic_mode();

                    // Bật Relay cho chế độ MIC
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to Local MIC mode");
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_LINE"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_LINE)
                {
                    ESP_LOGI(TAG, "Switch to Local LINE mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_LINE;

                    /* Change mode to LINE */
                    app_audio_change_to_local_line_in_mode();

                    // Bật Relay cho chế độ LINE IN
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
            }
        }

          /* Volume */
        char *sVolume = strstr((char *)buffer, "VOL=");
        if (sVolume)
        {
            uint8_t vol = GetNumberFromString(4, sVolume);

            /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
            if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE) &&
                app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
            {
                if (abs(app_audio_get_current_output_vol() - vol) > 3)
                {
                    ESP_LOGI(TAG, "Change codec volume: %u -> %u", app_audio_get_current_output_vol(), vol);
                    app_audio_change_codec_vol(vol);
                }
            }
        }

        /* Clear trạng thái FM module trước đó */
        if (strstr((char *)buffer, "MODE=LOCAL_IDLE"))
        {
            /* Nếu trước đó đang thu FM hoặc chạy từ MIC/LINE IN -> tắt codec, ngắt Relay và PA */
            if (m_fm_mode == APP_AUDIO_OPERATION_MODE_FM 
                || m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC
                ||m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
            {
                // Chuyển codec về mode DECODE
                app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                // Switch Relay to Codec output
                app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                /* Nếu PA đang ON thì OFF */
                if (!app_io_is_pa_off())
                {
                    app_io_control_pa(APP_IO_PA_OFF);
                }

                // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                ESP_LOGI(TAG, "call app_io_opto_control_all...");
                app_io_opto_control_all(APP_IO_OPTO_OFF);
            }
            m_fm_mode = APP_AUDIO_OPERATION_MODE_IDLE;
        }

        /* Các trạng thái FM */
        char *freq = strstr((char *)buffer, "FREQ=");
        if (freq)
        {
            m_freq_at_fm_slave = GetNumberFromString(5, freq);
        }

        /* Tọa độ GPS: GPS=<LAT>,<LNG>| */
        char *gps = strstr((char *)buffer, "GPS=");
        if (gps)
        {
            memset(m_last_gps_info, 0, sizeof(m_last_gps_info));
            CopyParameter(gps, m_last_gps_info, '=', '|');
        }

        /* Bản tin phản hồi module FM
         * 4G,MODE=<FM/4G/MIC/NONE/IDLE>,FREQ_RUN=<10270>,FREQ1=<>,FREQ2=<>,FREQ3=<>,VOL=<70>,THRESHOLD=<90>,CRC=12345#
         */
        index = sprintf(pResponse, "%s", "4G,MODE=");

        /* Chế độ/ trạng thái hoạt động */
        switch (app_flash_get_operate_mode())
        {
        case APP_AUDIO_OPERATION_MODE_INTERNET:
            /* Nếu đang không stream internet thì là ở trạng thái IDLE */
            if (!app_audio_is_http_audio_stream_running() &&
                (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || app_mqtt_get_master_streamming_status()->Value == 0 ||
                 slave_get_http_received_data_timeout() == 0)) /* Quá lâu không nhận được gói tin http */
            {
                if (resend_mode_to_fm_module)
                {
                    index += sprintf(&pResponse[index], "INTERNET,");
                }
                else
                {
                    index += sprintf(&pResponse[index], "IDLE,");
                }
            }
            else
            {
                index += sprintf(&pResponse[index], "INTERNET,");
            }
            break;

        case APP_AUDIO_OPERATION_MODE_FM:
            index += sprintf(&pResponse[index], "FM,FREQ_RUN=%u,", app_flash_get_current_fm_freq() / 10);
            break;
        case APP_AUDIO_OPERATION_MODE_MIC:
            index += sprintf(&pResponse[index], "MIC,");
            break;

        case APP_AUDIO_OPERATION_MODE_NO_OPERATION:
            index += sprintf(&pResponse[index], "NONE,");
            break;
        case APP_AUDIO_OPERATION_MODE_IDLE:
            index += sprintf(&pResponse[index], "IDLE,");
            break;
            
        default:
            break;
        }

        if (app_mqtt_is_connected_to_server())
        {
            index += sprintf(&pResponse[index], "%s%s,", "CONNECTED,", NET_IF_TAB[network_get_current_interface()]);
        }
        else
        {
            index += sprintf(&pResponse[index],  "%s", "DISCONNECTED,");
        }

        /* Volume */
        index += sprintf(&pResponse[index], "VOL=%u,", app_flash_get_volume());

        /* Tần số cấu hình Freq1,2,3 : Đơn vị dưới module FM là 10KHz -> /10 */
        index += sprintf(&pResponse[index], "FREQ1=%u,FREQ2=%u,FREQ3=%u,", app_flash_get_fm_freq1() / 10,
                         app_flash_get_fm_freq2() / 10, app_flash_get_fm_freq3() / 10);

        /* CRC and tail */
        uint16_t crc16 = CalculateCRC16((uint8_t *)pResponse, index);
        index += sprintf(&pResponse[index], "CRC=%05u#", crc16);
        /* send to UART */
        if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
        {
            xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
            uart_write_bytes(FM_UART_PORT_NUM, pResponse, index);
            xSemaphoreGive(m_sem_protect_gd32_uart);
        }
        else
        {
            min_msg_t msg;
            msg.id = MIN_ID_FORWARD;
            msg.len = index;
            msg.payload = pResponse;
            xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
            min_send_frame(&m_min_gd32_context, &msg);
            xSemaphoreGive(m_sem_protect_gd32_uart);
        }
    }
}

static void process_gd32_new_data(char *buffer, uint32_t len)
{
    char pResponse[128] = {0};
    uint16_t index = 0;

    if (m_hw_version == 3)
    {
        // Chưa khởi tạo xong audio codec thì không nhận lệnh chuyển mode!
        if (!m_fm_allowed)
            return;
    }

    /** Xử lý bản tin FM
     * FM|LOCAL_MODE=<FM/MIC/LINE/IDLE>|FREQ=<FREQ>|VOL=<70>|SNR=<>|RSSI=<>|GPS=<VD,KD>|CRC=12345#
     */
    ESP_LOGD(TAG, "%s\r\n", (char *)buffer);
    if (strstr((char *)buffer, "FM|") && strstr((char *)buffer, "#") && len > 10)
    {
        /* Check CRC: |CRC=12345# */
        char *crc = strstr((char *)buffer, "CRC=");
        if (crc)
        {
            uint16_t crc16 = GetNumberFromString(4, crc);

            /* Tinh CRC thuc cua chuoi: Tru 10 ki tu cuoi CRC=12345# */
            uint16_t crcCal = CalculateCRC16((uint8_t *)buffer, len - 10);

            if (crc16 != crcCal)
            {
                ESP_LOGI(TAG, "FM CRC failed: %u - %u", crc16, crcCal);
                return;
            }
        }
        else
        {
            return;
        }

        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
        {
            m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT;
        }

        /** Chỉ xét các chế độ đang chạy dưới module FM khi slave đang rảnh (chế độ INTERNET và không streaming) */
        if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
            && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !app_mqtt_get_master_streamming_status()->Value))
        {
            if (strstr((char *)buffer, "MODE=LOCAL_FM"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_FM)
                {
                    ESP_LOGI(TAG, "Switch to Local FM mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_FM;

                    /* Change mode to FM */
                    app_audio_change_to_local_fm_mode();

                    // Tắt Relay (nếu được bật ở chế độ LINE IN)
                    ESP_LOGI(TAG, "call app_io_opto_control_all...");
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to Local FM mode");
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_MIC"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_MIC)
                {
                    ESP_LOGI(TAG, "Switch to Local MIC mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_MIC;

                    /* Change mode to MIC */
                    app_audio_change_to_local_mic_mode();

                    // Bật Relay cho chế độ MIC
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to Local MIC mode");
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_LINE"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_LINE)
                {
                    ESP_LOGI(TAG, "Switch to Local LINE mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_LINE;

                    /* Change mode to LINE */
                    app_audio_change_to_local_line_in_mode();

                    // Bật Relay cho chế độ LINE IN
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
            }
        }

        /* Volume */
        char *sVolume = strstr((char *)buffer, "VOL=");
        if (sVolume)
        {
            uint8_t vol = GetNumberFromString(4, sVolume);

            /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
            if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
                 && app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
            {
                if (abs(app_audio_get_current_output_vol() - vol) > 3)
                {
                    ESP_LOGI(TAG, "Change codec volume: %u -> %u", app_audio_get_current_output_vol(), vol);

                    app_audio_change_codec_vol(vol);
                }
            }
        }

        /* Clear trạng thái FM module trước đó */
        if (strstr((char *)buffer, "MODE=LOCAL_IDLE"))
        {
            /* Nếu trước đó đang thu FM hoặc chạy từ MIC/LINE IN -> tắt codec, ngắt Relay và PA */
            if (m_fm_mode == APP_AUDIO_OPERATION_MODE_FM 
                || m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC
                 ||m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
            {
                // Chuyển codec về mode DECODE
                app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                // Switch Relay to Codec output
                app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                /* Nếu PA đang ON thì OFF */
                if (!app_io_is_pa_off())
                {
                    app_io_control_pa(APP_IO_PA_OFF);
                }

                // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                ESP_LOGI(TAG, "call app_io_opto_control_all...");
                app_io_opto_control_all(APP_IO_OPTO_OFF);
            }
            m_fm_mode = APP_AUDIO_OPERATION_MODE_IDLE;
        }

        /* Các trạng thái FM */
        char *freq = strstr((char *)buffer, "FREQ=");
        if (freq)
        {
            m_freq_at_fm_slave = GetNumberFromString(5, freq);
        }
        
        char *SNR = strstr((char *)buffer, "SNR=");
        if (SNR)
        {
            m_last_fm_msg.snr = GetNumberFromString(4, SNR);
        }
        char *RSSI = strstr((char *)buffer, "RSSI=");
        if (RSSI)
        {
            m_last_fm_msg.rssi = GetNumberFromString(5, RSSI);
        }
        char *dBm = strstr((char *)buffer, "dBm=");
        if (dBm)
        {
            m_last_fm_msg.dbm = GetNumberFromString(4, dBm);
        }

        /* Tọa độ GPS: GPS=<LAT>,<LNG>| */
        char *gps = strstr((char *)buffer, "GPS=");
        if (gps)
        {
            memset(m_last_gps_info, 0, sizeof(m_last_gps_info));
            CopyParameter(gps, m_last_gps_info, '=', '|');
        }

        /* Bản tin phản hồi module FM
         * 4G,MODE=<FM/4G/MIC/NONE/IDLE>,FREQ_RUN=<10270>,FREQ1=<>,FREQ2=<>,FREQ3=<>,VOL=<70>,THRESHOLD=<90>,CRC=12345#
         */
        index = sprintf(pResponse, "4G,MODE=");

        /* Chế độ/ trạng thái hoạt động */
        switch (app_flash_get_operate_mode())
        {
        case APP_AUDIO_OPERATION_MODE_INTERNET:
            /* Nếu đang không stream internet thì là ở trạng thái IDLE */
            // audio_element_state_t el_i2s_state = audio_element_get_state(i2s_stream_writer);
            // audio_element_state_t el_opus_state = audio_element_get_state(opus_decoder);
            if (!app_audio_is_http_audio_stream_running()
                 && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || app_mqtt_get_master_streamming_status()->Value == 0 ||
                 slave_get_http_received_data_timeout() == 0)) /* Quá lâu không nhận được gói tin http */
            {
                if (resend_mode_to_fm_module)
                {
                    index += sprintf(&pResponse[index], "INTERNET,");
                }
                else
                {
                    index += sprintf(&pResponse[index], "IDLE,");
                }
            }
            else
            {
                index += sprintf(&pResponse[index], "INTERNET,");
            }

            break;
        case APP_AUDIO_OPERATION_MODE_FM:
            index += sprintf(&pResponse[index], "FM,FREQ_RUN=%u,", app_flash_get_current_fm_freq() / 10);
            break;
        case APP_AUDIO_OPERATION_MODE_MIC:
            index += sprintf(&pResponse[index], "MIC,");
            break;
        case APP_AUDIO_OPERATION_MODE_NO_OPERATION:
            index += sprintf(&pResponse[index], "NONE,");
            break;
        case APP_AUDIO_OPERATION_MODE_IDLE:
            index += sprintf(&pResponse[index], "IDLE,");
            break;
        default:
            break;
        }

        /* Tần số cấu hình Freq1,2,3 : Đơn vị dưới module FM là 10KHz -> /10 */
        index += sprintf(&pResponse[index], "FREQ1=%u,FREQ2=%u,FREQ3=%u,", app_flash_get_fm_freq1() / 10,
                         app_flash_get_fm_freq2() / 10, app_flash_get_fm_freq3() / 10);

        /* Volume */
        index += sprintf(&pResponse[index], "VOL=%u,", app_flash_get_volume());

        /* CRC and tail */
        uint16_t crc16 = CalculateCRC16((uint8_t *)pResponse, index);
        index += sprintf(&pResponse[index], "CRC=%05u#", crc16);

        if (resend_mode_to_fm_module)
            resend_mode_to_fm_module--;
        xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
        /* send to UART */
        uart_write_bytes(FM_UART_PORT_NUM, pResponse, index);
        xSemaphoreGive(m_sem_protect_gd32_uart);
    }
    /* Xử lý bản tin từ IO_EXT MCU STM32/GD32 */
    else if (strstr((char *)buffer, "WORKER|") && strstr((char *)buffer, "#") && len > 10)
    {
        /* Check CRC: |CRC=12345# */
        char *crc = strstr((char *)buffer, "CRC=");
        if (crc)
        {
            uint16_t crc16 = GetNumberFromString(4, crc);

            /* Tinh CRC thuc cua chuoi: Tru 10 ki tu cuoi CRC=12345# */
            uint16_t crcCal = CalculateCRC16((uint8_t *)buffer, len - 10);

            if (crc16 != crcCal)
            {
                ESP_LOGI(TAG, "GD32 CRC failed: %u - %u", crc16, crcCal);
                return;
            }
        }
        else
        {
            return;
        }

        // Trạng thái input detect speaker: chỉ detect đúng khi EN_PA đang off
        char *isoInput = strstr((char *)buffer, "ISOINPUT=");
        if (isoInput != NULL)
        {
            uint8_t spkInput = GetNumberFromString(9, isoInput);

            // Test: print trạng thái (theo bảng trạng thái)
            /*switch(spkInput) {
                case 7:
                case 15:
                    ESP_LOGI(TAG, "Older version, does not support: %d", spkInput);
                    break;
                //Class AB
                case 12:
                    ESP_LOGI(TAG, "Class AB: Normal (%d)", spkInput);
                    break;
                case 13:
                    ESP_LOGI(TAG, "Class AB: Short to GND (%d)", spkInput);
                    break;
                case 14:
                    ESP_LOGI(TAG, "Class AB: Open circuit (%d)", spkInput);
                    break;
                //Class D
                case 0:
                    ESP_LOGI(TAG, "Class D: Normal (%d)", spkInput);
                    break;
                case 1:
                    ESP_LOGI(TAG, "Class D: Short to GND (%d)", spkInput);
                    break;
                case 6:
                    ESP_LOGI(TAG, "Class D: Open circuit (%d)", spkInput);
                    break;
                default:
                    ESP_LOGI(TAG, "Unknow input state: %d", spkInput);
                    break;
            } */

            if (IOExpander.BitName.EN_PA == APP_IO_PA_OFF && STM32Expander.BitName.EN_PA == APP_IO_STM_PA_OFF)
                m_speaker_detect_state.Value = spkInput;
            // else
            // ESP_LOGI(TAG, "EN_PA is ON, don't detect speaker state: %d", spkInput);
        }
        else
        {
            m_speaker_detect_state.Value = 255;
        }

        if (!m_is_in_test_mode)
        {
            if (strstr((char *)buffer, "TESTMODE=1|"))
            {
                slave_set_audio_switch_audio_in_test_mode();
                m_is_in_test_mode = 1;
                sprintf(app_audio_get_stream_url(), "%s%s", app_flash_get_http_stream_header(), "863674040993444");
                // Auto enable speaker_detect mode
                app_flash_write_u8(APP_FLASH_SPK_DETECT, 1);
                app_flash_speaker_detect_set_enable(1);
                app_audio_test_mode_change_codec_vol(100);
            }
        }
        // Version mạch MCU
        char *versionWorker = strstr((char *)buffer, "VERSION=");
        if (versionWorker != NULL)
        {
            m_worker_version = GetNumberFromString(8, versionWorker);
        }
        else
        {
            m_worker_version = 0;
        }
        m_gd32_protocol_state = APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT;
        // Phản hồi lệnh điều khiển GPIO nếu giá trị gpio dưới STM32 khác ESP32
        app_io_stm32_update_io(&STM32Expander);
    }
}

void app_io_exp_uart_task(void *arg)
{
    uart_event_t event;

    ESP_LOGI(TAG, "\t\t--- app_io_exp_uart_task is running ---");

    while (1)
    {
        // Loop will continually block (i.e. wait) on event messages from the event queue
        if (xQueueReceive(fm_uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
        {
            // Handle received event
            if (event.type == UART_DATA)
            {
                fm_uart_rx_len = 0;
                memset(fm_uart_rx_buf, 0, FM_UART_RX_BUF_SIZE);

                uint16_t uart_received_length = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(FM_UART_PORT_NUM, (size_t *)&uart_received_length));
                fm_uart_rx_len = uart_read_bytes(FM_UART_PORT_NUM, fm_uart_rx_buf, uart_received_length, 100);

                /* Process data */
                if (fm_uart_rx_len > 0)
                {
                    if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
                    {
                        // ESP_LOGW(TAG, "Feed1 %u bytes\r\n", fm_uart_rx_len);
                        for (uint32_t i = 0; i < fm_uart_rx_len; i++)
                        {
                            min_rx_feed(&m_min_gd32_context, fm_uart_rx_buf + i, 1);
                        }
                    }

                    if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
                    {
                        process_gd32_new_data((char*)fm_uart_rx_buf, fm_uart_rx_len);
                        if (m_gd32_protocol_state == APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT && m_min_gd32_rx_buffer)
                        {
                            ESP_LOGW(TAG, "GD32 main used string protocol");
                            free(m_min_gd32_rx_buffer);
                            m_min_gd32_rx_buffer = NULL;
                        }
                    }
                    else
                    {
                        min_timeout_poll(&m_min_gd32_context);
                    }
                }
            }
            // Handle frame error event
            else if (event.type == UART_FRAME_ERR)
            {
                // TODO...
            }
            // Final else statement to act as a default case
            else
            {
                // TODO...
            }
        }
        if (m_gd32_protocol_state == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
        {
            min_timeout_poll(&m_min_gd32_context);
        }

        if (m_fm_protocol_state == APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT && m_min_fm_rx_buffer)
        {
            ESP_LOGW(TAG, "FM module used string protocol\r\n");
            free(m_min_fm_rx_buffer);
            m_min_fm_rx_buffer = NULL;
        }
    }

    ESP_LOGI(TAG, "\t\t--- app_io_exp_uart_task is stopped ---");
    vTaskDelete(NULL);
}

uint8_t app_io_get_hardware_version(void)
{
    return m_hw_version;
}

void app_io_set_hardware_version(uint8_t version)
{
    m_hw_version = version;
}

bool app_io_is_mic_plugged(void)
{
    return !gpio_get_level(APP_IO_MIC_DETECT_NUM);
}

/**
 * Điều khiển ON/OFF cả 2 OptoOutput cho relay ngoài
 */
void app_io_opto_control_all(uint8_t state)
{
    if (state == APP_IO_OPTO_ON)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_ON;
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_ON;

        // STM32
        STM32Expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_ON;
        STM32Expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_ON;

        m_io_control.Name.IO1 = IO_ON;
        m_io_control.Name.IO2 = IO_ON;

        // Reset delay timeout all
       slave_reset_delay_turn_off_relay1_when_stop_stream();
        slave_reset_delay_turn_off_relay2_when_stop_stream();
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_OFF;
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_OFF;

        // STM32
        STM32Expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_OFF;
        STM32Expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_OFF;

        m_io_control.Name.IO1 = IO_OFF;
        m_io_control.Name.IO2 = IO_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_stm32_update_io(&STM32Expander);
    }

    // Lưu lại trạng thái output Opto
    app_flash_write_u8(APP_FLASH_IO_STATE_KEY, m_io_control.Value);
}


bool app_io_is_switch_codec_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.SW_CODEC_FM != APP_IO_OUT_CODEC_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (STM32Expander.BitName.SW_CODEC_FM != APP_IO_STM_CODEC_ON)
            return false;
    }
    return true;
}

static void on_fm_frame_callback(void *context, min_msg_t *frame)
{
    m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL;
    ESP_LOGD(TAG, "ON FM frame callback id %d", frame->id);
    switch (frame->id)
    {
        case MIN_ID_PING:
        case MIN_ID_FORWARD:
        {
            app_io_fm_ping_msg_t *msg = (app_io_fm_ping_msg_t*)frame->payload;
            memcpy(&m_last_fm_msg, msg, sizeof(app_io_fm_ping_msg_t));
            uint8_t mode = APP_AUDIO_OPERATION_MODE_NO_OPERATION;
            uint32_t freq = 0xFFFFFFFF;
            if (app_audio_is_http_audio_stream_running())
            {
                mode = APP_AUDIO_OPERATION_MODE_INTERNET;
            }
            else
            {
                mode = app_flash_get_operate_mode();
                freq = app_flash_get_current_fm_freq() / 10;
            }

            m_worker_version = m_last_fm_msg.fw_version;
            /** Chỉ xét các chế độ đang chạy dưới module FM khi slave đang rảnh (chế độ INTERNET và không streaming) */
            if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
                && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !app_mqtt_get_master_streamming_status()->Value))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_FM
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_FM)
                {
                    ESP_LOGI(TAG, "Switch to Local FM mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_FM;

                    /* Change mode to FM */
                    app_audio_change_to_local_fm_mode();
                    mqtt_publish_message("DBG", "Switch to Local FM mode");
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }

                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_MIC
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_MIC)
                {
                    ESP_LOGI(TAG, "Switch to Local MIC mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_MIC;

                    /* Change mode to MIC */
                    app_audio_change_to_local_mic_mode();
                    mqtt_publish_message("DBG", "switch to Local MIC mode");
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }

                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_LINE
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_LINE)
                    {
                        ESP_LOGI(TAG, "Switch to Local LINE mode");

                        m_fm_mode = APP_AUDIO_OPERATION_MODE_LINE;
                        
                        /* Change mode to LINE */
                        app_audio_change_to_local_line_in_mode();
                        // Bật Relay cho chế độ LINE IN
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                        mqtt_publish_message("DBG", "switch to Local LINE mode");
                    }

                /* Clear trạng thái FM module trước đó khi chuyển về trạng thái IDLE */
                if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_FM 
                    || m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC
                    ||m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_IDLE)
                {
                    ESP_LOGI(TAG, "Switch to IDLE mode");
                    // Chuyển codec về mode DECODE
                    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                    // Switch Relay to Codec output
                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                    /* Nếu PA đang ON thì OFF */
                    if (!app_io_is_pa_off())
                    {
                        app_io_control_pa(APP_IO_PA_OFF);
                    }

                    // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                    ESP_LOGI(TAG, "call app_io_opto_control_all...");
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                    m_fm_mode = APP_AUDIO_OPERATION_MODE_IDLE;
                }

                /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
                if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE) &&
                    app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
                {
                    if (abs(app_audio_get_current_output_vol() - m_last_fm_msg.vol) > 3)
                    {
                        ESP_LOGI(TAG, "Change codec volume: %u -> %u", app_audio_get_current_output_vol(), m_last_fm_msg.vol);
                        app_audio_change_codec_vol(m_last_fm_msg.vol);
                    }
                }
                
                sprintf(m_last_gps_info, "%.6f,%.6f", m_last_fm_msg.gps_lat, m_last_fm_msg.gps_long);

                /* Chế độ/ trạng thái hoạt động */
                switch (app_flash_get_operate_mode())
                {
                    case APP_AUDIO_OPERATION_MODE_INTERNET:
                        /* Nếu đang không stream internet thì là ở trạng thái IDLE */
                        if (!app_audio_is_http_audio_stream_running() &&
                            (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || app_mqtt_get_master_streamming_status()->Value == 0 ||
                            slave_get_http_received_data_timeout() == 0)) /* Quá lâu không nhận được gói tin http */
                        {
                            if (resend_mode_to_fm_module)
                            {
                                resend_mode_to_fm_module--;
                                mode = APP_AUDIO_OPERATION_MODE_INTERNET;
                            }
                            else
                            {
                                mode = APP_AUDIO_OPERATION_MODE_IDLE;
                            }
                        }
                        else
                        {
                            mode = APP_AUDIO_OPERATION_MODE_IDLE;
                        }
                        break;

                    case APP_AUDIO_OPERATION_MODE_FM:
                        freq = app_flash_get_current_fm_freq() / 10;
                        mode = APP_AUDIO_OPERATION_MODE_FM;
                        break;
                    case APP_AUDIO_OPERATION_MODE_MIC:
                        mode = APP_AUDIO_OPERATION_MODE_MIC;
                        break;

                    case APP_AUDIO_OPERATION_MODE_NO_OPERATION:
                        mode = APP_AUDIO_OPERATION_MODE_NO_OPERATION;
                        break;

                    case APP_AUDIO_OPERATION_MODE_IDLE:
                        mode = APP_AUDIO_OPERATION_MODE_IDLE;
                        break;
                        
                    default:
                        mode = APP_AUDIO_OPERATION_MODE_NO_OPERATION;
                        break;
                }

                m_freq_at_fm_slave = m_last_fm_msg.freq;

                ESP_LOGI(TAG, "mode %u, vol %u, gps (%.6f-%.6f), freq %uKhz,", 
                        m_last_fm_msg.mode,
                        m_last_fm_msg.vol,
                        m_last_fm_msg.gps_lat,
                        m_last_fm_msg.gps_long,
                        m_last_fm_msg.freq/1000);

                // // forward to gd32 in main
                // reply.id = MIN_ID_FORWARD;
                // reply.payload = buffer;
                // reply.len = size;
                // min_send_frame(&m_min_gd32_context, &reply);
            }

            /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
            if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE) 
               && app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
            {
                if (abs(app_audio_get_current_output_vol() - m_last_fm_msg.vol) > 3)
                {
                    ESP_LOGI(TAG, "Change codec volume: %u -> %u", app_audio_get_current_output_vol(), m_last_fm_msg.vol);
                    app_audio_change_codec_vol(m_last_fm_msg.vol);
                }
            }

            if (m_publish_fm_info == 1)
            {
                m_publish_fm_info++;
                if (m_publish_fm_info >= 5 && app_mqtt_is_connected_to_server())
                {
                    m_publish_fm_info = 0;
                    char tmp[48];
                    sprintf(tmp, "FM reset, version %u-%u", m_last_fm_msg.fw_version, m_last_fm_msg.hw_version);
                    mqtt_publish_message("DBG", tmp);
                }
            }

            static app_io_esp32_to_fm_frame_t frame;
            uint8_t mqtt_state = app_mqtt_get_state();
            frame.current_freq = freq;
            frame.mode = mode;
            frame.server_state = mqtt_state;
            frame.interface = network_get_current_interface();;
            frame.freq0 = app_flash_get_fm_freq1() / 10;
            frame.freq1 = app_flash_get_fm_freq2() / 10;
            frame.freq2 = app_flash_get_fm_freq3() / 10;
            frame.volume = app_flash_get_volume();
            
            
            static uint8_t buffer[MIN_MAX_PAYLOAD];
            uint32_t size = 0;


            min_msg_t reply;
            reply.id = MIN_ID_PING;
            reply.payload = (uint8_t*)&frame;
            reply.len = sizeof(frame);
            min_build_raw_frame_output(&reply, buffer, &size);
            reply.id = MIN_ID_FORWARD;
            reply.payload = buffer;
            reply.len = size;
            min_send_frame(&m_min_gd32_context, &reply);
            // ESP_LOGD(TAG, "mode %u, vol %u, gps (%.6f-%.6f), freq %uKhz, interface %d", 
            //         m_last_fm_msg.mode,
            //         m_last_fm_msg.vol,
            //         m_last_fm_msg.gps_lat,
            //         m_last_fm_msg.gps_long,
            //         m_last_fm_msg.freq/1000,
            //         frame.interface);
        }
            break;
        case MIN_ID_RESET:
        {
            ESP_LOGE(TAG, "FM reset\r\n");
            m_publish_fm_info = 1;
        }
            break;
        default:
            app_ota_on_slave_frame_callback(frame);
            break;
    }
}

void app_io_send_custom_frame_to_gd32(uint8_t id, uint8_t* data, uint32_t size)
{
    min_msg_t msg;
    msg.id = id;
    msg.len = size;
    msg.payload = data;
    xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
    min_send_frame(&m_min_gd32_context, &msg);
    xSemaphoreGive(m_sem_protect_gd32_uart);
}

void app_io_send_custom_frame_to_fm(uint8_t id, uint8_t* data, uint32_t size)
{
    min_msg_t reply;
    uint32_t out_size = 0;
    reply.id = id;
    reply.payload = (uint8_t*)data;
    reply.len = size;

    xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
    static uint8_t buffer[MIN_MAX_PAYLOAD];
    min_build_raw_frame_output(&reply, buffer, &out_size);
    ESP_LOGD(TAG, "Send min frame size %u", out_size);

    // forward to gd32 in main
    reply.id = MIN_ID_FORWARD;
    reply.payload = buffer;
    reply.len = out_size;

    min_send_frame(&m_min_gd32_context, &reply);
    xSemaphoreGive(m_sem_protect_gd32_uart);
}

static bool gd32_uart_tx(void *ctx, uint8_t data)
{
    if (ESP_OK != uart_write_bytes(FM_UART_PORT_NUM, (uint8_t*)&data, 1))
    {
        return false;
    }
    return true;
}
