/******************************************************************************
 * @file:    app_cli.c
 * @brief:
 * @version: V0.0.0
 * @date:    2019/11/12
 * @author:
 * @email:
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". VINSMART
 * JSC MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. VINSMART JSC
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 *
 * VINSMART JSC SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 *
 * (C)Copyright VINSMART JSC 2019 All rights reserved
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_cli.h"
#include "app_shell.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
static const char *TAG = "cli";

static int32_t cli_get_memory(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_get_current_time(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_get_config(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_wifi_connect(p_shell_context_t context, int32_t argc, char **argv);
static int32_t cli_gpio_action(p_shell_context_t context, int32_t argc, char **argv);
// static int32_t cli_get_vin(p_shell_context_t context, int32_t argc, char **argv);
// static int32_t cli_factory_reset(p_shell_context_t context, int32_t argc, char **argv);
// static int32_t cli_fake_alarm(p_shell_context_t context, int32_t argc, char **argv);
// static int32_t cli_fake_alarm_delay(p_shell_context_t context, int32_t argc, char **argv);
// static int32_t cli_fake_test_mode(p_shell_context_t context, int32_t argc, char **argv);

static const shell_command_context_t cli_command_table[] =
    {
        {"mem", "\tmem: Get free memory size\r\n", cli_get_memory, 0},
        {"reset", "\treset: Reset\r\n", cli_reset, -1},
        // {"ota", "\tota: Update firmware, ex ota http://asdasd/bin\r\n", cli_ota_update, 1},
        // {"time", "\ttime: Get current time\r\n", cli_get_current_time, 0},
        // {"cfg", "\tcfg: Configuration cmd\r\n", cli_get_config, 2},
        // {"wifi", "\twifi: Connect to ssid/password\r\n", cli_wifi_connect, 2},
        // {"gpio", "\tgpio: num-[read/set/reset]\r\n", cli_gpio_action, 0},
        // {"vin", "\tvin: Read vin voltage\r\n", cli_get_vin, 0},
        // {"factory", "\tfactory: Factory reset\r\n", cli_factory_reset, 0},
        // {"alarm", "\talarm: on/off to fake alarm event\r\n", cli_fake_alarm, 1},
        // {"delay", "\tdelay: Fake alarm delay\r\n", cli_fake_alarm_delay, 1},
        // {"test", "\ttest: Fake test mode\r\n", cli_fake_test_mode, 1},
};

static shell_context_struct m_user_context;
static app_cli_cb_t *m_cb;

void app_cli_poll(uint8_t ch)
{
    app_shell_task(ch);
}

void app_cli_start(app_cli_cb_t *callback)
{
    m_cb = callback;
    app_shell_set_context(&m_user_context);
    app_shell_init(&m_user_context,
                   m_cb->puts,
                   m_cb->printf,
                   ">",
                   true);

    /* Register CLI commands */
    for (int i = 0; i < sizeof(cli_command_table) / sizeof(shell_command_context_t); i++)
    {
        app_shell_register_cmd(&cli_command_table[i]);
    }

    /* Run CLI task */
    app_shell_task(APP_SHELL_INVALID_CHAR);
}

/* Reset System */
static int32_t cli_get_memory(p_shell_context_t context, int32_t argc, char **argv)
{
    char tmp[256];
    sprintf(tmp, "Free memory size %u-%u\r\n", xPortGetFreeHeapSize(), esp_get_free_internal_heap_size());
    // char tmp[512+128];
    // vTaskGetRunTimeStats(tmp);
    // ESP_LOGI(TAG, tmp);
    // ESP_LOGI(TAG, "\r\n");
    m_cb->printf(tmp);
    return 0;
}

static int32_t cli_reset(p_shell_context_t context, int32_t argc, char **argv)
{
    m_cb->printf("System will reset now\r\n");
    vTaskDelay(500);
    esp_restart();
    return 0;
}

// extern void app_ota_start(char *url);
static int32_t cli_ota_update(p_shell_context_t context, int32_t argc, char **argv)
{
    if (strstr(argv[1], "http://") || strstr(argv[1], "https://"))
    {
        ESP_LOGI(TAG, "OTA url %s\r\n", argv[1]);
        // ota_url_t url;
        // url.url = argv[1];
        // // url.port = atoi(argv[2]);
        // url.username = "";
        // url.password = "";
        // app_ota_start(&url);
    }

    return 0;
}

static int32_t cli_get_current_time(p_shell_context_t context, int32_t argc, char **argv)
{
    // app_sntp_debug_timenow();
    return 0;
}

static int32_t cli_get_config(p_shell_context_t context, int32_t argc, char **argv)
{
    if (strstr(argv[1], "dump"))
    {
        //		internal_flash_cfg_t *cfg = internal_flash_get_config();
        //		(void)cfg;
        //		ESP_LOGI(TAG, "\t\tConfig addr %s:%d\r\n\tPing nterval %ums", cfg->host_addr, cfg->port, cfg->ping_cycle);
    }

    return 0;
}
