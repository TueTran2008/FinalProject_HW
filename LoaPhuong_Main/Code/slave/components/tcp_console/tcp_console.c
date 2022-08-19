#include "tcp_console.h"
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "network.h"
#include "app_cli.h"

#define PORT                        80
#define KEEPALIVE_IDLE              60
#define KEEPALIVE_INTERVAL          120
#define KEEPALIVE_COUNT             3
#define TCP_RX_BUFFER_SIZE          128
#define PORT 80

static const char *TAG  = "tcp_console";
static bool m_task_created = false;
static uint32_t tcp_console_timeout = 120;      // 120s
static bool m_cli_started = false;

static void tcp_server_send(uint8_t *buffer, uint32_t size);
static int32_t tcp_printf(const char *msg);
static char *m_tcp_rx_buffer;
static uint32_t m_read_index = 0;
static int sock;
static app_cli_cb_t m_tcp_cli = 
{
    .puts = tcp_server_send,
    .printf = tcp_printf
};


static void do_retransmit(const int sock)
{
    int len;
    if (!m_tcp_rx_buffer)
    {
        m_tcp_rx_buffer = malloc(TCP_RX_BUFFER_SIZE);
        assert(m_tcp_rx_buffer);
    }
    do 
    {
        len = recv(sock, m_tcp_rx_buffer, TCP_RX_BUFFER_SIZE - 1, 0);
        if (len < 0) 
        {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            break;
        } 
        else if (len == 0) 
        {
            ESP_LOGW(TAG, "Connection closed");
            break;
        } 
        else 
        {
            m_tcp_rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGD(TAG, "Received %d bytes: %s", len, m_tcp_rx_buffer);
            
            if (m_cli_started == false)
            {
                m_cli_started = true;
                app_cli_start(&m_tcp_cli);
            }   

            for (uint32_t i = 0; i < len; i++)
            {
                app_cli_poll(m_tcp_rx_buffer[i]);
            }

            // // send() can return less bytes than supplied length.
            // // Walk-around for robust implementation.
            // int to_write = len;
            // while (to_write > 0) 
            // {
            //     int written = send(sock, m_tcp_rx_buffer + (len - to_write), to_write, 0);
            //     if (written < 0) {
            //         ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            //         to_write = 0;
            //     }
            //     else
            //     {
            //         to_write -= written;
            //     }
            // }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    } while (1);

    if (m_tcp_rx_buffer)
    {
        free(m_tcp_rx_buffer);
        m_tcp_rx_buffer = NULL;
    }
}

static void tcp_server_send(uint8_t *buffer, uint32_t size)
{
#if 0
    int to_write = size;
    while (to_write > 0) 
    {
        int written = send(sock, m_tcp_rx_buffer + (size - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            to_write = 0;
        }
        else
        {
            to_write -= written;
        }
    }
#else
    send(sock, buffer, size, 0);
#endif
}

static int32_t tcp_printf(const char *msg)
{
    uint32_t len = strlen(msg);
    tcp_server_send(msg, len);
    return len;
}


static void tcp_console_task(void *arg)
{
    while (!network_is_ppp_got_ip())
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) 
    {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) 
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) 
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) 
    {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) 
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
        break;
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void tcp_console_start(void)
{
    if (m_task_created)
    {
        return;
    }
    m_task_created = true;
    assert(xTaskCreate(tcp_console_task, "tcp_console", 2048+1024, NULL, tskIDLE_PRIORITY+1, NULL));
}