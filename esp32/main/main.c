/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    system_event_sta_disconnected_t *disconn;
    wifi_mode_t mode;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
        disconn = &event->event_info.disconnected;
        switch (disconn->reason) {
        case WIFI_REASON_AUTH_FAIL:
            printf("WiFi: desconntcted after auth fail\r\n");
            break;
        default:
            // try to reconnect
            if (esp_wifi_get_mode(&mode) == ESP_OK) {
                if (mode & WIFI_MODE_STA) {
                    printf("WiFi: try to reconnect...\r\n");
                    esp_wifi_connect();
                }
            }
            break;
        }
    default:
        break;
    }
    return ESP_OK;
}

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS    5
#define PIN_NUM_HS   22

static void post_setup_cb(spi_slave_transaction_t *trans)
{
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1<<PIN_NUM_HS));
}

static void post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1<<PIN_NUM_HS));
}

static void spi_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=PIN_NUM_CS,
        .queue_size=3,
        .flags=0,
        .post_setup_cb=post_setup_cb,
        .post_trans_cb=post_trans_cb
     };

    //Initialize the SPI bus
    ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 0);
    assert(ret == ESP_OK);
}

static void xgpio_init(void)
{
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);
    gpio_set_level(PIN_NUM_HS, 0);
    gpio_set_direction(PIN_NUM_HS, GPIO_MODE_OUTPUT);
}

#define PACKET_SIZE 12
#define PKT_QSIZE  64

static xQueueHandle pkt_queue;

static void spi_task(void *arg)
{
    char sendbuf[PACKET_SIZE];
    char recvbuf[PACKET_SIZE];

    // Currently unused
    memset(sendbuf, 0, PACKET_SIZE);

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = PACKET_SIZE*8;
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    while(1) {
        esp_err_t ret = spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
        //printf("spi_slave_transmit -> %x %02x %02x\n", ret, recvbuf[0], recvbuf[1]);
        if (ret == ESP_OK) {
            if (xQueueSend(pkt_queue, &recvbuf[0], 0) != pdTRUE) {
                printf("fail to queue packet\n");
            }
        }
    }
}

#define UDP_SERVER CONFIG_UDP_SERVER_ADDRESS
#define UDP_PORT CONFIG_UDP_PORT

static void udp_task(void *arg)
{
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    printf("UDP client task starting...\r\n");

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated socket\r\n");

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    caddr.sin_port = htons(UDP_PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (UDP_SERVER);
    saddr.sin_port = htons(UDP_PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }
    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    xTaskCreate(spi_task, "spi_task", 2048, NULL, 3, NULL);

    int len = PACKET_SIZE;
    char recvbuf[PACKET_SIZE];
    while(1) {
        if (xQueueReceive(pkt_queue, &recvbuf[0], portMAX_DELAY) == pdTRUE) {
            int n = send(s, recvbuf, len, 0);
            if (n < 0) {
            }
        }
    }
}

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_SSID,
            .password = CONFIG_SSID_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    esp_log_level_set("wifi", ESP_LOG_WARN);

    xgpio_init();
    spi_init();

    // packet queue
    pkt_queue = xQueueCreate(PKT_QSIZE, PACKET_SIZE);

    xTaskCreate(udp_task, "udp_task", 2048, NULL, 4, NULL);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_2, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

