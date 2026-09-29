#include "clock_service.h"
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static TimerHandle_t reconnect_timer;

bool clock_service_configured(void)
{
    return strlen(CONFIG_TARS_WIFI_SSID) > 0;
}

static void reconnect(TimerHandle_t timer)
{
    (void)timer;
    esp_wifi_connect();
}

static void network_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xTimerReset(reconnect_timer, 0);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xTimerStop(reconnect_timer, 0);
        esp_err_t error = esp_netif_sntp_start();
        if (error != ESP_OK) ESP_LOGW("clock", "SNTP: %s", esp_err_to_name(error));
    }
}

void clock_service_init(void)
{
    setenv("TZ", "BRT3", 1);
    tzset();
    if (!clock_service_configured()) return;
    if (strlen(CONFIG_TARS_WIFI_SSID) > 32 || strlen(CONFIG_TARS_WIFI_PASSWORD) > 64) {
        ESP_LOGE("clock", "Configuracao Wi-Fi excede o tamanho permitido");
        return;
    }
    esp_err_t error = nvs_flash_init();
    if (error != ESP_OK) {
        ESP_LOGE("clock", "NVS indisponivel: %s", esp_err_to_name(error));
        return;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(netif ? ESP_OK : ESP_ERR_NO_MEM);
    esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    sntp.start = false;
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp));
    reconnect_timer = xTimerCreate("wifi_retry", pdMS_TO_TICKS(5000), pdFALSE, NULL, reconnect);
    ESP_ERROR_CHECK(reconnect_timer ? ESP_OK : ESP_ERR_NO_MEM);
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, network_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, network_event, NULL));
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, CONFIG_TARS_WIFI_SSID, strlen(CONFIG_TARS_WIFI_SSID));
    memcpy(config.sta.password, CONFIG_TARS_WIFI_PASSWORD, strlen(CONFIG_TARS_WIFI_PASSWORD));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

bool clock_service_read(struct tm *local)
{
    time_t now = time(NULL);
    localtime_r(&now, local);
    return local->tm_year >= 124;
}
