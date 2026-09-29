idf_component_register(SRCS "main.c" "clock_service.c" INCLUDE_DIRS "." REQUIRES esp_driver_spi esp_driver_gpio esp_timer esp_wifi esp_event esp_netif nvs_flash)
