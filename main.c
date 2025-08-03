    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_wifi.h"
    #include "esp_event.h"
    #include "esp_log.h"
    #include "nvs_flash.h"

    static const char *TAG = "WiFiScanner";
    wifi_scan_config_t scan_config = {
                .ssid = 0,
                .bssid = 0,
                .channel = 0,
                .show_hidden = true};
    void wifi_scan_task(void *pvParameters)
    {
        while (1)
        {
            

            ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

            uint16_t ap_num = 0;
            esp_wifi_scan_get_ap_num(&ap_num);
            wifi_ap_record_t ap_records[ap_num];
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

            ESP_LOGI(TAG, "Found %d access points:", ap_num);
            for (int i = 0; i < ap_num; i++)
            {
                ESP_LOGI(TAG, "SSID: %s, RSSI: %d, Channel: %d",
                         ap_records[i].ssid,
                         ap_records[i].rssi,
                         ap_records[i].primary);
            }

            vTaskDelay(pdMS_TO_TICKS(2000)); // wait 2 seconds

        }
    }

    void app_main(void)
    {
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        xTaskCreate(&wifi_scan_task, "wifi_scan_task", 4096, NULL, 5, NULL);
    }


