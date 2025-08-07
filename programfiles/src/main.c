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
        init_servos();
       

        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
         sweep_servos_once();
      //  xTaskCreate(&wifi_scan_task, "wifi_scan_task", 4096, NULL, 5, NULL);
        

    // Call the sweep function once
    
    }


#include "driver/ledc.h"
#include "esp_err.h"

#define SERVO1_GPIO 19  // Servo 1: Clockwise
#define SERVO2_GPIO 18  // Servo 2: Anticlockwise

// Convert angle (0–180) to PWM duty cycle (for 16-bit resolution and 50Hz)
uint32_t angle_to_duty(int angle) {
    int pulse_width_us = 500 + (angle * 2000 / 180);  // 0.5ms to 2.5ms pulse width
    return (pulse_width_us * 65535) / 20000;          // Convert to 16-bit duty for 20ms period (50Hz)
}

void init_servos() {
    // Configure 50Hz PWM timer
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    // Servo 1 channel config
    ledc_channel_config_t servo1 = {
        .gpio_num = SERVO1_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&servo1);

    // Servo 2 channel config
    ledc_channel_config_t servo2 = {
        .gpio_num = SERVO2_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&servo2);
}

void sweep_servos_once() {
    // --- Servo 1: Clockwise (0° to 180°) ---
    for (int angle = 0; angle <= 180; angle += 10) {
        uint32_t duty = angle_to_duty(angle);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Hold at 180°
    vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds before starting Servo 2

    // --- Servo 2: Anticlockwise (180° to 0°) ---
    for (int angle = 180; angle >= 0; angle -= 10) {
        uint32_t duty = angle_to_duty(angle);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Optional: pause before returning
    vTaskDelay(pdMS_TO_TICKS(1000));
}


