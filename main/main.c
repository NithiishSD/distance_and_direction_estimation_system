#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bluedroid_hci.h"
#include "esp_gap_ble_api.h"

// Tag for logging
static const char *TAG = "DEVICE_SCAN";

// Define structures for storing detected devices
#define MAX_WIFI_DEVICES 20
#define MAX_BT_DEVICES 20

typedef struct
{
    char ssid[33]; // SSID max length is 32 + null terminator
    uint8_t mac[6];
    uint8_t channel;
    int32_t rssi;
} wifi_device_t;

typedef struct
{
    uint8_t mac[6];
    char name[32];
    int32_t rssi;
} bt_device_t;

// Global arrays to store detected devices
wifi_device_t wifi_devices[MAX_WIFI_DEVICES];
bt_device_t bt_devices[MAX_BT_DEVICES];
int wifi_device_count = 0;
int bt_device_count = 0;

// Baseline noise levels (average RSSI from non-target devices)
float wifi_baseline_noise = 0.0;
float bt_baseline_noise = 0.0;

// BLE GAP callback for scan results
static void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_SCAN_RESULT_EVT)
    {
        esp_ble_gap_cb_param_t *scan_result = param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT && bt_device_count < MAX_BT_DEVICES)
        {
            bt_device_t device;
            memcpy(device.mac, scan_result->scan_rst.bda, 6);
            // Get device name if available
            device.name[0] = '\0';
            if (scan_result->scan_rst.adv_data_len > 0)
            {
                uint8_t *adv_data = scan_result->scan_rst.adv_data;
                uint8_t len = 0;
                uint8_t *data = esp_ble_resolve_adv_data(adv_data, ESP_BLE_AD_TYPE_NAME_CMPL, &len);
                if (data && len < 32)
                {
                    memcpy(device.name, data, len);
                    device.name[len] = '\0';
                }
            }
            device.rssi = scan_result->scan_rst.rssi;
            bt_devices[bt_device_count++] = device;

            // Log device details
            ESP_LOGI(TAG, "BT Device: MAC=%02x:%02x:%02x:%02x:%02x:%02x, Name=%s, RSSI=%d",
                     device.mac[0], device.mac[1], device.mac[2], device.mac[3], device.mac[4], device.mac[5],
                     device.name, device.rssi);
        }
        else if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
        {
            // Scan complete, calculate baseline noise
            int32_t total_rssi = 0;
            for (int i = 0; i < bt_device_count; i++)
            {
                total_rssi += bt_devices[i].rssi;
            }
            bt_baseline_noise = (bt_device_count > 0) ? (float)total_rssi / bt_device_count : 0.0;
            ESP_LOGI(TAG, "BT Baseline Noise (Avg RSSI): %.2f dBm", bt_baseline_noise);
        }
    }
}

// Initialize NVS (required for Wi-Fi and Bluetooth)
static void initialize_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// Initialize Wi-Fi
static void initialize_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Initialize Bluetooth (BLE only)
static void initialize_ble(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)); // Release Classic BT memory
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_gap_callback));
}

// Perform Wi-Fi scan
static void wifi_scan(void)
{
    wifi_device_count = 0;
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0, // Scan all channels
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = {
            .min = 120,
            .max = 120}};

    ESP_LOGI(TAG, "Starting Wi-Fi scan...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    uint16_t ap_count = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_list == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for Wi-Fi scan results");
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));
    int32_t total_rssi = 0;

    for (int i = 0; i < ap_count && wifi_device_count < MAX_WIFI_DEVICES; i++)
    {
        wifi_device_t device;
        strncpy(device.ssid, (char *)ap_list[i].ssid, 33);
        memcpy(device.mac, ap_list[i].bssid, 6);
        device.channel = ap_list[i].primary;
        device.rssi = ap_list[i].rssi;
        wifi_devices[wifi_device_count++] = device;
        total_rssi += device.rssi;

        // Log device details
        ESP_LOGI(TAG, "WiFi Device: SSID=%s, MAC=%02x:%02x:%02x:%02x:%02x:%02x, Channel=%d, RSSI=%d",
                 device.ssid, device.mac[0], device.mac[1], device.mac[2], device.mac[3], device.mac[4], device.mac[5],
                 device.channel, device.rssi);
    }

    // Calculate baseline noise
    wifi_baseline_noise = (wifi_device_count > 0) ? (float)total_rssi / wifi_device_count : 0.0;
    ESP_LOGI(TAG, "Wi-Fi Baseline Noise (Avg RSSI): %.2f dBm", wifi_baseline_noise);

    free(ap_list);
}

// Perform BLE scan
static void ble_scan(void)
{
    bt_device_count = 0;
    ESP_LOGI(TAG, "Starting BLE scan...");
    esp_ble_scan_params_t ble_scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50, // 80 ms
        .scan_window = 0x30,   // 30 ms
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(5)); // Scan for 5 seconds
}

void init_servos()
{
    // Configure 50Hz PWM timer
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer);

    // Servo 1 channel config
    ledc_channel_config_t servo1 = {
        .gpio_num = SERVO1_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&servo1);

    // Servo 2 channel config
    ledc_channel_config_t servo2 = {
        .gpio_num = SERVO2_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&servo2);
}

void sweep_servos_once()
{
    // --- Servo 1: Clockwise (0° to 180°) ---
    for (int angle = 0; angle <= 180; angle += 10)
    {
        uint32_t duty = angle_to_duty(angle);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Hold at 180°
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before starting Servo 2

    // --- Servo 2: Anticlockwise (180° to 0°) ---
    for (int angle = 180; angle >= 0; angle -= 10)
    {
        uint32_t duty = angle_to_duty(angle);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Optional: pause before returning
    vTaskDelay(pdMS_TO_TICKS(1000));
}
void app_main(void)
{
    // Initialize NVS
    initialize_nvs();

    // Initialize Wi-Fi and perform scan
    initialize_wifi();
    wifi_scan();

    // Stop Wi-Fi to free resources for BLE
    ESP_ERROR_CHECK(esp_wifi_stop());

    // Initialize BLE and perform scan
    initialize_ble();
    ble_scan();

    // Note: BLE scan runs asynchronously; results are handled in ble_gap_callback
    // Wait for BLE scan to complete (5 seconds + margin)
    vTaskDelay(pdMS_TO_TICKS(6000));

    // Stop BLE to free resources
    ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
    ESP_ERROR_CHECK(esp_bluedroid_disable());
    ESP_ERROR_CHECK(esp_bluedroid_deinit());
    ESP_ERROR_CHECK(esp_bt_controller_disable());
    ESP_ERROR_CHECK(esp_bt_controller_deinit());

    ESP_LOGI(TAG, "Scan complete. Wi-Fi devices: %d, BLE devices: %d", wifi_device_count, bt_device_count);
    ESP_LOGI(TAG, "Devices saved for further use. Use wifi_devices and bt_devices arrays for target selection.");
}
// xTaskCreate(&wifi_scan_task, "wifi_scan_task", 4096, NULL, 5, NULL);
//  connect_to_ap("Varunesh", "varunbroo");
