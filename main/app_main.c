#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_console.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_ota.h>
#include <esp_insights.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

static const char *TAG = "app_main";

esp_rmaker_device_t *light_device;
esp_rmaker_device_t *temp_sensor_device;
esp_rmaker_device_t *alert_device;

/* Callback to handle param updates for the light */
static esp_err_t bulk_write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_write_req_t write_req[],
        uint8_t count, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    ESP_LOGI(TAG, "Light received %d params in write", count);
    for (int i = 0; i < count; i++) {
        const esp_rmaker_param_t *param = write_req[i].param;
        esp_rmaker_param_val_t val = write_req[i].val;
        const char *device_name = esp_rmaker_device_get_name(device);
        const char *param_name = esp_rmaker_param_get_name(param);
        if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
            ESP_LOGI(TAG, "Received value = %s for %s - %s",
                    val.val.b? "true" : "false", device_name, param_name);
            app_light_set_power(val.val.b);
        } else if (strcmp(param_name, ESP_RMAKER_DEF_BRIGHTNESS_NAME) == 0) {
            ESP_LOGI(TAG, "Received value = %d for %s - %s",
                    val.val.i, device_name, param_name);
            app_light_set_brightness(val.val.i);
        } else if (strcmp(param_name, ESP_RMAKER_DEF_HUE_NAME) == 0) {
            ESP_LOGI(TAG, "Received value = %d for %s - %s",
                    val.val.i, device_name, param_name);
            app_light_set_hue(val.val.i);
        } else if (strcmp(param_name, ESP_RMAKER_DEF_SATURATION_NAME) == 0) {
            ESP_LOGI(TAG, "Received value = %d for %s - %s",
                    val.val.i, device_name, param_name);
            app_light_set_saturation(val.val.i);
        } else {
            ESP_LOGI(TAG, "Updating for %s", param_name);
        }
        esp_rmaker_param_update(param, val);
    }
    return ESP_OK;
}

/* Simple alert callback */
static esp_err_t alert_callback(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                                const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    // Alert device is read-only for display purposes
    ESP_LOGI("ALERT", "Alert parameter is read-only");
    return ESP_OK;
}

/* Send security alert by updating alert parameters */
void send_security_alert(const char *message)
{
    ESP_LOGI("ALERT", "Security alert: %s", message);
    
    // Update alert status parameter
    esp_rmaker_param_t *alert_status = esp_rmaker_device_get_param_by_name(alert_device, "Alert Status");
    if (alert_status) {
        esp_rmaker_param_update_and_report(alert_status, esp_rmaker_str(message));
    }
    
    // Update alert level parameter (shows severity)
    esp_rmaker_param_t *alert_level = esp_rmaker_device_get_param_by_name(alert_device, "Alert Level");
    if (alert_level) {
        // Set alert level based on message content
        int level = 0; // Normal
        if (strstr(message, "breach") || strstr(message, "failed")) {
            level = 2; // High alert
        } else if (strstr(message, "attempt") || strstr(message, "warning")) {
            level = 1; // Warning
        } else if (strstr(message, "granted") || strstr(message, "ready")) {
            level = 0; // Normal
        }
        esp_rmaker_param_update_and_report(alert_level, esp_rmaker_int(level));
    }
    
    // Log to Insights
    ESP_LOGI("SECURITY_EVENT", "%s", message);
}

void app_main()
{
    /* Initialize Application specific hardware drivers and set initial state */
    esp_rmaker_console_init();
    app_driver_init();

    /* Initialize NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi */
    app_network_init();

    /* Initialize the ESP RainMaker Agent */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Smart Room", "Security System");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create Light Device (Google Assistant compatible) */
    light_device = esp_rmaker_lightbulb_device_create("Room Light", NULL, DEFAULT_POWER);
    esp_rmaker_device_add_bulk_cb(light_device, bulk_write_cb, NULL);
    esp_rmaker_device_add_param(light_device, esp_rmaker_brightness_param_create(ESP_RMAKER_DEF_BRIGHTNESS_NAME, DEFAULT_BRIGHTNESS));
    esp_rmaker_device_add_param(light_device, esp_rmaker_hue_param_create(ESP_RMAKER_DEF_HUE_NAME, DEFAULT_HUE));
    esp_rmaker_device_add_param(light_device, esp_rmaker_saturation_param_create(ESP_RMAKER_DEF_SATURATION_NAME, DEFAULT_SATURATION));
    esp_rmaker_node_add_device(node, light_device);

    /* Create Temperature Sensor Device with Humidity */
    temp_sensor_device = esp_rmaker_temp_sensor_device_create("Room Temperature", NULL, app_get_current_temperature());
    esp_rmaker_param_t *humidity_param = esp_rmaker_param_create("Humidity", "Humidity", 
                                                                esp_rmaker_float(app_get_current_humidity()),
                                                                PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (humidity_param) {
        esp_rmaker_device_add_param(temp_sensor_device, humidity_param);
    }
    esp_rmaker_node_add_device(node, temp_sensor_device);

    /* Create Security Alert Device as a simple switch */
    alert_device = esp_rmaker_switch_device_create("Security Alert", NULL, false);
    
    /* Add alert status parameter (shows messages) */
    esp_rmaker_param_t *alert_status = esp_rmaker_param_create("Alert Status", "Alert Status", 
                                                              esp_rmaker_str("System Ready"),
                                                              PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (alert_status) {
        esp_rmaker_device_add_param(alert_device, alert_status);
    }
    
    /* Add alert level parameter (shows severity 0-2) */
    esp_rmaker_param_t *alert_level = esp_rmaker_param_create("Alert Level", "Alert Level", 
                                                             esp_rmaker_int(0),
                                                             PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (alert_level) {
        esp_rmaker_device_add_param(alert_device, alert_level);
    }
    
    /* Add callback to make it read-only */
    esp_rmaker_device_add_cb(alert_device, alert_callback, NULL);
    
    esp_rmaker_node_add_device(node, alert_device);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable timezone service */
    esp_rmaker_timezone_service_enable();

    /* Enable scheduling */
    esp_rmaker_schedule_enable();

    /* Enable Scenes */
    esp_rmaker_scenes_enable();

    /* Enable system service */
    esp_rmaker_system_serv_config_t system_serv_config = {
        .flags = SYSTEM_SERV_FLAGS_ALL,
        .reboot_seconds = 2,
        .reset_seconds = 2,
        .reset_reboot_seconds = 2,
    };
    esp_rmaker_system_service_enable(&system_serv_config);

    /* Enable Insights */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the network */
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start network. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    ESP_LOGI(TAG, "Smart Security System started");
}