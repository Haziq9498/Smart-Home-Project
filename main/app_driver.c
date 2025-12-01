#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_insights.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <dht.h>
#include <iot_button.h>
#include <button_gpio.h>
#include <string.h>

#include <app_reset.h>

#include "app_priv.h"

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0
#define DHT11_GPIO          2

static TimerHandle_t sensor_timer;

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

// Light variables
static uint16_t g_value = DEFAULT_BRIGHTNESS;
static bool g_power = DEFAULT_POWER;

// Sensor variables
static float g_temperature = DEFAULT_TEMPERATURE;
static float g_humidity = DEFAULT_HUMIDITY;
static uint32_t g_sensor_read_count = 0;
static uint32_t g_sensor_error_count = 0;

/* ===== SIMPLE LED FUNCTIONS ===== */

/* Initialize LED GPIO */
static void init_led(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Start with LED off
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI("LED", "LED initialized on GPIO %d", LED_GPIO);
}

esp_err_t app_light_set_power(bool power)
{
    g_power = power;
    
    if (power) {
        gpio_set_level(LED_GPIO, 1); // Turn LED ON
        ESP_LOGI("LED", "LED turned ON");
    } else {
        gpio_set_level(LED_GPIO, 0); // Turn LED OFF
        ESP_LOGI("LED", "LED turned OFF");
    }
    return ESP_OK;
}

esp_err_t app_light_set_brightness(uint16_t brightness)
{
    g_value = brightness;
    // For simple LED, brightness just controls on/off
    if (brightness > 0) {
        app_light_set_power(true);
    } else {
        app_light_set_power(false);
    }
    ESP_LOGI("LED", "LED brightness set to: %d%%", brightness);
    return ESP_OK;
}

// For single LED, hue and saturation don't do anything
esp_err_t app_light_set_hue(uint16_t hue) 
{ 
    ESP_LOGI("LED", "Hue setting ignored for single LED");
    return ESP_OK; 
}

esp_err_t app_light_set_saturation(uint16_t saturation) 
{ 
    ESP_LOGI("LED", "Saturation setting ignored for single LED");
    return ESP_OK; 
}

esp_err_t app_light_init(void)
{
    init_led();
    if (g_power) {
        app_light_set_power(true);
    } else {
        app_light_set_power(false);
    }
    return ESP_OK;
}

/* ===== SENSOR FUNCTIONS ===== */

static bool read_dht11_sensor(void)
{
    float temperature = 0.0;
    float humidity = 0.0;
    
    esp_err_t result = dht_read_float_data(DHT_TYPE_DHT11, DHT11_GPIO, &humidity, &temperature);
    
    if (result == ESP_OK) {
        g_temperature = temperature;
        g_humidity = humidity;
        g_sensor_read_count++;
        
        ESP_LOGI("SENSOR_READING", "temperature:%.1f, humidity:%.1f", temperature, humidity);
        ESP_LOGI("DHT11", "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
        return true;
    } else {
        ESP_LOGE("DHT11", "Failed to read sensor data. Error: %d", result);
        g_sensor_error_count++;
        return false;
    }
}

static void app_sensor_update(TimerHandle_t handle)
{
    static uint32_t update_count = 0;
    update_count++;
    
    if (read_dht11_sensor()) {
        // Report temperature to RainMaker
        esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
                esp_rmaker_float(g_temperature));
        
        // Report humidity to RainMaker
        esp_rmaker_param_t *humidity_param = esp_rmaker_device_get_param_by_name(temp_sensor_device, "Humidity");
        if (humidity_param) {
            esp_rmaker_param_update_and_report(humidity_param, esp_rmaker_float(g_humidity));
        }
                 
    } else {
        ESP_LOGE("SENSOR_UPDATE", "update_count:%lu, status:failed", update_count);
    }
}

float app_get_current_temperature(void)
{
    return g_temperature;
}

float app_get_current_humidity(void)
{
    return g_humidity;
}

static void init_sensor(void)
{
    // Initialize DHT11 sensor with an initial read
    if (!read_dht11_sensor()) {
        ESP_LOGW("DHT11", "Initial sensor read failed, using default values");
    }

    // Create timer for periodic sensor updates
    sensor_timer = xTimerCreate(
        "sensor_update_tm", 
        pdMS_TO_TICKS(REPORTING_PERIOD * 1000),
        pdTRUE, 
        NULL, 
        app_sensor_update
    );
    
    if (sensor_timer) {
        if (xTimerStart(sensor_timer, 0) == pdPASS) {
            ESP_LOGI("DHT11", "Sensor timer started with %d second interval", REPORTING_PERIOD);
        }
    }
}

/* ===== KEYPAD FUNCTIONS ===== */

static const char keypad_map[4][4] = {
    {'D', 'C', 'B', 'A'},
    {'#', '9', '6', '3'},
    {'0', '8', '5', '2'},
    {'*', '7', '4', '1'}
};


static int row_pins[4] =  {ROW1_GPIO, ROW2_GPIO, ROW3_GPIO, ROW4_GPIO};
static int col_pins[4] =  {COL1_GPIO, COL2_GPIO, COL3_GPIO, COL4_GPIO};

static char password_buffer[20] = {0};
static int buffer_index = 0;
static int failed_attempts = 0;
static bool alarm_active = false;
static TimerHandle_t alarm_timer = NULL;

/* Initialize keypad GPIO */
void keypad_init(void)
{
    // Initialize row pins as outputs (set high initially)
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << row_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(row_pins[i], 1); // Set rows high initially
    }

    // Initialize column pins as inputs with pull-up
    for (int i = 0; i < KEYPAD_COLS; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << col_pins[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }

    // Initialize alarm LED
    gpio_config_t alarm_led_conf = {
        .pin_bit_mask = (1ULL << ALARM_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&alarm_led_conf);
    gpio_set_level(ALARM_LED_GPIO, 0); // Start with alarm LED off

    ESP_LOGI("KEYPAD", "Keypad initialized with password: %s", CORRECT_PASSWORD);
}

/* Scan keypad for pressed key */
char keypad_get_key(void)
{
    for (int row = 0; row < KEYPAD_ROWS; row++) {
        // Set current row low
        gpio_set_level(row_pins[row], 0);
        
        // Wait a bit for signal to settle
        vTaskDelay(10 / portTICK_PERIOD_MS);
        
        // Check each column
        for (int col = 0; col < KEYPAD_COLS; col++) {
            if (gpio_get_level(col_pins[col]) == 0) {
                // Key pressed - wait for debounce
                vTaskDelay(50 / portTICK_PERIOD_MS);
                
                // Confirm key is still pressed
                if (gpio_get_level(col_pins[col]) == 0) {
                    // Wait for key release
                    while (gpio_get_level(col_pins[col]) == 0) {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                    
                    // Restore row to high
                    gpio_set_level(row_pins[row], 1);
                    
                    char key = keypad_map[row][col];
                    ESP_LOGI("KEYPAD", "Key pressed: %c", key);
                    return key;
                }
            }
        }
        
        // Restore row to high
        gpio_set_level(row_pins[row], 1);
    }
    
    return '\0'; // No key pressed
}

/* Check if entered password is correct */
bool check_password(const char *input)
{
    bool correct = (strcmp(input, CORRECT_PASSWORD) == 0);
    ESP_LOGI("KEYPAD", "Password check: %s -> %s", input, correct ? "CORRECT" : "WRONG");
    return correct;
}

/* Blink alarm LED */
static void blink_alarm(TimerHandle_t xTimer)
{
    static bool led_state = false;
    led_state = !led_state;
    gpio_set_level(ALARM_LED_GPIO, led_state ? 1 : 0);
}

/* Trigger alarm - blink LED and send notification */
void trigger_alarm(void)
{
    if (!alarm_active) {
        alarm_active = true;
        ESP_LOGE("KEYPAD", "ALARM TRIGGERED! Too many failed attempts");
        
        // DEBUG: Log the trigger
        ESP_LOGI("DEBUG", "=== ALARM TRIGGER FUNCTION STARTED ===");
        ESP_LOGI("DEBUG", "Failed attempts: %d", failed_attempts);
        
        // Send push notification via parameter update
        char notification_msg[100];
        snprintf(notification_msg, sizeof(notification_msg), 
                 "Security breach! %d failed password attempts detected. Alarm activated.", 
                 failed_attempts);
        
        ESP_LOGI("DEBUG", "Calling send_security_alert: %s", notification_msg);
        send_security_alert(notification_msg);
        
        // METHOD 1: Try esp_rmaker_raise_alert
        ESP_LOGI("DEBUG", "Attempting Method 1: esp_rmaker_raise_alert");
        esp_err_t alert_result = esp_rmaker_raise_alert("🚨 SECURITY BREACH! Too many failed password attempts detected!");
        ESP_LOGI("DEBUG", "esp_rmaker_raise_alert result: %s", esp_err_to_name(alert_result));
        
        // METHOD 2: Update alert status with notification
        ESP_LOGI("DEBUG", "Attempting Method 2: param_update_and_notify");
        esp_rmaker_param_t *alert_status = esp_rmaker_device_get_param_by_name(alert_device, "Alert Status");
        if (alert_status) {
            ESP_LOGI("DEBUG", "Found alert_status parameter");
            esp_err_t notify_result = esp_rmaker_param_update_and_notify(alert_status, esp_rmaker_str("🚨 SECURITY BREACH - Too many failed attempts!"));
            ESP_LOGI("DEBUG", "esp_rmaker_param_update_and_notify result: %s", esp_err_to_name(notify_result));
        } else {
            ESP_LOGE("DEBUG", "Could not find alert_status parameter!");
        }
        
        // METHOD 3: Also update alert level with notification
        esp_rmaker_param_t *alert_level = esp_rmaker_device_get_param_by_name(alert_device, "Alert Level");
        if (alert_level) {
            ESP_LOGI("DEBUG", "Found alert_level parameter");
            esp_rmaker_param_update_and_notify(alert_level, esp_rmaker_int(2)); // High alert
        } else {
            ESP_LOGE("DEBUG", "Could not find alert_level parameter!");
        }
        
        ESP_LOGI("DEBUG", "=== ALARM TRIGGER FUNCTION COMPLETED ===");
        
        // Create timer to blink LED every 500ms
        alarm_timer = xTimerCreate(
            "alarm_blink",
            pdMS_TO_TICKS(500),
            pdTRUE,
            NULL,
            blink_alarm
        );
        
        if (alarm_timer) {
            xTimerStart(alarm_timer, 0);
            ESP_LOGI("DEBUG", "Alarm blink timer started");
        } else {
            ESP_LOGE("DEBUG", "Failed to create alarm blink timer");
        }
        
        // Alarm will blink for 30 seconds then stop automatically
        TimerHandle_t stop_timer = xTimerCreate(
            "alarm_stop",
            pdMS_TO_TICKS(30000),
            pdFALSE,
            NULL,
            (TimerCallbackFunction_t)stop_alarm
        );
        if (stop_timer) {
            xTimerStart(stop_timer, 0);
            ESP_LOGI("DEBUG", "Alarm stop timer started (30 seconds)");
        } else {
            ESP_LOGE("DEBUG", "Failed to create alarm stop timer");
        }
    } else {
        ESP_LOGI("DEBUG", "Alarm already active, ignoring duplicate trigger");
    }
}

/* Stop alarm */
void stop_alarm(void)
{
    if (alarm_active) {
        alarm_active = false;
        ESP_LOGI("DEBUG", "Stopping alarm");
        if (alarm_timer) {
            xTimerStop(alarm_timer, 0);
            xTimerDelete(alarm_timer, 0);
            alarm_timer = NULL;
            ESP_LOGI("DEBUG", "Alarm timer stopped and deleted");
        }
        gpio_set_level(ALARM_LED_GPIO, 0);
        ESP_LOGI("KEYPAD", "Alarm stopped");
        
        // Send all-clear notification
        send_security_alert("Alarm deactivated. System normal.");
    }
}

/* Keypad task to handle password input */
static void keypad_task(void *pvParameters)
{
    ESP_LOGI("KEYPAD", "Keypad task started");
    
    while (1) {
        char key = keypad_get_key();
        
        if (key != '\0') {
            ESP_LOGI("DEBUG", "Keypad input: %c", key);
            
            if (key == '#') {
                // Enter key - check password
                password_buffer[buffer_index] = '\0';
                ESP_LOGI("DEBUG", "Checking password: %s", password_buffer);
                
                if (check_password(password_buffer)) {
                    ESP_LOGI("KEYPAD", "Password CORRECT! Access granted");
                    failed_attempts = 0; // Reset counter
                    stop_alarm(); // Stop alarm if active
                    
                    // Send success notification
                    send_security_alert("Access granted. Security system disarmed.");
                    
                    // Turn on main light as reward
                    app_light_set_power(true);
                    esp_rmaker_param_update_and_report(
                        esp_rmaker_device_get_param_by_type(light_device, ESP_RMAKER_PARAM_POWER),
                        esp_rmaker_bool(true));
                } else {
                    failed_attempts++;
                    ESP_LOGI("KEYPAD", "Wrong password! Attempt %d/%d", failed_attempts, MAX_ATTEMPTS);
                    
                    // Send warning notification on first wrong attempt
                    if (failed_attempts == 1) {
                        ESP_LOGI("DEBUG", "First failed attempt - sending warning");
                        send_security_alert("Failed password attempt detected.");
                    }
                    
                    if (failed_attempts >= MAX_ATTEMPTS) {
                        ESP_LOGI("DEBUG", "Max attempts reached - triggering alarm");
                        trigger_alarm();
                    }
                }
                
                // Clear buffer
                buffer_index = 0;
                memset(password_buffer, 0, sizeof(password_buffer));
                
            } else if (key == '*') {
                // Clear key
                buffer_index = 0;
                memset(password_buffer, 0, sizeof(password_buffer));
                ESP_LOGI("KEYPAD", "Input cleared");
                
            } else if ((key >= '0' && key <= '9') || (key >= 'A' && key <= 'D')) {
                // Number or letter key
                if (buffer_index < sizeof(password_buffer) - 1) {
                    password_buffer[buffer_index++] = key;
                    password_buffer[buffer_index] = '\0';
                    ESP_LOGI("KEYPAD", "Current input: %s", password_buffer);
                } else {
                    ESP_LOGI("KEYPAD", "Input buffer full");
                }
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay between scans
    }
}

/* ===== BUTTON HANDLING ===== */

static void push_btn_cb(void *arg, void *data)
{
    app_light_set_power(!g_power);
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(light_device, ESP_RMAKER_PARAM_POWER),
            esp_rmaker_bool(g_power));
}

/* ===== MAIN DRIVER INITIALIZATION ===== */

void app_driver_init()
{
    ESP_LOGI("DEBUG", "=== APP DRIVER INIT START ===");
    
    // Initialize light
    app_light_init();
    ESP_LOGI("DEBUG", "Light initialized");
    
    // Initialize DHT11 sensor
    init_sensor();
    ESP_LOGI("DEBUG", "Sensor initialized");
    
    // Initialize keypad
    keypad_init();
    ESP_LOGI("DEBUG", "Keypad initialized");
    
    // Start keypad task
    xTaskCreate(keypad_task, "keypad_task", 4096, NULL, 5, NULL);
    ESP_LOGI("DEBUG", "Keypad task created");
    
    // Initialize button (if configured)
    if (BUTTON_GPIO >= 0) {
        button_config_t btn_cfg = {
            .long_press_time = 0,
            .short_press_time = 0,
        };
        button_gpio_config_t gpio_cfg = {
            .gpio_num = BUTTON_GPIO,
            .active_level = BUTTON_ACTIVE_LEVEL,
            .enable_power_save = false,
        };
        button_handle_t btn_handle = NULL;
        if (iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn_handle) == ESP_OK && btn_handle) {
            iot_button_register_cb(btn_handle, BUTTON_SINGLE_CLICK, NULL, push_btn_cb, NULL);
            app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
            ESP_LOGI("DEBUG", "Button initialized");
        }
    }

    ESP_LOGI("DEBUG", "=== APP DRIVER INIT COMPLETE ===");
    ESP_LOGI("APPLICATION_START", "app_driver_initialized");
}