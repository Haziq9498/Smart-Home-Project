#pragma once
#include <stdint.h>
#include <stdbool.h>

// Light defaults
#define DEFAULT_POWER       false  // Start with LED OFF
#define DEFAULT_HUE         180
#define DEFAULT_SATURATION  100
#define DEFAULT_BRIGHTNESS  100    // Full brightness when on

// Sensor defaults
#define DEFAULT_TEMPERATURE 25.0
#define DEFAULT_HUMIDITY    50.0
#define REPORTING_PERIOD    10 /* Seconds */

// LED GPIO
#define LED_GPIO            3
#define ALARM_LED_GPIO      21    // For blinking alarm

// Keypad GPIO pins
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

// Row pins (output)
#define ROW1_GPIO 4
#define ROW2_GPIO 5
#define ROW3_GPIO 6
#define ROW4_GPIO 7

// Column pins (input)
#define COL1_GPIO 8
#define COL2_GPIO 9
#define COL3_GPIO 10
#define COL4_GPIO 20

// Password settings
#define CORRECT_PASSWORD "1234"   // Change this to your desired password
#define MAX_ATTEMPTS 3

extern esp_rmaker_device_t *light_device;
extern esp_rmaker_device_t *temp_sensor_device;
extern esp_rmaker_device_t *alert_device;

void app_driver_init(void);
esp_err_t app_light_set_power(bool power);
esp_err_t app_light_set_brightness(uint16_t brightness);
esp_err_t app_light_set_hue(uint16_t hue);
esp_err_t app_light_set_saturation(uint16_t saturation);

// Sensor functions  
float app_get_current_temperature(void);
float app_get_current_humidity(void);

// Keypad functions
void keypad_init(void);
char keypad_get_key(void);
bool check_password(const char *input);
void trigger_alarm(void);
void stop_alarm(void);

// Notification function
void send_security_alert(const char *message);