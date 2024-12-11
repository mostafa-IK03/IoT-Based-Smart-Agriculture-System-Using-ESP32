#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Libraries
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

// Drivers
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"

// WiFi Configuration
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define ESP_WIFI_SSID "DESKTOP-K7N460P" // Hotspot from another laptop
#define ESP_WIFI_PASSWORD "123456789"   // Hotspot password
#define ESP_MAXIMUM_RETRY 5             // Maximum number of connection retries

// Defining ADC Channels for Rainfall and Soil Moisture Sensors
#define ADC1_CHANNEL_RAIN ADC1_CHANNEL_4
#define ADC1_CHANNEL_MOISTURE ADC1_CHANNEL_5

// MCP9808 Temperature Sensor I2C Configuration
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_SDA_IO 23
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 1000

// MCP9808 Register Definitions
#define MCP9808_I2CADDR_DEFAULT 0x18
#define MCP9808_REG_AMBIENT_TEMP 0x05
#define MCP9808_REG_MANUF_ID 0x06
#define MCP9808_REG_DEVICE_ID 0x07

// Water Pump Relay GPIO (Active Low)
#define PUMP_RELAY_GPIO GPIO_NUM_17

// I2C
#define ACK_CHECK_EN 0x1
#define ACK_VAL 0x0
#define NACK_VAL 0x1

// MCP9808 temperature sensor struct
typedef struct
{
    uint8_t address;    // I2C address of the temp sensor
    i2c_port_t i2c_num; // I2C port number
} mcp9808_desc_t;

// Handle for temp sensor
typedef void *mcp9808_handle_t;

// Global variables used later on for the logic excution
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "IrrigationSensor";
static int s_retry_num = 0;
static int raw_rain_value = 0;     // ADC value for rain sensor
static int raw_moisture_value = 0; // ADC value for moisture sensor
static float temperature = 0.0;    // Temperature read using MCP9808
static bool pump_on = false;       // Status of the water pump (on or off)

// HTML & CSS Web Page and with Monitoring live values of the sensor readings and with optional manual control for the water pump
const char *html_page =
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta charset=\"UTF-8\">"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
    "<title>Irrigation System Monitoring</title>"
    "<link rel=\"stylesheet\" href=\"https://use.fontawesome.com/releases/v5.15.4/css/all.css\" "
    "integrity=\"sha384-DyZ88mC6Up2uqS4h/Kd+Z3mZZm7R+eKfnPnTfYROGv0KUw3tE4mc0bM+0Jo0Rb0c\" crossorigin=\"anonymous\">"
    "<style>"
    "body {"
    "font-family: 'Arial', sans-serif;"
    "background-color: #f0f2f5;"
    "margin: 0;"
    "padding: 0;"
    "color: #333;"
    "}"
    "header {"
    "background-color: #4CAF50;"
    "padding: 20px 0;"
    "text-align: center;"
    "color: white;"
    "box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);"
    "}"
    "header h1 {"
    "margin: 0;"
    "font-size: 2.5em;"
    "}"
    "header p {"
    "margin: 5px 0 0 0;"
    "font-size: 1.2em;"
    "}"
    ".container {"
    "display: grid;"
    "grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));"
    "gap: 20px;"
    "padding: 40px 20px;"
    "max-width: 1200px;"
    "margin: 0 auto;"
    "}"
    ".card {"
    "background-color: white;"
    "border-radius: 10px;"
    "box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);"
    "padding: 30px;"
    "text-align: center;"
    "transition: transform 0.2s, box-shadow 0.2s;"
    "}"
    ".card:hover {"
    "transform: translateY(-5px);"
    "box-shadow: 0 8px 16px rgba(0, 0, 0, 0.2);"
    "}"
    ".card i {"
    "font-size: 3em;"
    "color: #4CAF50;"
    "margin-bottom: 15px;"
    "}"
    ".card h3 {"
    "margin: 15px 0;"
    "font-size: 1.5em;"
    "color: #333;"
    "display: flex;"
    "align-items: center;"
    "justify-content: center;"
    "gap: 10px;"
    "}"
    ".card p {"
    "font-size: 1.2em;"
    "color: #666;"
    "}"
    ".button {"
    "padding: 10px 20px;"
    "margin: 5px;"
    "font-size: 1em;"
    "border: none;"
    "border-radius: 5px;"
    "cursor: pointer;"
    "color: white;"
    "}"
    ".button-on {"
    "background-color: #28a745;"
    "}"
    ".button-off {"
    "background-color: #dc3545;"
    "}"
    ".footer {"
    "background-color: #4CAF50;"
    "color: white;"
    "text-align: center;"
    "padding: 15px 0;"
    "position: fixed;"
    "width: 100%;"
    "bottom: 0;"
    "box-shadow: 0 -2px 4px rgba(0, 0, 0, 0.1);"
    "}"
    "@media (max-width: 600px) {"
    ".card {"
    "padding: 20px;"
    "}"
    "header h1 {"
    "font-size: 2em;"
    "}"
    "header p {"
    "font-size: 1em;"
    "}"
    "}"
    "</style>"
    "</head>"
    "<body>"
    "<header>"
    "<h1>Irrigation System Monitoring</h1>"
    "<p>Keep your garden well-watered and healthy!</p>"
    "</header>"
    "<div class='container'>"
    "<div class='card'>"
    "<i class=\"fas fa-tint\"></i>"
    "<h3>🌧️ Rainfall Sensor</h3>"
    "<p>Current Rainfall Sensor Percentage: <span id='rainValue'>Loading...</span></p>"
    "</div>"
    "<div class='card'>"
    "<i class=\"fas fa-seedling\"></i>"
    "<h3>🌱 Soil Moisture Sensor</h3>"
    "<p>Current Moisture Sensor Percentage: <span id='moistureValue'>Loading...</span></p>"
    "</div>"
    "<div class='card'>"
    "<i class=\"fas fa-thermometer-half\"></i>"
    "<h3>🌡️ Temperature Sensor</h3>"
    "<p>Current Temperature: <span id='tempValue'>Loading...</span> °C</p>"
    "</div>"
    "<div class='card'>"
    "<i class=\"fas fa-water\"></i>"
    "<h3>💧 Water Pump</h3>"
    "<p>Status: <span id='pumpStatus'>Loading...</span></p>"
    "<button class=\"button button-on\" onclick=\"controlPump('on')\">Turn On</button>"
    "<button class=\"button button-off\" onclick=\"controlPump('off')\">Turn Off</button>"
    "</div>"
    "</div>"
    "<div class='footer'>"
    "<p>&copy; 2024 Irrigation Systems. All rights reserved.</p>"
    "</div>"
    "<script>"
    "function fetchSensorData() {"
    "fetch('/sensor_data')"
    ".then(response => response.json())"
    ".then(data => {"
    "document.getElementById('rainValue').innerText = data.rain;"
    "document.getElementById('moistureValue').innerText = data.moisture;"
    "document.getElementById('tempValue').innerText = data.temperature.toFixed(2);"
    "document.getElementById('pumpStatus').innerText = data.pump_status ? 'ON' : 'OFF';"
    "})"
    ".catch(error => {"
    "console.error('Error fetching sensor data:', error);"
    "document.getElementById('rainValue').innerText = 'Error';"
    "document.getElementById('moistureValue').innerText = 'Error';"
    "document.getElementById('tempValue').innerText = 'Error';"
    "document.getElementById('pumpStatus').innerText = 'Error';"
    "});"
    "}"
    "function controlPump(action) {"
    "let endpoint = '/pump_' + action;"
    "fetch(endpoint)"
    ".then(response => response.json())"
    ".then(data => {"
    "if(data.success){"
    "document.getElementById('pumpStatus').innerText = data.pump_status ? 'ON' : 'OFF';"
    "} else {"
    "alert('Failed to control pump.');"
    "}"
    "})"
    ".catch(error => {"
    "console.error('Error controlling pump:', error);"
    "alert('Error controlling pump.');"
    "});"
    "}"
    "setInterval(fetchSensorData, 500);  // Fetch new data every 0.5 seconds"
    "window.onload = fetchSensorData;      // Fetch data on page load"
    "</script>"
    "</body>"
    "</html>";

// Functions for temp sensor
static esp_err_t i2c_master_init(void);
static esp_err_t mcp9808_read16(const mcp9808_desc_t *desc, uint8_t reg, uint16_t *res);
static esp_err_t mcp9808_init(uint8_t address, i2c_port_t i2c_num, mcp9808_handle_t *handle, uint16_t *manuf_id, uint16_t *dev_id);
static esp_err_t mcp9808_ambient_temp(mcp9808_handle_t handle, float *res);
static void mcp9808_delete(mcp9808_handle_t handle);

// I2C master interface initialize
static esp_err_t i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver installation failed");
    }

    return ret;
}


static esp_err_t mcp9808_read16(const mcp9808_desc_t *desc, uint8_t reg, uint16_t *res)
{
    uint8_t upper, lower;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (desc->address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (desc->address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &upper, ACK_VAL);
    i2c_master_read_byte(cmd, &lower, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(desc->i2c_num, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read failed with error: %d", ret);
        return ESP_FAIL;
    }

    *res = ((uint16_t)upper << 8) | lower;
    return ESP_OK;
}

// Initialize temperature sensor
static esp_err_t mcp9808_init(uint8_t address, i2c_port_t i2c_num, mcp9808_handle_t *handle, uint16_t *manuf_id, uint16_t *dev_id)
{
    mcp9808_desc_t *desc = (mcp9808_desc_t *)calloc(1, sizeof(mcp9808_desc_t));
    if (!desc)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for MCP9808 descriptor");
        return ESP_ERR_NO_MEM;
    }
    desc->address = address;
    desc->i2c_num = i2c_num;
    if (mcp9808_read16(desc, MCP9808_REG_MANUF_ID, manuf_id) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read Manufacturer ID");
        free(desc);
        return ESP_FAIL;
    }
    if (mcp9808_read16(desc, MCP9808_REG_DEVICE_ID, dev_id) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read Device ID");
        free(desc);
        return ESP_FAIL;
    }
    *handle = (mcp9808_handle_t)desc;
    return ESP_OK;
}

// Read ambient temp
static esp_err_t mcp9808_ambient_temp(mcp9808_handle_t handle, float *res)
{
    mcp9808_desc_t *desc = (mcp9808_desc_t *)handle;
    uint16_t val;

    if (mcp9808_read16(desc, MCP9808_REG_AMBIENT_TEMP, &val) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read ambient temperature");
        return ESP_FAIL;
    }

    if (val == 0xFFFF)
    { 
        ESP_LOGE(TAG, "Invalid temperature read");
        return ESP_FAIL;
    }

    // Convert raw value to degree Celsius
    float temp = (float)(val & 0x0FFF);
    temp /= 16.0f;
    if (val & 0x1000)
    {
        temp -= 256.0f;
    }

    *res = temp;
    return ESP_OK;
}

// Delete temp sensor handle and free allocated memory
static void mcp9808_delete(mcp9808_handle_t handle)
{
    if (handle == NULL)
        return;
    free(handle);
}

// Initialize ADC for rainfall and soil moisture sensors
void init_adc()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_RAIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_MOISTURE, ADC_ATTEN_DB_11);
}

// Read the raw ADC value from the rainfall sensor and converts it to percentage (0% is no rain)
int read_rain_sensor()
{
    const float max_value = 4095.0f;
    int adc_value = adc1_get_raw(ADC1_CHANNEL_RAIN);
    if (adc_value == -1)
    {
        ESP_LOGE(TAG, "Failed to read ADC1_CHANNEL_RAIN");
        return -1; 
    }
    float rain_percentage = (1.0f - ((float)adc_value / max_value)) * 100.0f;
    return (int)rain_percentage;
}

// Read the raw ADC value from the soil moisture sensor and converts it to percentage (100% is maximum dryness)
int read_moisture_sensor()
{
    const float max_value = 4095.0f;
    int adc_value = adc1_get_raw(ADC1_CHANNEL_MOISTURE);
    if (adc_value == -1)
    {
        ESP_LOGE(TAG, "Failed to read ADC1_CHANNEL_MOISTURE");
        return -1; 
    }
    float soil_percentage = ((float)adc_value / max_value) * 100.0f;
    return (int)soil_percentage;
}

// HTTP GET handlers for main HTML page
esp_err_t get_req_handler(httpd_req_t *req)
{
    return httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
}

// Raw sensor data in JSON format
esp_err_t sensor_data_handler(httpd_req_t *req)
{
    raw_rain_value = read_rain_sensor();         
    raw_moisture_value = read_moisture_sensor(); 

    // JSON response
    char response_data[128];
    snprintf(response_data, sizeof(response_data),
             "{\"rain\": %d, \"moisture\": %d, \"temperature\": %.2f, \"pump_status\": %d}",
             raw_rain_value, raw_moisture_value, temperature, pump_on ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
}

// Turn the water pump ON (Active Low)
esp_err_t pump_on_handler(httpd_req_t *req)
{
    gpio_set_level(PUMP_RELAY_GPIO, 0);
    pump_on = true;

    // JSON response
    char response_data[64];
    snprintf(response_data, sizeof(response_data), "{\"success\": 1, \"pump_status\": %d}", pump_on ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
}

// Turn the water pump OFF (Active Low)
esp_err_t pump_off_handler(httpd_req_t *req)
{
    gpio_set_level(PUMP_RELAY_GPIO, 1); 
    pump_on = false;

    // JSON response
    char response_data[64];
    snprintf(response_data, sizeof(response_data), "{\"success\": 1, \"pump_status\": %d}", pump_on ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
}

// URI Handlers for HTTP Server
// Handler for the root URL
httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_req_handler,
    .user_ctx = NULL};

// /sensor_data endpoint
httpd_uri_t uri_sensor_data = {
    .uri = "/sensor_data",
    .method = HTTP_GET,
    .handler = sensor_data_handler,
    .user_ctx = NULL};

// /pump_on endpoint
httpd_uri_t uri_pump_on = {
    .uri = "/pump_on",
    .method = HTTP_GET,
    .handler = pump_on_handler,
    .user_ctx = NULL};

// /pump_off endpoint
httpd_uri_t uri_pump_off = {
    .uri = "/pump_off",
    .method = HTTP_GET,
    .handler = pump_off_handler,
    .user_ctx = NULL};

// Start the HTTP server and registers URI handlers
httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting web server...");

    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Web server started.");
        httpd_register_uri_handler(server, &uri_get);         // Main page
        httpd_register_uri_handler(server, &uri_sensor_data); // Sensor data
        httpd_register_uri_handler(server, &uri_pump_on);     // Pump ON
        httpd_register_uri_handler(server, &uri_pump_off);    // Pump OFF
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start web server");
    }

    return server;
}

// Wifi event handler
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to the AP...");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connection to AP failed.");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Connect to Wifi
void connect_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); 

    ESP_LOGI(TAG, "Wi-Fi initialization finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected to SSID:%s", ESP_WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", ESP_WIFI_SSID);
    }

    vEventGroupDelete(s_wifi_event_group);
}

// Reading temp value

void temperature_task(void *pvParameter)
{
    mcp9808_handle_t mcp;
    uint16_t manuf_id, device_id;

    if (i2c_master_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2C master");
        vTaskDelete(NULL);
    }

    if (mcp9808_init(MCP9808_I2CADDR_DEFAULT, I2C_MASTER_NUM, &mcp, &manuf_id, &device_id) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MCP9808");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "MCP9808 Manufacturer ID: 0x%X, Device ID: 0x%X", manuf_id, device_id);

    while (1)
    {
        float temp;
        if (mcp9808_ambient_temp(mcp, &temp) == ESP_OK)
        {
            temperature = temp; 
        }
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    mcp9808_delete(mcp);
    vTaskDelete(NULL);
}

// Rekay for water pump
void init_pump_relay()
{
    gpio_reset_pin(PUMP_RELAY_GPIO);
    gpio_set_direction(PUMP_RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_RELAY_GPIO, 1);
    pump_on = false;
}

// Code execution logic
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(ret);
    }
    connect_wifi();
    init_adc();
    init_pump_relay();
    xTaskCreate(&temperature_task, "temperature_task", 4096, NULL, 5, NULL);
    start_webserver();
    ESP_LOGI(TAG, "Irrigation System Monitoring is running...");

    TickType_t lastDecisionTime = xTaskGetTickCount();
    TickType_t currentTime;

    while (1)
    {
        raw_rain_value = read_rain_sensor();         // Rain %
        raw_moisture_value = read_moisture_sensor(); // Dryness %

        // Print readings in terminal 
        printf("Rainfall Sensor: %d%%\n", raw_rain_value);
        printf("Moisture Sensor: %d%%\n", raw_moisture_value);
        printf("Temperature: %.2f°C\n", temperature);
        printf("----------------------------------------------------------------\n");

        // Check the watering decision (every 1 minute for facilitaing testing however it should be every 10 minutes) 
        currentTime = xTaskGetTickCount();
        if ((currentTime - lastDecisionTime) * portTICK_PERIOD_MS >= 60000)
        {
            bool turn_pump_on = true;
            char reason[256] = "";

            // check for rain
            if (raw_rain_value >= 20)
            {
                turn_pump_on = false;
                strcat(reason, "Enough rain. ");
            }

            // check dryness of the soil
            if (raw_moisture_value <= 70)
            {
                turn_pump_on = false;
                strcat(reason, "Soil not dry enough. ");
            }

            // check temperature
            if (temperature >= 27.0f)
            {
                turn_pump_on = false;
                strcat(reason, "Temperature not ideal. ");
            }

            // ON/OFF pump
            if (turn_pump_on)
            {
                gpio_set_level(PUMP_RELAY_GPIO, 0); 
                pump_on = true;
                printf("Pump turned ON\n");
            }
            else
            {
                gpio_set_level(PUMP_RELAY_GPIO, 1);
                pump_on = false;
                printf("Pump remains OFF (%s)\n", reason);
            }
            lastDecisionTime = currentTime;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}