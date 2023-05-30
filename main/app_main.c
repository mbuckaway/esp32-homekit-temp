/*
 * MIT License
 * 
 * HomeKit temp Project with BME 680 support
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <esp_event.h>
#include <esp_log.h>
#include <math.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>


#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <dht.h>

//libraries for BME68x support
#include "driver/i2c.h"
#include "bsec_integration.h"
#include <nvs_flash.h>
#include <nvs.h>
#include "bsec_iaq.h"
#include "esp_timer.h"

// Library for HD44780 Screen Support
#include <hd44780.h>


#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_FREQUENCY   100000
#define I2C_GPIO_SDA    GPIO_NUM_21
#define I2C_GPIO_SCL    GPIO_NUM_22
#define ACTIVE_I2C      I2C_NUM_1

#define SENSOR_IN_USE   1 /*!< set to 1 for BME68X and 2 for DHT */

static const char* sensor_binary = "sensor_blob";


static const dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;

static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

    static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration1;

#if CONFIG_IDF_TARGET_ESP32
static const adc_bitwidth_t width = ADC_BITWIDTH_12;
#elif CONFIG_IDF_TARGET_ESP32S2
// 13bit ADC will cause issues with battery voltage formula
static const adc_bitwidth_t width = ADC_BITWIDTH_12;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;


static int adc_raw;
static int voltage;

/*  Required for server verification during OTA, PEM format as string  */
char server_cert[] = {};

static const char *TAG = "HAP";

static const uint16_t temp_TASK_PRIORITY = 1;
static const uint16_t temp_TASK_STACKSIZE = 4 * 1024;
static const char *temp_TASK_NAME = "hap_garage";

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
static const uint16_t RESET_NETWORK_BUTTON_TIMEOUT = 3;

/* Reset to factory if button is pressed and held for more than 10 seconds */
 static const uint16_t RESET_TO_FACTORY_BUTTON_TIMEOUT = 10;

/* The button "Boot" will be used as the Reset button for the example */
static const uint16_t RESET_GPIO = GPIO_NUM_0;

// Stack size of the bSEC processing loop
static const uint16_t BSEC_STACK_SIZE = 10 * 1024;


/*!
 * @brief           Write operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 * param[in]        intf_ptr        interface pointer
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    // ...
    // Please insert system specific function to write to the bus where BME68x is connected
    // ...
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    assert(data_len > 0 && reg_data_ptr != NULL); // Safeguarding the assumptions
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x76 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data_ptr, data_len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ACTIVE_I2C, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    // ESP_OK matches with the function success code (0)
    return (int8_t)ret;
    return 0;
}
/*!
 * @brief           Read operation in either Wire or SPI
 *
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 * param[in]        intf_ptr        interface pointer
 * 
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
    // ...
    // Please insert system specific function to read from bus where BME68x is connected
    // ...
    // ...
    // Please insert system specific function to read from bus where BME680 is connected
    // ...
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    assert(data_len > 0 && reg_data_ptr != NULL); // Safeguarding the assumptions
    // Feeding the command in
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x76 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    //bme680_sleep(150);
    // Reading data back
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x76 << 1) | I2C_MASTER_READ, true);
    if (data_len > 1) {
        i2c_master_read(cmd, reg_data_ptr, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data_ptr + data_len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ACTIVE_I2C, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    // ESP_OK matches with the function success code (0)
    return (int8_t)ret;
    //return 0;
}





/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_us     Time in microseconds
 * @param[in]       intf_ptr Pointer to the interface descriptor
 * 
 * @return          none
 */
static void bme680_sleep(uint32_t t_us, void *intf_ptr)
{
    // ...
    // Please insert system specific function sleep or delay for t_ms milliseconds
    // ...
    vTaskDelay(pdMS_TO_TICKS(t_us/10));
}
/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp               time in nanoseconds
 * @param[in]       iaq                     Indoor-air-quality estimate [0-500]
 * @param[in]       iaq_accuracy            Indoor-air-quality accuracy
 * @param[in]       siaq                    Unscaled indoor-air-quality estimate
 * @param[in]       siaq_accuracy           Unscaled indoor-air-quality accuracy
 * @param[in]       compensateTemperature   Sensor heat compensated temperature [degrees Celsius]
 * @param[in]       compensateHumidity      Sensor heat compensated humidity [%]
 * @param[in]       raw_pressure            Pressure sensor signal [Pa]
 * @param[in]       raw_temp                Temperature sensor signal [degrees Celsius]
 * @param[in]       raw_humidity            Relative humidity sensor signal [%]
 * @param[in]       raw_gas                 Gas sensor signal [Ohm]
 * @param[in]       co2                     CO2 equivalent estimate [ppm]
 * @param[in]       bsec_status             value returned by the bsec_do_steps() call
 *
 * @return          none
 */
static float BME68xtemperature = 0.0;
static float BME68xhumidity = 0.0;
static float BME68xsIAQ = 0.0;
static float BME68xC02 = 0.0;
static float BME68xbVOC = 0.0;

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float siaq, uint8_t siaq_accuracy, float compensateTemperature, float compensateHumidity,
     float raw_pressure, float raw_temp, float raw_humidity, float raw_gas, float co2, float bVOC, bsec_library_return_t bsec_status) {
    // ...
    // Please insert system specific code to further process or display the BSEC outputs
    // ...

    BME68xtemperature = compensateTemperature;
    BME68xhumidity = compensateHumidity;
    BME68xsIAQ = siaq;
    BME68xC02 = co2;
    BME68xbVOC = bVOC;
    
    ESP_LOGI("BME 680", "[timestamp: %"PRId64"] [IAQ reading: %f] [IAQ Accuracy: %d] [SIAQ reading: %f] [sIAQ Accuracy: %d] [Compensated Temperature: %f] [Compensated Humidity: %f] [raw_pressure: %f] [raw_temp: %f] [raw_humidity: %f] [raw_gas: %f] [co2_equivalent: %f] [bVOC: %f] [bsec_status: %d]\n", timestamp, iaq, iaq_accuracy, siaq, siaq_accuracy, compensateTemperature, compensateHumidity, raw_pressure, raw_temp, raw_humidity, raw_gas, co2, bVOC, bsec_status);
}

float bm68xtempReturn(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "temp sensor received read from %s", hap_req_get_ctrl_id(read_priv));
    }

    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_TEMPERATURE))
    {
        hap_val_t new_val;
        new_val.f = BME68xtemperature;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"temp status updated to %0.01f", new_val.f);
    }

    return HAP_SUCCESS;
}

float bm68xhumidReturn(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "temp sensor received read from %s", hap_req_get_ctrl_id(read_priv));
    }

    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) 
    {
        hap_val_t new_val;
        new_val.f = BME68xhumidity;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"humidity status updated to %0.01f%%", new_val.f);
    }

    return HAP_SUCCESS;
}

int bm68xIAQReturn(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "temp sensor received read from %s", hap_req_get_ctrl_id(read_priv));
    }

    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_AIR_QUALITY)) 
    {
        hap_val_t new_val;
        if (round(BME68xsIAQ) <= 50) {
            new_val.u = 1;
        } else if (round(BME68xsIAQ) <= 100) {
            new_val.u = 2;
        }
        else if (round(BME68xsIAQ) <= 150) {
            new_val.u = 3;
        }
        else if (round(BME68xsIAQ) <= 200) {
            new_val.u = 4;
        }
        else {
            new_val.u = 5;
        }
        
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG, "IAQ status updated to %"PRIu32, new_val.u);
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CARBON_DIOXIDE_LEVEL)) 
    {
        hap_val_t new_val;
        new_val.f = BME68xC02;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG, "CO2 PPM value updated to %0.01f", new_val.f);
    }   
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_VOC_DENSITY)) 
    {
        hap_val_t new_val;
        new_val.f = BME68xbVOC; 
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG, "VOC value updated to %0.01f", new_val.f);
    }       

    return HAP_SUCCESS;
}


/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */

//uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
uint32_t state_load(uint8_t *state_buffer, size_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("state", NVS_READONLY, &my_handle);
    //ESP_ERROR_CHECK( err );

    err = nvs_get_blob(my_handle, sensor_binary, state_buffer, &n_buffer);
    // We close this anyway even if the operation didn't succeed.
    nvs_close(my_handle);
    if (err == ESP_OK){
        return n_buffer;
    }
    ESP_LOGW(TAG, "loading sensor binary blob failed with code %d", err);
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("state", NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK( err );

    err = nvs_set_blob(my_handle, sensor_binary, state_buffer, length);
    ESP_ERROR_CHECK( err );
    err = nvs_commit(my_handle);
    ESP_ERROR_CHECK(err);
    nvs_close(my_handle);
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    ESP_LOGI(TAG, "Loading configuration: buffer-size %" PRIu32 "  config size %d", n_buffer, sizeof(bsec_config_iaq));
    assert(n_buffer >= sizeof(bsec_config_iaq));
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));

    return sizeof(bsec_config_iaq);
}


static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_GPIO_SDA,
        .scl_io_num = I2C_GPIO_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };
    i2c_param_config(ACTIVE_I2C, &conf);
    return i2c_driver_install(ACTIVE_I2C, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


/*!
 * @brief       Main function which configures BSEC library and then reads and processes the data from sensor based
 *              on timer ticks
 *
 * @return      result of the processing
 */
int initialize_sensor()
{
    return_values_init ret;
    esp_err_t err = i2c_master_init();
    ESP_ERROR_CHECK( err );
    
    struct bme68x_dev bme_dev;
	memset(&bme_dev,0,sizeof(bme_dev)); 
 
    ESP_LOGI(TAG, "I2C initialized");
    /* Call to the function which initializes the BSEC library BSEC_SAMPLE_RATE_SCAN
     * Switch on low-power mode and provide no temperature offset BSEC_SAMPLE_RATE_LP */ //BSEC_SAMPLE_RATE_ULP
    //void bsec_iot_init(float sample_rate, float temperature_offset, bme68x_write_fptr_t bus_write, bme68x_read_fptr_t bus_read, sleep_fct sleep_n, state_load_fct state_load, config_load_fct config_load, struct bme68x_dev dev);
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, bme680_sleep, state_load, config_load, bme_dev);
    if (ret.bme68x_status)
    {
        /* Could not initialize BME680 */
        ESP_LOGE(TAG, "initializing BME680 failed %d", ret.bme68x_status);
        return (int)ret.bme68x_status;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        ESP_LOGE(TAG, "initializing BSEC failed %d", ret.bsec_status);
        return (int)ret.bsec_status;
    }
    return 0;
}






/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

static uint8_t get_battery_level(void)
{
    float percentage = 100;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CONFIG_BATTERY_ADC_CHANNEL, &adc_raw));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, CONFIG_BATTERY_ADC_CHANNEL, adc_raw);
    if (do_calibration1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, CONFIG_BATTERY_ADC_CHANNEL, voltage);
    }


    //I don't use battery, so
    voltage = 5000;

    // Voltage is half because of the divide resistors. ADC max's out at 2.8V
    voltage*=2;
    float voltage_f = (float)(voltage) / 1000.0;
    ESP_LOGI(TAG, "Battery Level Raw: %d\tVoltage: %dmV (%0.02fV)", adc_raw, voltage, voltage_f);
    percentage = (2808.3808 * pow(voltage_f, 4)) - (43560.9157 * pow(voltage_f, 3)) + (252848.5888 * pow(voltage_f, 2)) - (650767.4615 * voltage_f) + 626532.5703;
    if (voltage_f > 4.19) percentage = 100.0;
    else if (voltage_f <= 3.50) percentage = 0.0;
    ESP_LOGI(TAG, "Battery Level %0.01f%%", percentage);
    return percentage;
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int temp_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void temp_hap_event_handler(void* arg, esp_event_base_t event_base, int32_t event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED :
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED :
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED :
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED :
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                        (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED :
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED :
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING : {
            char *reason = (char *)data;
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",  reason ? reason : "null");
            break;
        }
        default:
            /* Silently ignore unknown events */
            break;
    }
}

/* 
 * In an actual accessory, this should read from hardware.
 * Read routines are generally not required as the value is available with th HAP core
 * when it is updated from write routines. For external triggers (like fan switched on/off
 * using physical button), accessories should explicitly call hap_char_update_val()
 * instead of waiting for a read request.
 */
static int temp_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    static float temperature = 0.0;
    static float humidity = 0.0;

    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "temp sensor received read from %s", hap_req_get_ctrl_id(read_priv));
    }

    // Only update the sensor info on a temperature read since they are read one after another
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_TEMPERATURE)) 
    {
        hap_val_t new_val;

        if (dht_read_float_data(sensor_type, CONFIG_GPIO_OUTPUT_IO_DHT22, &humidity, &temperature) == ESP_OK)
            ESP_LOGI(TAG, "Read Temp: %0.01fC Humidity: %0.01f%% ", temperature, humidity);
        else
            ESP_LOGE(TAG, "Could not read data from DHT sensor on GPIO %d", CONFIG_GPIO_OUTPUT_IO_DHT22);

        new_val.f = temperature;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"temp status updated to %0.01f", new_val.f);
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) 
    {
        hap_val_t new_val;
        new_val.f = humidity;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG,"humidity status updated to %0.01f%%", new_val.f);
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_BATTERY_LEVEL)) 
    {
        hap_val_t new_val;
        new_val.i = get_battery_level();
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG, "battery level updated to %d", new_val.i);
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_STATUS_LOW_BATTERY)) 
    {
        hap_val_t new_val;
        uint8_t battery_level = get_battery_level();
        new_val.i = battery_level<25?1:0;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG, "battery low level updated to %d (%s)", new_val.i, new_val.i?"low battery":"battery ok");
    }
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CHARGING_STATE)) 
    {
        hap_val_t new_val;
        // Figure out if we are charging, and update this info.
        new_val.i = 0;
        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
        ESP_LOGI(TAG, "battery charging state updated");
    }
    return HAP_SUCCESS;
}

/**
 * @brief Main Thread to handle setting up the service and accessories for the GarageDoor
 */
static void temp_thread_entry(void *p)
{
    hap_acc_t *tempaccessory = NULL;
    hap_serv_t *tempservice = NULL;
    hap_serv_t *humidityservice = NULL;
    hap_serv_t *aqiService = NULL;
    hap_serv_t *battery_service = NULL;
    hap_char_t *VOCService = NULL;
    hap_char_t *co2Service = NULL;
    int adc_gpio_num = 0;
    float temperature = 0.0;
    float humidity = 0.0;
    int aqiReading = 0;
    float vocReading = 0;
    float co2Reading = 0;
    /*BME68xC02;BME68xbVOC;
     * Configure the ADC for reading battery level
     */

    ESP_LOGI(TAG, "configuring ADC for battery level");
    // new implementation
    //-------------ADC1 Init---------------//

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = width,
        .atten = atten,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CONFIG_BATTERY_ADC_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, atten, &adc1_cali_handle);
    ESP_LOGI(TAG, "Battery Level ADC running on GPIO %d", adc_gpio_num);

    /* Configure HomeKit core to make the Accessory name (and thus the WAC SSID) unique,
     * instead of the default configuration wherein only the WAC SSID is made unique.
     */
    ESP_LOGI(TAG, "configuring HAP");
    hap_cfg_t hap_cfg;
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    ESP_LOGI(TAG, "initializing HAP");
    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Esp-Term",
        .manufacturer = "Espressif",
        .model = "EspTermp01",
        .serial_num = "001122334455",
        .fw_rev = "1.0.0",
        .hw_rev =  (char*)esp_get_idf_version(),
        .pv = "1.0.0",
        .identify_routine = temp_identify,
        .cid = HAP_CID_SENSOR,
    };
    ESP_LOGI(TAG, "Creating temperature accessory...");
    /* Create accessory object */
    tempaccessory = hap_acc_create(&cfg);

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(tempaccessory, product_data, sizeof(product_data));
    if (SENSOR_IN_USE == 1) {
        temperature = BME68xtemperature;
        humidity = BME68xhumidity;
    } else if (SENSOR_IN_USE == 2) {
        if (dht_read_float_data(sensor_type, CONFIG_GPIO_OUTPUT_IO_DHT22, &humidity, &temperature) == ESP_OK)
            ESP_LOGI(TAG, "Sensor Read: Temperature: %0.01f Humidity: %0.01f", temperature, humidity);
        else
            ESP_LOGE(TAG, "Could not read data from sensor on GPIO %d\n", CONFIG_GPIO_OUTPUT_IO_DHT22);
    }

    
    ESP_LOGI(TAG, "Creating temperature service (current temp: %0.01fC)", temperature);
    /* Create the temp Service. Include the "name" since this is a user visible service  */
    tempservice = hap_serv_temperature_sensor_create(temperature);
    hap_serv_add_char(tempservice, hap_char_name_create("ESP Temperature Sensor"));
    /* Set the read callback for the service (optional) */
    if (SENSOR_IN_USE == 1) {
        hap_serv_set_read_cb(tempservice, bm68xtempReturn);
    } else if (SENSOR_IN_USE == 2) {
        hap_serv_set_read_cb(tempservice, temp_read);
    }
    /* Add the temp Service to the Accessory Object */
    hap_acc_add_serv(tempaccessory, tempservice);


    ESP_LOGI(TAG, "Creating humidity service (current humidity: %0.01f%%)", humidity);
    /* Create the temp Service. Include the "name" since this is a user visible service  */
    humidityservice = hap_serv_humidity_sensor_create(humidity);
    hap_serv_add_char(humidityservice, hap_char_name_create("ESP Humidity Sensor"));
    /* Set the read callback for the service (optional) */
    if (SENSOR_IN_USE == 1) {
        hap_serv_set_read_cb(humidityservice, bm68xhumidReturn);
    } else if (SENSOR_IN_USE == 2) {
        hap_serv_set_read_cb(humidityservice, temp_read);
    }
    /* Add the humidity Service to the Accessory Object */
    hap_acc_add_serv(tempaccessory, humidityservice);

    // DHT line of sensors do not support AQI 
    if (SENSOR_IN_USE == 1) {
        ESP_LOGI(TAG, "Creating AQI service (current humidity: %d)", aqiReading);
        /* Create the aqi Service. Include the "name" since this is a user visible service  */
        aqiService = hap_serv_air_quality_sensor_create(aqiReading);
        VOCService = hap_char_voc_density_create(vocReading);
        co2Service = hap_char_carbon_dioxide_level_create(co2Reading);
        hap_serv_add_char(aqiService, hap_char_name_create("ESP AQI Sensor"));
        hap_serv_add_char(aqiService, co2Service);
        hap_serv_add_char(aqiService, VOCService);
        /* Set the read callback for the service (optional) */
        hap_serv_set_read_cb(aqiService, bm68xIAQReturn);
        /* Add the AQI Service to the Accessory Object */
        hap_acc_add_serv(tempaccessory, aqiService);
    }    

    
    u_int8_t battery_level = get_battery_level();
    ESP_LOGI(TAG, "Creating battery service (current battery level: %d)", battery_level);
    // Create the CloseIf switch
    battery_service = hap_serv_battery_service_create(battery_level, 0, (battery_level<25)?1:0);
    hap_serv_add_char(battery_service, hap_char_name_create("ESP Battery Level"));
    hap_serv_set_read_cb(battery_service, temp_read);
    hap_acc_add_serv(tempaccessory, battery_service);


#if 0
    /* Create the Firmware Upgrade HomeKit Custom Service.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    service = hap_serv_fw_upgrade_create(&ota_config);
    /* Add the service to the Accessory Object */
    hap_acc_add_serv(accessory, service);
#endif

    /* Add the Accessory to the HomeKit Database */
    ESP_LOGI(TAG, "Adding Temperature Accessory...");
    hap_add_accessory(tempaccessory);

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    ESP_LOGI(TAG, "Register reset GPIO (reset button) on pin %d", RESET_GPIO);
    reset_key_init(RESET_GPIO);

    /* Register an event handler for HomeKit specific events */
    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &temp_hap_event_handler, NULL);

    /* Query the controller count (just for information) */
    ESP_LOGI(TAG, "Accessory is paired with %d controllers", hap_get_paired_controller_count());


    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_HOMEKIT_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_HOMEKIT_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_HOMEKIT_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_HOMEKIT_SETUP_CODE, CONFIG_HOMEKIT_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_HOMEKIT_SETUP_CODE, CONFIG_HOMEKIT_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* mfi is not supported */
    hap_enable_mfi_auth(HAP_MFI_AUTH_NONE);

    ESP_LOGI(TAG, "Starting WIFI...");
    /* Initialize Wi-Fi */
    app_wifi_init();

    /* After all the initializations are done, start the HAP core */
    ESP_LOGI(TAG, "Starting HAP...");
    hap_start();

    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);
    
    ESP_LOGI(TAG, "HAP initialization complete.");

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);
}



static void bSECReadTask(void *p)
{
    ESP_LOGI(TAG, "entered BME 680 sensor read loop");
    initialize_sensor();
    bsec_iot_loop(bme680_sleep, esp_timer_get_time, output_ready, state_save, 10000);
    /* The task won't end. It should stay in the bsec_iot_loop. */
    vTaskDelete(NULL);
}


// ADC calibration init function declaration
// ripped from https://github.com/espressif/esp-idf/blob/release/v5.0/examples/peripherals/adc/oneshot_read/main/oneshot_read_main.c
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = width,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = width,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup...");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_LOGI(TAG, "[APP] Creating main thread...");

    xTaskCreate(temp_thread_entry, temp_TASK_NAME, temp_TASK_STACKSIZE, NULL, temp_TASK_PRIORITY, NULL);

    // task to poll bme680 sensor and initialize it 
    if (SENSOR_IN_USE == 1) {
        xTaskCreate(bSECReadTask, "bSECSensorPoll", BSEC_STACK_SIZE, NULL, temp_TASK_PRIORITY, NULL);
    }
    
}






