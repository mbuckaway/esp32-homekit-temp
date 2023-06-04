/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_integration.c
 *
 * @brief
 * Private part of the example for using of BSEC library.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "bsec_integration.h"

/**********************************************************************************************************************/
/* local macro definitions */
/**********************************************************************************************************************/

#define NUM_USED_OUTPUTS 10

/**********************************************************************************************************************/
/* global variable declarations */
/**********************************************************************************************************************/

/* Global sensor APIs data structure */
static struct bme68x_dev bme68x_g;
static struct bme68x_conf conf;
static struct bme68x_heatr_conf heatr_conf;
static struct bme68x_data sensor_data[3];

uint8_t dev_addr = BME68X_I2C_ADDR_LOW;

/* State change and temporary data place holders */
uint8_t lastOpMode = BME68X_SLEEP_MODE;
float extTempOffset = 0.0f;
uint8_t opMode;
uint8_t nFields, iFields;
/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief        Virtual sensor subscription
 *               Please call this function before processing of data using bsec_do_steps function
 *
 * @param[in]    sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 *  
 * @return       subscription result, zero when successful
 */
static bsec_library_return_t bme68x_bsec_update_subscription(float sample_rate)
{
    bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
    uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;
    
    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    
    bsec_library_return_t status = BSEC_OK;
    
    /* note: Virtual sensors as desired to be added here */
    //useless for BME 680
    // requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_1;
    // requested_virtual_sensors[0].sample_rate = sample_rate;
    // requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_2;
    // requested_virtual_sensors[1].sample_rate = sample_rate;
    // requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_3;
    // requested_virtual_sensors[2].sample_rate = sample_rate;
    // requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_GAS_ESTIMATE_4;
    // requested_virtual_sensors[3].sample_rate = sample_rate;

    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    requested_virtual_sensors[0].sample_rate = sample_rate;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[1].sample_rate = sample_rate;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[2].sample_rate = sample_rate;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[3].sample_rate = sample_rate;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[4].sample_rate = sample_rate;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[5].sample_rate = sample_rate;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[6].sample_rate = sample_rate;
    requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[7].sample_rate = sample_rate;
    // requested_virtual_sensors[8].sensor_id = BSEC_OUTPUT_RAW_GAS_INDEX;
    // requested_virtual_sensors[8].sample_rate = sample_rate;
    requested_virtual_sensors[8].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[8].sample_rate = sample_rate;
    requested_virtual_sensors[9].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    requested_virtual_sensors[9].sample_rate = sample_rate;
    
    /* Call bsec_update_subscription() to enable/disable the requested virtual sensors */
    status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings,
        &n_required_sensor_settings);
    
    return status;
}

/*!
 * @brief       Initialize the bme68x sensor and the BSEC library
 *
 * @param[in]   sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 * @param[in]   temperature_offset  device-specific temperature offset (due to self-heating)
 * @param[in]   bus_write           pointer to the bus writing function
 * @param[in]   bus_read            pointer to the bus reading function
 * @param[in]   sleep               pointer to the system specific sleep function
 * @param[in]   state_load          pointer to the system-specific state load function
 * @param[in]   config_load         pointer to the system-specific config load function
 * @param[in]   dev         		pointer to the sensor communication and inventory details strucuture	
 *
 * @return      zero if successful, negative otherwise
 */
return_values_init bsec_iot_init(float sample_rate, float temperature_offset, bme68x_write_fptr_t bus_write, 
                    bme68x_read_fptr_t bus_read, sleep_fct sleep, state_load_fct state_load, config_load_fct config_load,  struct bme68x_dev dev)
{
    return_values_init ret = {BME68X_OK, BSEC_OK};
    
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE] = {0};
    int32_t bsec_state_len, bsec_config_len;

    bme68x_g = dev;


    // is this needed? not sure
    //bme68x_g.dev_id = BME680_I2C_ADDR_SECONDARY;
    bme68x_g.intf = BME68X_I2C_INTF;

    bme68x_g.read = bus_read;
    bme68x_g.write = bus_write;
    bme68x_g.delay_us = sleep;

    extTempOffset = temperature_offset;
	
    /* Initialize bme68x API */
    ret.bme68x_status = bme68x_init(&bme68x_g);
    if (ret.bme68x_status != BME68X_OK)
    {
        return ret;
    }
    
    /* Initialize BSEC library */
    ret.bsec_status = bsec_init();
    if (ret.bsec_status != BSEC_OK)
    {
        return ret;
    }
    
    /* Load library config, if available */
    bsec_config_len = config_load(bsec_config, sizeof(bsec_config));
    if (bsec_config_len != 0)
    {       
        ret.bsec_status = bsec_set_configuration(bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer));
        if (ret.bsec_status != BSEC_OK)
        {
            return ret;
        }
    }
    
    /* Load previous library state, if available */
    bsec_state_len = state_load(bsec_state, sizeof(bsec_state));
    if (bsec_state_len != 0)
    {       
        ret.bsec_status = bsec_set_state(bsec_state, bsec_state_len, work_buffer, sizeof(work_buffer));     
        if (ret.bsec_status != BSEC_OK)
        {
            return ret;
        }
    }
    
    /* Call to the function which sets the library with subscription information */
    ret.bsec_status = bme68x_bsec_update_subscription(sample_rate);
    if (ret.bsec_status != BSEC_OK)
    {
        return ret;
    }
    
    return ret;
}

/*!
 * @brief       This function is written to process the sensor data for the requested virtual sensors
 *
 * @param[in]   bsec_inputs         input structure containing the information on sensors to be passed to do_steps
 * @param[in]   num_bsec_inputs     number of inputs to be passed to do_steps
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 *
 * @return      library function return codes, zero when successful 
 */
static bsec_library_return_t bme68x_bsec_process_data(bsec_input_t *bsec_inputs, uint8_t num_bsec_inputs, output_ready_fct output_ready)
{
    /* Output buffer set to the maximum virtual sensor outputs supported */
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_bsec_outputs = 0;
    uint8_t index = 0;

    bsec_library_return_t bsec_status = BSEC_OK;
    
    int64_t timestamp = 0;
    // float gas_estimate_1 = 0.0f;
    // float gas_estimate_2 = 0.0f;
    // float gas_estimate_3 = 0.0f;
    // float gas_estimate_4 = 0.0f;
	float raw_pressure = 0.0f;
	float raw_temp = 0.0f;
	float raw_humidity = 0.0f;
	float raw_gas = 0.0f;
    float iaq = 0.0f;
    uint8_t iaq_accuracy = 0;
	float siaq = 0.0f;
    uint8_t siaq_accuracy = 0;
	float compensateTemperature = 0.0f;
	float compensateHumidity = 0.0f;
    float co2 = 0.0f;
    float bVOC = 0.0f;
	// uint8_t raw_gas_index = 0;

    // requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    // requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    // requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    // requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    //BSEC_OUTPUT_CO2_EQUIVALENT

    /* Check if something should be processed by BSEC */
    if (num_bsec_inputs > 0)
    {
        /* Set number of outputs to the size of the allocated buffer */
        /* BSEC_NUMBER_OUTPUTS to be defined */
        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;
        
        /* Perform processing of the data by BSEC 
           Note:
           * The number of outputs you get depends on what you asked for during bsec_update_subscription(). This is
             handled under bme68x_bsec_update_subscription() function in this example file.
           * The number of actual outputs that are returned is written to num_bsec_outputs. */
        bsec_status = bsec_do_steps(bsec_inputs, num_bsec_inputs, bsec_outputs, &num_bsec_outputs);
        
        /* Iterate through the outputs and extract the relevant ones. */
        for (index = 0; index < num_bsec_outputs; index++)
        {
            switch (bsec_outputs[index].sensor_id)
            {
                // case BSEC_OUTPUT_GAS_ESTIMATE_1:
                //     gas_estimate_1 = bsec_outputs[index].signal;
                //     break;
                // case BSEC_OUTPUT_GAS_ESTIMATE_2:
                //     gas_estimate_2 = bsec_outputs[index].signal;
                //     break;
                // case BSEC_OUTPUT_GAS_ESTIMATE_3:
                //     gas_estimate_3 = bsec_outputs[index].signal;
                //     break;
                // case BSEC_OUTPUT_GAS_ESTIMATE_4:
                //     gas_estimate_4 = bsec_outputs[index].signal;
                //     break;
                case BSEC_OUTPUT_IAQ:
                    iaq = bsec_outputs[index].signal;
                    iaq_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_STATIC_IAQ:
                    siaq = bsec_outputs[index].signal;
                    siaq_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                    compensateTemperature = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                    compensateHumidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_CO2_EQUIVALENT:
                    co2 = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                    bVOC = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_PRESSURE:
                    raw_pressure = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_TEMPERATURE:
                    raw_temp = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_HUMIDITY:
                    raw_humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_GAS:
                    raw_gas = bsec_outputs[index].signal;
                    break;
                // case BSEC_OUTPUT_RAW_GAS_INDEX:
                //     raw_gas_index = (uint8_t)bsec_outputs[index].signal;
                //     break;
                default:
                    continue;
            }
            
            /* Assume that all the returned timestamps are the same */
            timestamp = bsec_outputs[index].time_stamp;
        }
        
        /* Pass the extracted outputs to the user provided output_ready() function. */
        output_ready(timestamp, iaq, iaq_accuracy, siaq, siaq_accuracy, compensateTemperature, compensateHumidity, raw_pressure, raw_temp, raw_humidity, raw_gas, co2, bVOC, bsec_status);
    }
	return bsec_status;
}

/*!
 * @brief       Read the data from registers and populate the inputs structure to be passed to do_steps function
 *
 * @param[in]   currTimeNs      		system timestamp value passed for processing data
 * @param[in]   data                  	input structure that contains the gas sensor data to be passed to process data
 * @param[in]   bsec_process_data       process data variable returned from sensor_control
 * @param[in]   output_ready        	pointer to the function processing obtained BSEC outputs
 *
 * @return      function result, one when successful & zero when unsuccessful
 */
uint8_t processData(int64_t currTimeNs, struct bme68x_data data, int32_t bsec_process_data, output_ready_fct output_ready)
{
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
	bsec_library_return_t bsec_status = BSEC_OK;
    uint8_t nInputs = 0;
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HEATSOURCE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[nInputs].signal = extTempOffset;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_TEMPERATURE))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
#else
        inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE / 100.0f;
#endif
        inputs[nInputs].signal = data.temperature;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_HUMIDITY))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;
#else
        inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY / 1000.0f;
#endif
        inputs[nInputs].signal = data.humidity;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PRESSURE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[nInputs].signal = data.pressure;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_GASRESISTOR) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[nInputs].signal = data.gas_resistance;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_process_data, BSEC_INPUT_PROFILE_PART) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[nInputs].signal = (opMode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }

    if (nInputs > 0)
    {
        /* Processing of the input signals and returning of output samples is performed by bsec_do_steps() */
		bsec_status = bme68x_bsec_process_data(inputs, nInputs, output_ready);

        if (bsec_status != BSEC_OK)
            return 0;
    }
    return 1;
}

/*!
 * @brief       Function to get the measurement duration in microseconds
 *
 * @param[in]   mode      			sensor operation mode to calculate the shared heater duration
 *
 * @return      calculated duration
 */
uint32_t getMeasDur(uint8_t mode)
{
	if (mode == BME68X_SLEEP_MODE)
		mode = lastOpMode;

	return bme68x_get_meas_dur(mode, &conf, &bme68x_g);
}

/**
 * @brief Set the BME68X sensor configuration to parallel mode
 *
 * @param[in]   sensor_settings     settings of the bme68x sensor adopted by sensor control function
 *
 * @return      none
 */
void setBme68xConfigParallel(bsec_bme_settings_t *sensor_settings)
{
    uint16_t sharedHeaterDur = 0;
	int8_t status;
    
	/* Set the filter, odr, temperature, pressure and humidity settings */
	status = bme68x_get_conf(&conf, &bme68x_g);
	if (status != BME68X_OK)
		return;

	conf.os_hum = sensor_settings->humidity_oversampling;
	conf.os_temp = sensor_settings->temperature_oversampling;
	conf.os_pres = sensor_settings->pressure_oversampling;
	status = bme68x_set_conf(&conf, &bme68x_g);
    if (status != BME68X_OK)
		return;
        

    sharedHeaterDur = BSEC_TOTAL_HEAT_DUR - (getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));
	heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp_prof = sensor_settings->heater_temperature_profile;
	heatr_conf.heatr_dur_prof = sensor_settings->heater_duration_profile;
	heatr_conf.shared_heatr_dur = sharedHeaterDur;
	heatr_conf.profile_len = sensor_settings->heater_profile_len;
    status = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme68x_g);
    if (status != BME68X_OK)
		return;

    status = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme68x_g);
	if (status != BME68X_OK)
		return;
		
	lastOpMode = BME68X_PARALLEL_MODE;
    opMode = BME68X_PARALLEL_MODE;
}

/**
 * @brief Set the BME68X sensor configuration to forced mode
 *
 * @param[in]   sensor_settings     settings of the bme68x sensor adopted by sensor control function
 *
 * @return      none
 */
void setBme68xConfigForced(bsec_bme_settings_t *sensor_settings)
{
	int8_t status;
	
    /* Set the filter, odr, temperature, pressure and humidity settings */
	status = bme68x_get_conf(&conf, &bme68x_g);
    if (status != BME68X_OK)
		return;
	
	conf.os_hum = sensor_settings->humidity_oversampling;
	conf.os_temp = sensor_settings->temperature_oversampling;
	conf.os_pres = sensor_settings->pressure_oversampling;
	status = bme68x_set_conf(&conf, &bme68x_g);
    if (status != BME68X_OK)
		return;

    heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp = sensor_settings->heater_temperature;
	heatr_conf.heatr_dur = sensor_settings->heater_duration;
	status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme68x_g);
    if (status != BME68X_OK)
		return;

    status = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x_g);
	if (status != BME68X_OK)
		return;
	
	lastOpMode = BME68X_FORCED_MODE;
    opMode = BME68X_FORCED_MODE;
}

/**
 * @brief Function to get a single data field
 */
/*!
 * @brief       Function to get a single data field
 *
 * @param[in]   currTimeNs      		system timestamp value passed for processing data
 * @param[in]   data                  	input structure that contains the gas sensor raw data collected
 *
 * @return      number of fields to process, zero when nothing to process
 */ 
uint8_t getData(struct bme68x_data *data)
{
	if (lastOpMode == BME68X_FORCED_MODE)
	{
		*data = sensor_data[0];
	} else
	{
		if (nFields)
		{
			/* iFields spans from 0-2 while nFields spans from
			 * 0-3, where 0 means that there is no new data
			 */
			*data = sensor_data[iFields];
			iFields++;

			/* Limit reading continuously to the last fields read */
			if (iFields >= nFields)
			{
				iFields = nFields - 1;
				return 0;
			}

			/* Indicate if there is something left to read */
			return nFields - iFields;
		}
	}

	return 0;
}

/*!
 * @brief       Runs the main (endless) loop that queries sensor settings, applies them, and processes the measured data
 *
 * @param[in]   sleep               pointer to the system specific sleep function
 * @param[in]   get_timestamp_us    pointer to the system specific timestamp derivation function
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 * @param[in]   state_save          pointer to the system-specific state save function
 * @param[in]   save_intvl          interval at which BSEC state should be saved (in samples)
 *
 * @return      none
 */
void bsec_iot_loop(sleep_fct sleep, get_timestamp_us_fct get_timestamp_us, output_ready_fct output_ready,
                    state_save_fct state_save, uint32_t save_intvl)
{
    /* Timestamp variables */
    int64_t time_stamp = 0;
    
    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;
	memset(&sensor_settings, 0, sizeof(sensor_settings));
	
	/* BSEC sensor data */
	struct bme68x_data data;
    
    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
	uint8_t nFieldsLeft = 0;
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;
	int8_t ret_val;
	
    opMode = sensor_settings.op_mode;
	
    bsec_library_return_t bsec_status = BSEC_OK;
	sensor_settings.next_call = 0;
	
    while (1)
    {
        /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
        time_stamp = get_timestamp_us() * 1000;

        // printf("INFO: NEXT CALL IS: %"PRId64"\n", sensor_settings.next_call);
		if (time_stamp >= sensor_settings.next_call)
		{
			/* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
			bsec_status = bsec_sensor_control(time_stamp, &sensor_settings);
			if (bsec_status != BSEC_OK)
			{
				if (bsec_status < BSEC_OK)
				{
					printf("ERROR: bsec_sensor_control: %d\n", bsec_status);
					break;
				}
				else
				{
					printf("WARNING: bsec_sensor_control: %d\n", bsec_status);
				}
			}
			switch (sensor_settings.op_mode)
			{
			case BME68X_FORCED_MODE:
				setBme68xConfigForced(&sensor_settings);
				break;
			case BME68X_PARALLEL_MODE:
				if (opMode != sensor_settings.op_mode)
				{
					setBme68xConfigParallel(&sensor_settings);
				}
				break;
			case BME68X_SLEEP_MODE:
				if (opMode != sensor_settings.op_mode)
				{
					ret_val = bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme68x_g);
					if ((ret_val == BME68X_OK) && (opMode != BME68X_SLEEP_MODE))
					{
						opMode = BME68X_SLEEP_MODE;
					}
				}
				break;
			}
			
			if (sensor_settings.trigger_measurement && sensor_settings.op_mode != BME68X_SLEEP_MODE)
			{
				nFields = 0;
				bme68x_get_data(lastOpMode, &sensor_data[0], &nFields, &bme68x_g);
				iFields = 0;
				if(nFields)
				{
					do
					{
						nFieldsLeft = getData(&data);
						/* check for valid gas data */
						if (data.status & BME68X_GASM_VALID_MSK)
						{
							if (!processData(time_stamp, data, sensor_settings.process_data, output_ready))
							{
								return;
							}
						}
					}while(nFieldsLeft);
				}
			}
			
			/* Increment sample counter */
			n_samples++;
			
			/* Retrieve and store state if the passed save_intvl */
			if (n_samples >= save_intvl)
			{
				bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
				if (bsec_status == BSEC_OK)
				{
					state_save(bsec_state, bsec_state_len);
				}
				n_samples = 0;
			}
		}
	}
}

/*! @}*/

