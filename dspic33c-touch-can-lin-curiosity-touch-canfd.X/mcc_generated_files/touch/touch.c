/*******************************************************************************
  Touch Library 3.0.0 Release

  @Company
    Microchip Technology Inc.

  @File Name
    touch.c

  @Summary
    QTouch Modular Library

  @Description
    Provides Initialization, Processing and ISR handler of touch library,
          Simple API functions to get/set the key touch parameters from/to the
          touch library data structures.
	
*******************************************************************************/
/*
© [2023] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef TOUCH_C
#define TOUCH_C
/*----------------------------------------------------------------------------
 *     include files
 *----------------------------------------------------------------------------*/
#include "touch.h" 
#include "../system/clock.h"
#include "../timer/sccp4.h"


/*----------------------------------------------------------------------------
 *   prototypes
 *----------------------------------------------------------------------------*/

/*! \brief configure keys, wheels and sliders.
 */
static touch_ret_t touch_sensors_config(void);

/*! \brief Touch measure complete callback function example prototype.
 */
static void qtm_measure_complete_callback(void);

/*! \brief Touch post process complete callback function prototype.
 */
static void qtm_post_process_complete();

/*! \brief Touch Error callback function prototype.
 */
static void qtm_error_callback(uint8_t error);

/*! \brief Timer config.
 */
static void touch_timer_config(void);

/*----------------------------------------------------------------------------
 *     Global Variables
 *----------------------------------------------------------------------------*/

/* Flag to indicate time for touch measurement */
volatile uint8_t time_to_measure_touch_flag = 0;
/* post-process request flag */
volatile uint8_t touch_postprocess_request = 0;

/* Measurement Done Touch Flag  */
volatile uint8_t measurement_done_touch = 0;

/* Error Handling */
uint8_t module_error_code = 0;

/* Acquisition module internal data - Size to largest acquisition set */

uint16_t touch_acq_signals_raw[DEF_NUM_CHANNELS];

/* Acquisition set 1 - General settings */
qtm_acq_node_group_config_t adc_qtlib_acq_gen1
        = {DEF_NUM_CHANNELS, DEF_SENSOR_TYPE, DEF_SEL_FREQ_INIT,DEF_ADC_INT_PRIORITY};

/* Node status, signal, calibration values */
qtm_acq_node_data_t adc_qtlib_node_stat1[DEF_NUM_CHANNELS];

/* Node configurations */
qtm_cvd_acq_dspic33ck_node_config_t adc_seq_node_cfg1[DEF_NUM_CHANNELS] = { NODE_0_PARAMS, NODE_1_PARAMS,NODE_2_PARAMS};

qtm_cvd_acq_dspic33ck_device_config_t adc_clk_info = {
    .clk_freq = CLOCK_InstructionFrequencyGet(),
    .clk_src = FVCO_DIV4,
    .device_id = DSPIC33_DEVICE_IDENTIFIER,
    .ext_clk_freq =200      
};
/* Container */
qtm_acquisition_control_t qtlib_acq_set1 = {    .qtm_acq_node_group_config = &adc_qtlib_acq_gen1,
    .qtm_acq_node_config = &adc_seq_node_cfg1[0],
    .qtm_acq_node_data = &adc_qtlib_node_stat1[0],
    .qtm_acq_device_info = &adc_clk_info    };
/**********************************************************/
/*********************** Keys Module **********************/
/**********************************************************/

/* Keys set 1 - General settings */
qtm_touch_key_group_config_t qtlib_key_grp_config_set1 = {DEF_NUM_SENSORS,
                                                          DEF_TOUCH_DET_INT,
                                                          DEF_MAX_ON_DURATION,
                                                          DEF_ANTI_TCH_DET_INT,
                                                          DEF_ANTI_TCH_RECAL_THRSHLD,
                                                          DEF_TCH_DRIFT_RATE,
                                                          DEF_ANTI_TCH_DRIFT_RATE,
                                                          DEF_DRIFT_HOLD_TIME,
                                                          DEF_REBURST_MODE};

qtm_touch_key_group_data_t qtlib_key_grp_data_set1;

/* Key data */
qtm_touch_key_data_t qtlib_key_data_set1[DEF_NUM_SENSORS];

/* Key Configurations */
qtm_touch_key_config_t qtlib_key_configs_set1[DEF_NUM_SENSORS] = { KEY_0_PARAMS, KEY_1_PARAMS,KEY_2_PARAMS}; 
/* Container */
qtm_touch_key_control_t qtlib_key_set1
    = {&qtlib_key_grp_data_set1, &qtlib_key_grp_config_set1, &qtlib_key_data_set1[0], &qtlib_key_configs_set1[0]};
/**********************************************************/
/***************** Scroller Module ********************/
/**********************************************************/

/* Individual and Group Data */
qtm_scroller_data_t       qtm_scroller_data1[DEF_NUM_SCROLLERS];
qtm_scroller_group_data_t qtm_scroller_group_data1 = {0};

/* Group Configuration */
qtm_scroller_group_config_t qtm_scroller_group_config1 = {&qtlib_key_data_set1[0], DEF_NUM_SCROLLERS};

/* Scroller Configurations */
qtm_scroller_config_t qtm_scroller_config1[DEF_NUM_SCROLLERS] = {SCROLLER_0_PARAMS};  

/* Container */
qtm_scroller_control_t qtm_scroller_control1
    = {&qtm_scroller_group_data1, &qtm_scroller_group_config1, &qtm_scroller_data1[0], &qtm_scroller_config1[0]};



/*----------------------------------------------------------------------------
 *   function definitions
 *----------------------------------------------------------------------------*/

/*============================================================================
static touch_ret_t touch_sensors_config(void)
------------------------------------------------------------------------------
Purpose: Initialization of touch key sensors
Input  : none
Output : none
Notes  :
============================================================================*/
/* Touch sensors config - assign nodes to buttons / wheels / sliders / surfaces / water level / etc */
static touch_ret_t touch_sensors_config(void)
{
    uint16_t    sensor_nodes;
    touch_ret_t touch_ret = TOUCH_SUCCESS;
    /* Init acquisition module */
    touch_ret = qtm_cvd_init_acquisition_module(&qtlib_acq_set1);
    /* Init pointers to DMA sequence memory */
    
    if (TOUCH_SUCCESS != touch_ret)
    {
        /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
        qtm_error_callback(0x80);
    }
    else
    {
        /* Init pointers to DMA sequence memory */
        touch_ret = qtm_cvd_qtlib_assign_signal_memory(&touch_acq_signals_raw[0]);

    	/* Initialize sensor nodes */
    	for (sensor_nodes = 0u; sensor_nodes < DEF_NUM_CHANNELS; sensor_nodes++) {
            /* Enable each node for measurement and mark for calibration */
            touch_ret |= qtm_enable_sensor_node(&qtlib_acq_set1, sensor_nodes);
            touch_ret |= qtm_calibrate_sensor_node(&qtlib_acq_set1, sensor_nodes);

    	}	
	    /* Enable sensor keys and assign nodes */
	    for (sensor_nodes = 0u; sensor_nodes < DEF_NUM_CHANNELS; sensor_nodes++) {

            touch_ret |= qtm_init_sensor_key(&qtlib_key_set1, sensor_nodes, &adc_qtlib_node_stat1[sensor_nodes]);

	    }

    
		/* scroller init */
		touch_ret |= qtm_init_scroller_module(&qtm_scroller_control1);

	}

    return (touch_ret);
}

/*============================================================================
static void qtm_measure_complete_callback( void )
------------------------------------------------------------------------------
Purpose: Callback function called after the completion of
         measurement cycle. This function sets the post processing request
         flag to trigger the post processing.
Input  : none
Output : none
Notes  :
============================================================================*/
static void qtm_measure_complete_callback(void)
{
    touch_postprocess_request = 1u;
}

/*============================================================================
static void qtm_post_process_complete(void)
------------------------------------------------------------------------------
Purpose: Callback function from binding layer called after the completion of
         post processing. This function sets the reburst flag based on the
         key sensor group status, calls the datastreamer output function to
         display the module data.
Input  : none
Output : none
Notes  :
============================================================================*/
static void qtm_post_process_complete(void)
{

}

/*============================================================================
static void qtm_error_callback(uint8_t error)
------------------------------------------------------------------------------
Purpose: Callback function called after the completion of
         post processing. This function is called only when there is error.
Input  : error code
Output : decoded module error code
Notes  :
Derived Module_error_codes:
    Acquisition module error =1
    post processing module1 error = 2
    post processing module2 error = 3
    ... and so on

============================================================================*/
static void qtm_error_callback(uint8_t error)
{
    module_error_code = 0;
    if (error & 0x80)
    {
        module_error_code = 1;
    }
    else if (error & 0x40)
    {
        module_error_code = (error & 0x0F) + 2;
    }

}

/*============================================================================
void touch_timer_config(void)
------------------------------------------------------------------------------
Purpose: register Timer callback
Input  : none
Output : none
Notes  :
============================================================================*/
static void touch_timer_config(void)
{  
    Timer.Stop();
    Timer.TimeoutCallbackRegister(touch_timer_handler); 
    Timer.Start();

}
/*============================================================================
void touch_init(void)
------------------------------------------------------------------------------
Purpose: Initialization of touch library. PTC, timer, and
         datastreamer modules are initialized in this function.
Input  : none
Output : none
Notes  :
============================================================================*/
void touch_init(void)
{
    
    touch_timer_config();

	/* Configure touch sensors with Application specific settings */
	touch_sensors_config();
	
}

/*============================================================================
void touch_process(void)
------------------------------------------------------------------------------
Purpose: Main processing function of touch library. This function initiates the
         acquisition, calls post processing after the acquistion complete and
         sets the flag for next measurement based on the sensor status.
Input  : none
Output : none
Notes  :
============================================================================*/
void touch_process(void)
{
    touch_ret_t touch_ret;


    /* check the time_to_measure_touch for Touch Acquisition */
    if (time_to_measure_touch_flag == 1u)
	{
        /* Do the acquisition */
        touch_ret = qtm_cvd_start_measurement_seq(&qtlib_acq_set1, qtm_measure_complete_callback);

        /* if the Acquistion request was successful then clear the request flag */
        if (TOUCH_SUCCESS == touch_ret) {
            /* Clear the Measure request flag */
			time_to_measure_touch_flag = 0u;
        }
        else
        {
            /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0x81);
        }
    }
    /* check the flag for node level post processing */
    if (touch_postprocess_request == 1u){
        /* Reset the flags for node_level_post_processing */
        touch_postprocess_request = 0u;
        /* Run Acquisition module level post processing*/
        touch_ret = qtm_acquisition_process();
        /* Check the return value */
        if (TOUCH_SUCCESS == touch_ret) {
            /* Returned with success: Start module level post processing */

            touch_ret = qtm_key_sensors_process(&qtlib_key_set1);
            if (TOUCH_SUCCESS != touch_ret) {
                qtm_error_callback(1);
           }


            touch_ret = qtm_scroller_process(&qtm_scroller_control1);
            if (TOUCH_SUCCESS != touch_ret) {
                qtm_error_callback(2);
			}



         }else {
           /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0);
        }


        if((0u != (qtlib_key_set1.qtm_touch_key_group_data->qtm_keys_status & 0x80u)))
        {
        time_to_measure_touch_flag = 1u;
        } else {
            measurement_done_touch = 1u;
        }
        qtm_post_process_complete();

    }

    
}

uint16_t interrupt_cnt;
/*============================================================================
void touch_timer_handler(void)
------------------------------------------------------------------------------
Purpose: This function updates the time elapsed to the touch key module to
         synchronize the internal time counts used by the module.
Input  : none
Output : none
Notes  :
============================================================================*/
void touch_timer_handler(void)
{
    interrupt_cnt++;
	if (interrupt_cnt >= DEF_TOUCH_MEASUREMENT_PERIOD_MS) {
		interrupt_cnt = 0;
		/* Count complete - Measure touch sensors */
		time_to_measure_touch_flag = 1u;
    qtm_update_qtlib_timer(DEF_TOUCH_MEASUREMENT_PERIOD_MS);
}
}


/*============================================================================
 * Helper functions
============================================================================*/
uint16_t get_sensor_node_signal(uint16_t sensor_node)
{
    return (adc_qtlib_node_stat1[sensor_node].node_acq_signals);
}

void update_sensor_node_signal(uint16_t sensor_node, uint16_t new_signal)
{
    adc_qtlib_node_stat1[sensor_node].node_acq_signals = new_signal;
}

uint16_t get_sensor_node_reference(uint16_t sensor_node)
{
    return (qtlib_key_data_set1[sensor_node].channel_reference);
}

void update_sensor_node_reference(uint16_t sensor_node, uint16_t new_reference)
{
    qtlib_key_data_set1[sensor_node].channel_reference = new_reference;
}

uint16_t get_sensor_cc_val(uint16_t sensor_node)
{
    return (adc_qtlib_node_stat1[sensor_node].node_comp_caps);
}

void update_sensor_cc_val(uint16_t sensor_node, uint16_t new_cc_value)
{
    adc_qtlib_node_stat1[sensor_node].node_comp_caps = new_cc_value;
}

uint8_t get_sensor_state(uint16_t sensor_node)
{
    return (qtlib_key_set1.qtm_touch_key_data[sensor_node].sensor_state);
}
uint8_t get_sensor_threshold(uint16_t sensor_node)
{
    return (qtlib_key_configs_set1[sensor_node].channel_threshold);
}
void update_sensor_state(uint16_t sensor_node, uint8_t new_state)
{
    qtlib_key_set1.qtm_touch_key_data[sensor_node].sensor_state = new_state;
}
void calibrate_node(uint16_t sensor_node)
{
    /* Calibrate Node */
    qtm_calibrate_sensor_node(&qtlib_acq_set1, sensor_node);
    /* Initialize key */
    qtm_init_sensor_key(&qtlib_key_set1, sensor_node, &adc_qtlib_node_stat1[sensor_node]);
}
uint8_t get_scroller_state(uint16_t sensor_node)
{
	return (qtm_scroller_control1.qtm_scroller_data[sensor_node].scroller_status);
}

uint16_t get_scroller_position(uint16_t sensor_node)
{
	return (qtm_scroller_control1.qtm_scroller_data[sensor_node].position);
}



/*============================================================================
ISR(ADC)
------------------------------------------------------------------------------
Purpose:  Interrupt handler for ADC / EOC Interrupt
Input    :  none
Output  :  none
Notes    :  none
============================================================================*/
void __attribute__((__interrupt__, auto_psv, weak)) _ADCInterrupt(void)
{

    qtm_dspic33_touch_handler_eoc();
}
#endif /* TOUCH_C */