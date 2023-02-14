/*******************************************************************************
  Touch Library Release

  Company:
    Microchip Technology Inc.

  File Name:
    touch.c

  Summary:
    QTouch Modular Library

  Description:
    Provides Initialization, Processing and ISR handler of touch library,
    Simple API functions to get/set the key touch parameters from/to the
    touch library data structures
*******************************************************************************/

/*******************************************************************************
Copyright (c)  2023 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/




#ifndef TOUCH_C
#define TOUCH_C
/*----------------------------------------------------------------------------
 *     include files
 *----------------------------------------------------------------------------*/

#include "touch.h"
#include "datastreamer/datastreamer.h"
#include "../timer/tmr1.h"

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
uint32_t local_sys_freq = CLOCK_SystemFrequencyGet();

/* Measurement Done Touch Flag  */
volatile uint8_t measurement_done_touch = 0;

/* Flag to indicate time for touch measurement */
volatile uint8_t time_to_measure_touch_var = 0;
/* post-process request flag */
volatile uint8_t touch_postprocess_request = 0;

/* Error Handling */
uint8_t module_error_code = 0;

/* Acquisition module internal data - Size to largest acquisition set */
uint16_t touch_acq_signals_raw[DEF_NUM_CHANNELS];

/* Acquisition set 1 - General settings */
qtm_acq_node_group_config_t adc_qtlib_acq_gen1
        = {DEF_NUM_CHANNELS, DEF_SENSOR_TYPE, DEF_SEL_FREQ_INIT};

/* Node status, signal, calibration values */
qtm_acq_node_data_t adc_qtlib_node_stat1[DEF_NUM_CHANNELS];

/* Node configurations */
qtm_cvd_acq_dspic33ck_node_config_t adc_seq_node_cfg1[DEF_NUM_CHANNELS] = {
    NODE_0_PARAMS, NODE_1_PARAMS, NODE_2_PARAMS
};

qtm_cvd_acq_dspic33ck_device_config_t adc_clk_info = {
    .clk_freq = CLOCK_InstructionFrequencyGet(),
    .clk_src = FVCO_DIV4,
    .device_id = DSPIC33_DEVICE_IDENTIFIER
};

/* Container */
qtm_acquisition_control_t qtlib_acq_set1 = {
    .qtm_acq_node_group_config = &adc_qtlib_acq_gen1,
    .qtm_acq_node_config = &adc_seq_node_cfg1[0],
    .qtm_acq_node_data = &adc_qtlib_node_stat1[0],
    .qtm_acq_device_info = &adc_clk_info
};

/**********************************************************/
/*********** Frequency Hop Auto tune Module **********************/
/**********************************************************/

/* Buffer used with various noise filtering functions */
uint16_t noise_filter_buffer[DEF_NUM_SENSORS * NUM_FREQ_STEPS];
uint8_t freq_hop_delay_selection[NUM_FREQ_STEPS] = {DEF_MEDIAN_FILTER_FREQUENCIES};
uint8_t freq_hop_autotune_counters[NUM_FREQ_STEPS];

/* Configuration */
qtm_freq_hop_autotune_config_t qtm_freq_hop_autotune_config1 = {DEF_NUM_CHANNELS,
    NUM_FREQ_STEPS,
    &adc_qtlib_acq_gen1.freq_option_select,
    &freq_hop_delay_selection[0],
    DEF_FREQ_AUTOTUNE_ENABLE,
    FREQ_AUTOTUNE_MAX_VARIANCE,
    FREQ_AUTOTUNE_COUNT_IN};

/* Data */
qtm_freq_hop_autotune_data_t qtm_freq_hop_autotune_data1
        = {0, 0, &noise_filter_buffer[0], &adc_qtlib_node_stat1[0], &freq_hop_autotune_counters[0]};

/* Container */
qtm_freq_hop_autotune_control_t qtm_freq_hop_autotune_control1
        = {&qtm_freq_hop_autotune_data1, &qtm_freq_hop_autotune_config1};

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

qtm_touch_key_config_t qtlib_key_configs_set1[DEF_NUM_SENSORS] = {
    KEY_0_PARAMS, KEY_1_PARAMS, KEY_2_PARAMS
};
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
    uint16_t sensor_nodes;
    touch_ret_t touch_ret = TOUCH_SUCCESS;
    
    touch_ret = qtm_cvd_init_acquisition_module(&qtlib_acq_set1);
    
    if (TOUCH_SUCCESS != touch_ret)
    {
        /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
        qtm_error_callback(0x80);
    }
    else
    {
        /* Init pointers to DMA sequence memory */
        touch_ret = qtm_cvd_qtlib_assign_signal_memory(&touch_acq_signals_raw[0],NULL,NULL);

        /* Initialize sensor nodes */
        for (sensor_nodes = 0u; sensor_nodes < DEF_NUM_CHANNELS; sensor_nodes++)
        {
            /* Enable each node for measurement and mark for calibration */
            touch_ret |= qtm_enable_sensor_node(&qtlib_acq_set1, sensor_nodes);
            touch_ret |= qtm_calibrate_sensor_node(&qtlib_acq_set1, sensor_nodes);
        }

        /* Enable sensor keys and assign nodes */
        for (sensor_nodes = 0u; sensor_nodes < DEF_NUM_CHANNELS; sensor_nodes++)
        {
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
Purpose: Callback function from binding layer called after the completion of
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
#if DEF_TOUCH_DATA_STREAMER_ENABLE == 1
    datastreamer_output();
#endif
}

/*============================================================================
static void qtm_error_callback(uint8_t error)
------------------------------------------------------------------------------
Purpose: Callback function from binding layer called after the completion of
         post processing. This function is called only when there is error.
Input  : error code
Output : decoded module error code
Notes  :
Error Handling supported by Binding layer module:
    Acquisition Module Error codes: 0x8<error code>
    0x81 - Qtm init
    0x82 - start acq
    0x83 - cal sensors
    0x84 - cal hardware

    Post processing Modules error codes: 0x4<process_id>
    0x40, 0x41, 0x42, ...
    process_id is the sequence of process IDs listed in #define LIB_MODULES_PROC_LIST macro.
    Process IDs start from zero and maximum is 15

    Examples:
    0x40 -> error in post processing module 1
    0x42 -> error in post processing module 3

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

#if DEF_TOUCH_DATA_STREAMER_ENABLE == 1
    datastreamer_output();
#endif
}

/*============================================================================
void Timer_set_period(const uint8_t val)
------------------------------------------------------------------------------
Purpose: This function sets the time interval on the RTC/Timer peripheral based
         on the user configuration.
Input  : Time interval
Output : none
Notes  :
============================================================================*/
void Timer_set_period(const uint8_t val)
{

}

/*============================================================================
void touch_init(void)
------------------------------------------------------------------------------
Purpose: Initialization of touch library. PTC, timer, binding layer and
         datastreamer modules are initialized in this function.
Input  : none
Output : none
Notes  :
============================================================================*/
void touch_init(void)
{
    
    touch_timer_config();
    
    /* Initialize touch  */
    touch_sensors_config();

#if (DEF_TOUCH_DATA_STREAMER_ENABLE == 1u)
    datastreamer_init();
#endif
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
    touch_ret_t touch_ret = TOUCH_SUCCESS;

    /* check the time_to_measure_touch for Touch Acquisition */
    if (time_to_measure_touch_var == 1u)
    {

        /* Do the acquisition */
        touch_ret = qtm_cvd_start_measurement_seq(&qtlib_acq_set1, qtm_measure_complete_callback);

        /* if the Acquistion request was successful then clear the request flag */
        if (TOUCH_SUCCESS == touch_ret)
        {
            /* Clear the Measure request flag */
            time_to_measure_touch_var = 0;
        }
        else
        {
            /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0x81);
        }
    }
    /* check the flag for node level post processing */
    if (touch_postprocess_request == 1u)
    {
        /* Reset the flags for node_level_post_processing */
        touch_postprocess_request = 0u;
        /* Run Acquisition module level post processing*/
        touch_ret = qtm_acquisition_process();
        /* Check the return value */
        if (TOUCH_SUCCESS == touch_ret)
        {
            /* Returned with success: Start module level post processing */
            touch_ret = qtm_freq_hop_autotune(&qtm_freq_hop_autotune_control1);
            
            if (TOUCH_SUCCESS != touch_ret)
            {
                qtm_error_callback(0x41);
            }

            touch_ret = qtm_update_acq_freq_delay(*qtm_freq_hop_autotune_config1.freq_option_select);

            if (TOUCH_SUCCESS != touch_ret)
            {
                qtm_error_callback(0x41);
            }
            touch_ret = qtm_key_sensors_process(&qtlib_key_set1);
            if (TOUCH_SUCCESS != touch_ret)
            {
                qtm_error_callback(0x42);
            }
			touch_ret = qtm_scroller_process(&qtm_scroller_control1);
            if (TOUCH_SUCCESS != touch_ret) {
                qtm_error_callback(3);
            }
        }
        else
        {
            /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0x40);
        }


        if (0u != (qtlib_key_set1.qtm_touch_key_group_data->qtm_keys_status & QTM_KEY_REBURST))
        {
            time_to_measure_touch_var = 1u;
        }
        else
        {
            measurement_done_touch = 1u;
        }
        
        qtm_post_process_complete();
    }
}

uint8_t interrupt_cnt;

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
    if (interrupt_cnt >= DEF_TOUCH_MEASUREMENT_PERIOD_MS)
    {
        interrupt_cnt = 0;
        /* Count complete - Measure touch sensors */
        time_to_measure_touch_var = 1u;
        qtm_update_qtlib_timer((uint16_t) DEF_TOUCH_MEASUREMENT_PERIOD_MS);
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
    Timer1.Stop();
    Timer1.TimeoutCallbackRegister(touch_timer_handler); 
    Timer1.Start(); 
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
    return (qtlib_key_data_set1[sensor_node].sensor_state);
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

uint8_t get_current_frequency(void)
{
    return *qtm_freq_hop_autotune_config1.freq_option_select;
}

uint8_t get_filter_frequency(uint8_t idx)
{
    return qtm_freq_hop_autotune_config1.median_filter_freq[idx];
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
void ADC1_CommonCallback (void)
{
    qtm_dspic33_touch_handler_eoc();
}

#endif /* TOUCH_C */
