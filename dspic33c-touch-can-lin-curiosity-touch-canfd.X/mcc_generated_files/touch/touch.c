/*******************************************************************************
  Touch Library 4.0.0 Release

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
© [2024] Microchip Technology Inc. and its subsidiaries.

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

/*! \brief Device config.
 */
static touch_ret_t touch_device_config(void);

/*----------------------------------------------------------------------------
 *     Global Variables
 *----------------------------------------------------------------------------*/

/* Flag to indicate time for touch measurement */
volatile uint8_t time_to_measure_touch_flag = 0;
/* post-process request flag */
volatile uint8_t touch_postprocess_request = 0;

/* Measurement Done Touch Flag  */
volatile uint8_t measurement_done_touch = 0;

/* track touch in progress flag */
volatile uint8_t touch_in_progress = 0u;

/* Error Handling */
volatile uint8_t module_error_code = 0;

/* Acquisition module internal data - Size to largest acquisition set */

uint32_t touch_acq_signals_raw[DEF_NUM_CHANNELS];
uint16_t touch_acq_scanA_signals[DEF_NUM_CHANNELS];
uint16_t touch_acq_scanB_signals[DEF_NUM_CHANNELS];
uint16_t touch_cvd_scan_data[2u * (1u<<FILTER_LEVEL_64)]; /* memory of size (max oversampling of all channels * 2) */

/* Acquisition set 1 - General settings */
qtm_acq_node_group_config_t adc_qtlib_acq_gen1 = 
    {DEF_NUM_CHANNELS, DEF_SENSOR_TYPE, DEF_SEL_FREQ_INIT};

/* Node status, signal, calibration values */
qtm_acq_node_data_t adc_qtlib_node_stat1[DEF_NUM_CHANNELS];

/* Node configurations */
qtm_acq_dspic33c_node_config_t adc_seq_node_cfg1[DEF_NUM_CHANNELS] = { NODE_0_PARAMS, NODE_1_PARAMS,NODE_2_PARAMS};

qtm_acq_dspic33c_device_config_t device_config = {
    .adc_clk_src = FOSC,
    .device_id = DSPIC33_DEVICE_IDENTIFIER,
    .cvd_scan_data_ptr = &touch_cvd_scan_data[0u],
    .partner_adc_channel = CVD_PARTNER_ADC_CHANNEL
};
/* Container */
qtm_acquisition_control_t qtlib_acq_set1 = {    .qtm_acq_node_group_config = &adc_qtlib_acq_gen1,
    .qtm_acq_node_config = &adc_seq_node_cfg1[0],
    .qtm_acq_node_data = &adc_qtlib_node_stat1[0],
    .qtm_acq_device_config = &device_config    };
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
        touch_ret = qtm_cvd_qtlib_assign_signal_memory(&touch_acq_signals_raw[0u]);

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
static void touch_device_config(void)
------------------------------------------------------------------------------
Purpose: To set Device configurations
Input  : none
Output : none
Notes  :  1. The touch_calculate_frequency function will calculate and update the adc_clk_src_freq, dma_transfer_delay, and sccp_drive_delay of device_config structure.
          2. The sccp_drive_delay is calculated such that the shield is driven shortly after the start of charge share.  
          3. If required, adjust sccp_drive_delay by checking the delay between start of charge share and shield drive using oscilloscope. Ensure that shield is not driven before start of charge share.          
 ============================================================================*/
static touch_ret_t touch_device_config(void)
{
    uint32_t adc_clk_src_frequency = 0uL;
    uint32_t fvco_frequency = 0uL;
    uint16_t pll_pre = CLKDIVbits.PLLPRE;
    uint16_t pll_fbd = PLLFBD;
    uint16_t local_nosc = OSCCONbits.NOSC;
    uint32_t peripheral_clk_frequency = 0uL; 
    uint32_t core_adc_clk_freq = 0uL;
    uint32_t adc_core_src_frequency = 0uL;
    uint32_t core_clkdiv = 1uL; 
    uint8_t apll_avcodiv = 0u;
    uint8_t delay_mod_val = 0u;
    adc_clock_src_t adc_clock_src = device_config.adc_clk_src;
    touch_ret_t touch_ret = TOUCH_SUCCESS;
    
    /* SCCP and DMA clock source is Fp (FOSC/2) */
    peripheral_clk_frequency = CLOCK_PeripheralFrequencyGet();
    
    #if defined (_APLLPRE) 
    /* read Aux PLL settings */
    if(adc_clock_src == AFVCODIV)
    {
        pll_pre = _APLLPRE;
        pll_fbd = APLLFBD1;
        apll_avcodiv = _AVCODIV; 
    }
    #endif

    /* calculate frequency */
    if(adc_clock_src == FOSC)
    {
        adc_clk_src_frequency = CLOCK_SystemFrequencyGet();
    }
    else if(adc_clock_src == PERIPHERAL_CLK)
    {
        adc_clk_src_frequency = CLOCK_PeripheralFrequencyGet();
    }
    else if(adc_clock_src == INSTRUCTION_CLK)
    {
        adc_clk_src_frequency = CLOCK_InstructionFrequencyGet();
    }
    else if((adc_clock_src == FVCO_DIV4) || (adc_clock_src == FVCO_DIV3) || (adc_clock_src == AFVCODIV))
    {
        if (pll_pre == 0u)
        {
            pll_pre = 1u; /* avoid divide by 0 */
        }
        if (pll_fbd == 0u)
        {
            pll_fbd = 1u; /* avoid divide by 0 */
        }
        
        if (local_nosc == 3u) /* Primary Oscillator with PLL */
        {
            #if defined (EXT_CLK_FREQ) 
            fvco_frequency = (EXT_CLK_FREQ * (pll_fbd & 0x00ffu)) / pll_pre;
            #endif
        }
        else
        {
            #if defined (FRC_CLK_FREQ) 
            fvco_frequency = (FRC_CLK_FREQ * (pll_fbd & 0x00ffu)) / pll_pre;
            #endif
        }
        
        if(adc_clock_src == FVCO_DIV4)
        {
            adc_clk_src_frequency = fvco_frequency / 4u;
        }
        else if (adc_clock_src == FVCO_DIV3)
        {
            adc_clk_src_frequency = fvco_frequency / 3u;
        }
		else if (adc_clock_src == AFVCODIV)
        {
            adc_clk_src_frequency = fvco_frequency/ (4u - apll_avcodiv);
        }
        else
        {
            /* invalid */
        }

    }
    else
    {
        /* invalid */
    }
    /* set adc clock source frequency (FSRC) */
    device_config.adc_clk_src_freq = adc_clk_src_frequency;
    
    if(adc_clk_src_frequency > MAX_INPUT_ADC_CLK_FREQ)
    {
        touch_ret = TOUCH_INVALID_INPUT_PARAM;
    }
    else 
    {
        if (adc_clk_src_frequency > MAX_CORESRC_ADC_CLK_FREQ)
        {
            /* The ADC Core source Clock (FCORESRC)*/
            adc_core_src_frequency = adc_clk_src_frequency >> 1uL; /* divide by 2 */
        }
        else
        {
            /* The ADC Core source Clock (FCORESRC) */
            adc_core_src_frequency = adc_clk_src_frequency;
        }
       
        
        /*
        * Identifying the sccp drive delay and DMA transfer delay
        */
        /* minimum Shared ADC Core Clock Divider */
        core_clkdiv = 1u; /* divider = 2 when value is 0 or 1 */
        /* The Shared ADC Core Clock */
        core_adc_clk_freq = adc_core_src_frequency >> 1uL;

        /* loop until the required core divider is found */
        /* the actual divider is twice of the register set value */
        while (core_adc_clk_freq > (uint32_t)MAX_SHARED_ADC_CLK_FREQ )
        {
            /* increment the divider - actual divider is twice */
            core_clkdiv++;  
            /* divide by core divider - actual divider is twice */
            core_adc_clk_freq = (adc_core_src_frequency / (uint32_t)(core_clkdiv << 1uL));
        }
        
        /* Peripheral clock frequency(Fp) must not be lesser than the Core ADC frequency(FADCORE) */
        if(peripheral_clk_frequency < core_adc_clk_freq)
        {
            touch_ret = TOUCH_INVALID_INPUT_PARAM;
        }
        else
        {
            /* DMA transfer delay required in ADC core cycles  - 3 DMA clocks */ 
            device_config.dma_transfer_delay = (uint8_t)(ceil((((double)DEF_DMA_TRANSFER_TIME * (double)core_adc_clk_freq)/(double)peripheral_clk_frequency))) ;

            /* shield drive(sccp) delay required( in SCCP clocks)  = (dma_transfer_delay + DEF_PTG_STROBE_DELAY) ADC core clock */
            device_config.sccp_drive_delay = (uint8_t)(ceil(((double)(device_config.dma_transfer_delay + DEF_PTG_STROBE_TIME) * (double)(peripheral_clk_frequency)) /(double)core_adc_clk_freq));

            /* If the calculated delays are exact multiple of define values then there is a possibility of shield driven before start of charge share */
            delay_mod_val = (device_config.sccp_drive_delay % DEF_PTG_STROBE_TIME) | (device_config.dma_transfer_delay % DEF_DMA_TRANSFER_TIME);
            if(delay_mod_val == 0u)
            {
                /* Incrementing SCCP delay by one to avoid shield driven before charge share */
                device_config.sccp_drive_delay  += 1u;
            }
        }
    }
    
    return touch_ret;
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
    TMR1_Stop();
    TMR1_TimeoutCallbackRegister(touch_timer_handler); 
    TMR1_Start();

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
   touch_ret_t touch_ret = TOUCH_SUCCESS;
   
   /* set device configurations */
    touch_ret = touch_device_config();
    
     if (TOUCH_SUCCESS != touch_ret)
    {
        /* Acq module Error Detected */
        qtm_error_callback(0x81);
    }
    
    touch_timer_config();

	    /* Configure touch sensors with Application specific settings */
    touch_ret = touch_sensors_config();
     if (TOUCH_SUCCESS != touch_ret)
        {
            /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0x81);
        }
    
    /* Set Freq Hop Time scale */
	touch_ret = qtm_update_acq_freq_delay(qtlib_acq_set1.qtm_acq_node_group_config->freq_option_select, DEF_FREQ_HOP_TIME_SCALE);
    if (TOUCH_SUCCESS != touch_ret)
    {
            /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0x81);
    }
	
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
    if ((time_to_measure_touch_flag == 1u) && (touch_in_progress == 0u) && (touch_postprocess_request==0u))
    {

        /* Do the acquisition */
        touch_ret = qtm_cvd_start_measurement_seq(&qtlib_acq_set1, qtm_measure_complete_callback);

        /* if the Acquistion request was successful then clear the request flag */
        if (TOUCH_SUCCESS == touch_ret) {
            /* Clear the Measure request flag */
			time_to_measure_touch_flag = 0u;
            /* set in progress flag */
            touch_in_progress = 1u;
        }
        else
        {
            /* Acq module Error Detected: Issue an Acq module common error code 0x80 */
            qtm_error_callback(0x81);
        }
    }
    /* check the flag for node level post processing */
    if (touch_postprocess_request == 1u){
        /* clear in progress flag */
        touch_in_progress = 0u;
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
ISR(PTG)
------------------------------------------------------------------------------
Purpose  :  Interrupt handler for PTG Interrupt
Input    :  none
Output   :  none
Notes    :  This interrupt is called when all CVD conversions for one channel are completed. 
============================================================================*/
void __attribute__((interrupt, no_auto_psv)) _PTG0Interrupt()
{
    /* get the channel number first. execute the handler and then read the scanA and scanB */
    volatile uint16_t channel_number = qtm_get_current_measure_channel();
    
    qtm_dspic33c_touch_handler_eoc();
    
    touch_acq_scanA_signals[channel_number] = qtm_cvd_get_scanA_signal();
    touch_acq_scanB_signals[channel_number] = qtm_cvd_get_scanB_signal();

}
#endif /* TOUCH_C */