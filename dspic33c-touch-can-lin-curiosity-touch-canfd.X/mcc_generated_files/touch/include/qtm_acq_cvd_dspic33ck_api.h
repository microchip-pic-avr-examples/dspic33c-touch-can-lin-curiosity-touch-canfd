/*============================================================================
Filename : qtm_acq_cvd_dspic33ck_api.h
Project : QTouch Modular Library
Purpose : dsPIC33 Acquisition module - DSPIC33CK 0x004b
------------------------------------------------------------------------------
Copyright (c) 2022 Microchip. All rights reserved.
============================================================================*/


#ifndef QTM_ACQ_CVD_DSPIC33CK_API_H
#define	QTM_ACQ_CVD_DSPIC33CK_API_H

#include "qtm_common_components_api.h"


/* X line bit position */
#define X_NONE 0u
#define X(n) ((uint32_t)(1uL << (n)))

/* Y line bit position */
#define Y(n) ((uint32_t)(1uL << (n)))

/* Extract Digital Gain */
#define NODE_GAIN_DIG(m) (uint8_t)(m)
/* Combine Analog / Digital Gain */
#define NODE_GAIN(d) (uint8_t)(d)

typedef enum tag_filter_level_t {
    FILTER_LEVEL_1,
    FILTER_LEVEL_2,
    FILTER_LEVEL_4,
    FILTER_LEVEL_8,
    FILTER_LEVEL_16,
    FILTER_LEVEL_32,
    FILTER_LEVEL_64,
    FILTER_LEVEL_128
} filter_level_t;

/**
 * acquisition frequency delay setting.

 * inserts "n" uS between consecutive measurements
 * e.g.  FREQ_HOP_SEL_14 setting inserts 14 uS.
 */
typedef enum tag_freq_config_sel_t {
    FREQ_SEL_0,
    FREQ_SEL_1,
    FREQ_SEL_2,
    FREQ_SEL_3,
    FREQ_SEL_4,
    FREQ_SEL_5,
    FREQ_SEL_6,
    FREQ_SEL_7,
    FREQ_SEL_8,
    FREQ_SEL_9,
    FREQ_SEL_10,
    FREQ_SEL_11,
    FREQ_SEL_12,
    FREQ_SEL_13,
    FREQ_SEL_14,
    FREQ_SEL_15,
} freq_config_sel_t;

typedef enum tagADCCLKSRC 
{
    PERIPHERAL_CLK = 0u,
    FOSC = 1u,
    AFVCODIV = 2u,
    FVCO_DIV4 = 3u,
    FVCO_DIV3 = 4u,
    INSTRUCTION_CLK = 5u
} adc_clock_src_t;

typedef enum tagDeviceID {
    DSPIC33_CK64MP = 0u,
    DSPIC33_CK256MP = 1u,
    DSPIC33_CK512MP = 2u,
    DSPIC33_CK1024MP = 3u,
    DSPIC33_CK64MC = 4u,
    DSPIC33_CK256MC = 5u
} dspic33_device_id;


#define ADC_INTERRUPT_PRIORITY 1

/*----------------------------------------------------------------------------
 * Structure Declarations
 *----------------------------------------------------------------------------*/

/* device information configuration */
typedef struct {
    adc_clock_src_t clk_src;        /* Selects the ADC clock source */
    uint32_t    clk_freq;           /* frequency Hz for touch timing */
    dspic33_device_id device_id;    /* Device Selection */
} qtm_cvd_acq_dspic33ck_device_config_t;

/* Node configuration */
typedef struct {
    uint32_t node_xmask; /* Selects the X Pins for this node */
    uint32_t node_ymask; /* Selects the Y Pins for this node */
    uint8_t node_csd; /* Charge Share Delay */
    uint8_t node_gain; /* Bits 7:4 = Analog gain, Bits 3:0 = Digital gain */
    uint8_t node_oversampling; /* Accumulator setting */
} qtm_cvd_acq_dspic33ck_node_config_t;

/* Node run-time data - Defined in common api as it will be used with all acquisition modules */

/* Node group configuration */
typedef struct qtm_acq_node_group_config_type {
    uint16_t num_sensor_nodes; /* Number of sensor nodes */
    uint8_t acq_sensor_type; /* Self or mutual sensors */
    uint8_t freq_option_select; /* SDS or ASDV setting */
} qtm_acq_node_group_config_t;

/* Container structure for sensor group */
typedef struct {
    qtm_acq_node_group_config_t* qtm_acq_node_group_config;
    qtm_cvd_acq_dspic33ck_node_config_t* qtm_acq_node_config;
    qtm_acq_node_data_t* qtm_acq_node_data;
    qtm_cvd_acq_dspic33ck_device_config_t* qtm_acq_device_info;
} qtm_acquisition_control_t;

typedef struct {
    qtm_acquisition_control_t* qtm_acq_control;
    uint16_t auto_scan_node_number;
    uint8_t auto_scan_node_threshold;
    uint8_t auto_scan_trigger;
} qtm_auto_scan_config_t;

/* Touch library GAIN setting */
typedef enum tag_gain_t {
    GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_16, GAIN_32
} gain_t;

/* Library prototypes */
/*============================================================================
touch_ret_t qtm_cvd_init_acquisition_module(qtm_acquisition_control_t* qtm_acq_control_ptr)
------------------------------------------------------------------------------
Purpose: Initialize the ADC, Assign pins, set scaling for CVD vs system clock
Input  : pointer to acquisition set,
Output : touch_ret_t: TOUCH_SUCCESS or INVALID_PARAM
Notes  : qtm_cvd_init_acquisition module must be called ONLY once with a pointer to each config set
============================================================================*/
touch_ret_t qtm_cvd_init_acquisition_module(qtm_acquisition_control_t* qtm_acq_control_ptr);

/*============================================================================
touch_ret_t qtm_cvd_qtlib_assign_signal_memory(uint16_t* qtm_signal_raw_data_ptr, uint16_t* qtm_signal_scanA_ptr, uint16_t* qtm_signal_scanB_ptr)
------------------------------------------------------------------------------
Purpose : Assign raw signals pointer to array defined in application code
Input   : pointer to raw data array
Output  : touch_ret_t: TOUCH_SUCCESS or INVALID_POINTER
Notes   : set NULL to scanA and scanB memory pointer if not required
============================================================================*/
touch_ret_t qtm_cvd_qtlib_assign_signal_memory(uint16_t* qtm_signal_raw_data_ptr, uint16_t* qtm_signal_scanA_ptr, uint16_t* qtm_signal_scanB_ptr);

/*============================================================================
touch_ret_t qtm_enable_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number)
------------------------------------------------------------------------------
Purpose: Enables a sensor node for measurement
Input  : Node configurations pointer, node (channel) number
Output : touch_ret_t:
Notes  :
============================================================================*/
touch_ret_t qtm_enable_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number);

/*============================================================================
touch_ret_t qtm_calibrate_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number)
------------------------------------------------------------------------------
Purpose: Marks a sensor node for calibration
Input  : Node configurations pointer, node (channel) number
Output : touch_ret_t:
Notes  :
============================================================================*/
touch_ret_t qtm_calibrate_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number);

/*============================================================================
void qtm_cvd_de_init(void)
------------------------------------------------------------------------------
Purpose: Clear PTC Pin registers, set TOUCH_STATE_NULL
Input  : none
Output : none
Notes  : none
============================================================================*/
void qtm_cvd_de_init(void);

/*============================================================================
touch_ret_t qtm_cvd_start_measurement_seq(qtm_acquisition_control_t* qtm_acq_control_pointer, void (*measure_complete_callback) (void));
------------------------------------------------------------------------------
Purpose: Loads touch configurations for first channel and start,
Input  : Node configurations pointer, measure complete callback pointer
Output : touch_ret_t:
Notes  :
============================================================================*/
touch_ret_t qtm_cvd_start_measurement_seq(qtm_acquisition_control_t* qtm_acq_control_pointer, void (*measure_complete_callback)(void));

/*============================================================================
touch_ret_t qtm_acquisition_process(void)
------------------------------------------------------------------------------
Purpose: Signal capture and processing
Input  : (Measured signals, config)
Output : TOUCH_SUCCESS or TOUCH_CAL_ERROR
Notes  : none
============================================================================*/
touch_ret_t qtm_acquisition_process(void);

/*============================================================================
touch_ret_t qtm_update_acq_freq_delay(void)
------------------------------------------------------------------------------
Purpose: Sets the delay used as part of freqency hop
Input  : Frequency hop->current frequency.
Output : TOUCH_SUCCESS or TOUCH_CAL_ERROR
Notes  : none
============================================================================*/
touch_ret_t qtm_update_acq_freq_delay(uint8_t update_delay);

/*============================================================================
void qtm_dspic33_touch_handler_eoc(void)
------------------------------------------------------------------------------
Purpose: determines which channel triggered ADC interrupt, 
         updates touch data,clears channel interrupt flags
Input  : none
Output : none
Notes  : none
============================================================================*/
void qtm_dspic33_touch_handler_eoc(void);

/*============================================================================
touch_ret_t get_sensor_node_signalA(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t sensor_node, uint16_t *signalPtr)
------------------------------------------------------------------------------
Purpose: To Get the scanA signal of the sensor node
Input  : Node configurations pointer, node (channel) number, pointer to store signal
Output : touch_ret_t
Notes  : signal will be stored in the memory pointed by signalPtr 
============================================================================*/
touch_ret_t get_sensor_node_signalA(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t sensor_node, uint16_t *signalPtr);

/*============================================================================
touch_ret_t get_sensor_node_signalB(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t sensor_node, uint16_t *signalPtr)
------------------------------------------------------------------------------
Purpose: To get the scanB signal of the sensor node
Input  : Node configurations pointer, node (channel) number, pointer to store signal
Output : touch_ret_t
Notes  : signal will be stored in the memory pointed by signalPtr
============================================================================*/
touch_ret_t get_sensor_node_signalB(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t sensor_node, uint16_t *signalPtr);

/*============================================================================
uint8_t GetXY_PortPinMap(uint8_t TouchLine)
------------------------------------------------------------------------------
Purpose:  Returns the port and pin info for the touch line
Input  :  XY Pin number
Output :  Port (Bits 7:5), Pin (Bits 4:0)
Notes  : none
============================================================================*/
uint8_t GetXY_PortPinMap(uint8_t TouchLine);

/*============================================================================
void qtm_disable_touch_adc(void)
------------------------------------------------------------------------------
Purpose:  Turn off touch ADC module
Input  :  none
Output : none
Notes  :
============================================================================*/
void qtm_disable_touch_adc(void);


/*============================================================================
void qtm_enable_touch_adc(void)
------------------------------------------------------------------------------
Purpose:  Turn ON touch ADC module
Input  :  none
Output : none
Notes  :
============================================================================*/
void qtm_enable_touch_adc(void);


/*============================================================================
uint16_t qtm_dspic33ck_acq_module_get_id(void)
------------------------------------------------------------------------------
Purpose: Returns the module ID
Input  : none
Output : Module ID
Notes  : none
============================================================================*/
uint16_t qtm_dspic33ck_acq_module_get_id(void);

/*============================================================================
uint8_t qtm_dspic33ck_acq_module_get_version(void)
------------------------------------------------------------------------------
Purpose: Returns the module Firmware version
Input  : none
Output : Module ID - Upper nibble major / Lower nibble minor
Notes  : none
============================================================================*/
uint8_t qtm_dspic33ck_acq_module_get_version(void);


/*----------------------------------------------------------------------------
 *     device definitions
 *----------------------------------------------------------------------------*/
#if     defined (__dsPIC33CK32MP102__) || defined (__dsPIC33CK32MP103__) || defined (__dsPIC33CK32MP105__) || \
        defined (__dsPIC33CK64MP102__) || defined (__dsPIC33CK64MP103__) || defined (__dsPIC33CK64MP105__) 

#define DSPIC33_DEVICE_IDENTIFIER   DSPIC33_CK64MP

#elif   defined (__dsPIC33CK32MP202__) || defined(__dsPIC33CK32MP203__)|| defined(__dsPIC33CK32MP205__) || defined(__dsPIC33CK32MP206__) || \
        defined (__dsPIC33CK64MP202__)||defined(__dsPIC33CK64MP203__)||defined(__dsPIC33CK64MP205__)||defined(__dsPIC33CK64MP206__)||defined(__dsPIC33CK64MP208__)|| \
        defined (__dsPIC33CK128MP202__)||defined(__dsPIC33CK128MP203__)||defined(__dsPIC33CK128MP205__)||defined(__dsPIC33CK128MP206__)||defined(__dsPIC33CK128MP208__)|| \
        defined (__dsPIC33CK256MP202__)||defined(__dsPIC33CK256MP203__)||defined(__dsPIC33CK256MP205__)||defined(__dsPIC33CK256MP206__)||defined(__dsPIC33CK256MP208__)|| \
        defined (__dsPIC33CK32MP502__) || defined(__dsPIC33CK32MP503__)|| defined(__dsPIC33CK32MP505__) || defined(__dsPIC33CK32MP506__) || \
        defined (__dsPIC33CK64MP502__)||defined(__dsPIC33CK64MP503__)||defined(__dsPIC33CK64MP505__)||defined(__dsPIC33CK64MP506__)||defined(__dsPIC33CK64MP508__)|| \
        defined (__dsPIC33CK128MP502__)||defined(__dsPIC33CK128MP503__)||defined(__dsPIC33CK128MP505__)||defined(__dsPIC33CK128MP506__)||defined(__dsPIC33CK128MP508__)|| \
        defined (__dsPIC33CK256MP502__)||defined(__dsPIC33CK256MP503__)||defined(__dsPIC33CK256MP505__)||defined(__dsPIC33CK256MP506__)||defined(__dsPIC33CK256MP508__)

#define DSPIC33_DEVICE_IDENTIFIER   DSPIC33_CK256MP

#elif   defined(__dsPIC33CK256MP305__)||defined(__dsPIC33CK256MP306__)||defined(__dsPIC33CK256MP308__) || \
        defined(__dsPIC33CK512MP305__)||defined(__dsPIC33CK512MP306__)||defined(__dsPIC33CK512MP308__) || \
        defined(__dsPIC33CK256MP605__)||defined(__dsPIC33CK256MP606__)||defined(__dsPIC33CK256MP608__) || defined(__dsPIC33CK256MPT608__) || \
        defined(__dsPIC33CK512MP605__)||defined(__dsPIC33CK512MP606__)||defined(__dsPIC33CK512MP608__) || defined(__dsPIC33CK512MPT608__)

#define DSPIC33_DEVICE_IDENTIFIER  DSPIC33_CK512MP

#elif   defined(__dsPIC33CK256MP405__)||defined(__dsPIC33CK256MP406__)||defined(__dsPIC33CK256MP408__) || defined(__dsPIC33CK256MP410__) || \
        defined(__dsPIC33CK512MP405__)||defined(__dsPIC33CK512MP406__)||defined(__dsPIC33CK512MP408__) || defined(__dsPIC33CK512MP410__) || \
        defined(__dsPIC33CK1024MP405__)||defined(__dsPIC33CK1024MP406__)||defined(__dsPIC33CK1024MP408__) || defined(__dsPIC33CK1024MP410__) || \
        defined(__dsPIC33CK256MP705__)||defined(__dsPIC33CK256MP706__)||defined(__dsPIC33CK256MP708__) || defined(__dsPIC33CK256MP710__) || \
        defined(__dsPIC33CK512MP705__)||defined(__dsPIC33CK512MP706__)||defined(__dsPIC33CK512MP708__) || defined(__dsPIC33CK512MP710__) || \
        defined(__dsPIC33CK1024MP705__)||defined(__dsPIC33CK1024MP706__)||defined(__dsPIC33CK1024MP708__) || defined(__dsPIC33CK1024MP710__)

#define DSPIC33_DEVICE_IDENTIFIER  DSPIC33_CK1024MP

#elif   defined (__dsPIC33CK32MC102__) || defined(__dsPIC33CK32MC103__)|| defined(__dsPIC33CK32MC105__) || \
        defined (__dsPIC33CK64MC102__) || defined(__dsPIC33CK64MC103__)|| defined(__dsPIC33CK64MC105__)

#define DSPIC33_DEVICE_IDENTIFIER  DSPIC33_CK64MC

#elif   defined (__dsPIC33CK128MC102__) || defined(__dsPIC33CK128MC103__)|| defined(__dsPIC33CK128MC105__) || defined(__dsPIC33CK128MC106__) || \
        defined (__dsPIC33CK256MC102__) || defined(__dsPIC33CK256MC103__)|| defined(__dsPIC33CK256MC105__) || defined(__dsPIC33CK256MC106__) || \
        defined (__dsPIC33CK128MC502__) || defined(__dsPIC33CK128MC503__)|| defined(__dsPIC33CK128MC505__) || defined(__dsPIC33CK128MC506__) || \
        defined (__dsPIC33CK256MC502__) || defined(__dsPIC33CK256MC503__)|| defined(__dsPIC33CK256MC505__) || defined(__dsPIC33CK256MC506__) 

#define DSPIC33_DEVICE_IDENTIFIER  DSPIC33_CK256MC 

#endif
 

#endif	/* QTM_ACQ_CVD_DSPIC33CK_API_H */

