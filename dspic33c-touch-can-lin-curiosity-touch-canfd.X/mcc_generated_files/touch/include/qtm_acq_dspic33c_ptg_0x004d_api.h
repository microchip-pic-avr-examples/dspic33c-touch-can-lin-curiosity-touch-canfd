/*******************************************************************************
  Touch Library

  Company:
    Microchip Technology Inc.

  File Name:
    qtm_acq_dspic33c_0x004d_api.h

  Summary:
    Touch Modular Library

  Description:
    API for acquisition module of dsPIC33C device family - DSPIC33C 0x004d
	
*******************************************************************************/

/*******************************************************************************
Copyright (c) Microchip Technology Inc.  All rights reserved.

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

#ifndef QTM_ACQ_DSPIC33C_API_H
#define	QTM_ACQ_DSPIC33C_API_H

#include "qtm_common_components_api.h"


/* X line bit position */
#define X_NONE 0u
#define X(n) ((uint32_t)(1uL << (n)))

/* Y line bit position */
#define Y(n) ((uint32_t)(1uL << (n)))


/* Combine Analog / Digital Gain */
#define NODE_GAIN(d) (uint8_t)(d)



/* Touch library Oversampling setting */
typedef enum tag_filter_level_t 
{
  FILTER_LEVEL_1,
  FILTER_LEVEL_2,
  FILTER_LEVEL_4,
  FILTER_LEVEL_8,
  FILTER_LEVEL_16,
  FILTER_LEVEL_32,
  FILTER_LEVEL_64
} filter_level_t;

/* Touch library GAIN setting */
typedef enum tag_gain_t 
{
  GAIN_1,
  GAIN_2,
  GAIN_4,
  GAIN_8,
  GAIN_16
} gain_t;

/**
 * acquisition frequency delay setting.

 * inserts "n" ADC core clock cycles between consecutive measurements
 * e.g.  FREQ_HOP_SEL_14 setting inserts 14 ADC core clock cycles.
 */
typedef enum tag_freq_config_sel_t 
{
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
  FREQ_SEL_15
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
    DSPIC33_CK512MP = 2u, /* dsPIC33CK512MPT608 family devices are included here, as there is no difference related to touch */
    DSPIC33_CK1024MP = 3u,
    DSPIC33_CK64MC = 4u,
    DSPIC33_CK256MC = 5u,
	DSPIC33_CH128MP_MASTER = 6u,
	DSPIC33_CH512MP_MASTER = 7u,
} dspic33c_device_id;

/*----------------------------------------------------------------------------
 * Structure Declarations
 *----------------------------------------------------------------------------*/

/* device information configuration */
typedef struct {
    adc_clock_src_t         adc_clk_src;            /* Selects the ADC clock source */
    uint32_t                adc_clk_src_freq;       /* selected clock source frequency(Hz) for touch, min-16MHz and max 500MHz */
    dspic33c_device_id      device_id;              /* Device Identifier */
    uint8_t                 partner_adc_channel;    /* partner reference ADC channel - must be from Port B,C or D */
    uint8_t                 dma_transfer_delay;     /* DMA transfer delay to start charge share */
    uint8_t                 sccp_drive_delay;       /* SCCP delay to drive partner and shield pin */
    uint16_t*               cvd_scan_data_ptr;      /* memory for storing the CVD scan data. Allocate memory of size (max oversampling of all channels * 2) */

} qtm_acq_dspic33c_device_config_t;

/* Node configuration */
typedef struct
{
 uint32_t node_xmask;           /* Selects the X channels for this node */
 uint32_t node_ymask;           /* Selects the Y channels for this node */
 uint16_t node_csd;              /* Charge Share Delay Range: 8 to 1023*/
 uint8_t node_gain;             /* Bits 7:4 = reserved, Bits 3:0 = Digital gain */
 uint8_t node_oversampling;     /* Accumulator setting - Number of oversamples per measurement */
}qtm_acq_dspic33c_node_config_t;

/* Node run-time data - Defined in common api as it will be used with all acquisition modules */

/* Node group configuration */
typedef struct
{
  uint16_t num_sensor_nodes;            /* Number of sensor nodes */
  uint8_t acq_sensor_type;              /* Self or mutual sensors */
  uint8_t freq_option_select;           /* Hop Frequency selected */
} qtm_acq_node_group_config_t;

/* Container structure for sensor group */
typedef struct
{
    qtm_acq_node_group_config_t    *qtm_acq_node_group_config;
    qtm_acq_dspic33c_node_config_t  *qtm_acq_node_config;
    qtm_acq_node_data_t            *qtm_acq_node_data;
    qtm_acq_dspic33c_device_config_t *qtm_acq_device_config;
} qtm_acquisition_control_t;

/*----------------------------------------------------------------------------
* prototypes
*----------------------------------------------------------------------------*/

/* Library prototypes */

/*============================================================================
touch_ret_t qtm_cvd_init_acquisition_module(qtm_acquisition_control_t* qtm_acq_control_ptr);
------------------------------------------------------------------------------
Purpose: Initialize the ADC, Assign pins, set scaling for CVD vs system clock
Input  : pointer to acquisition set,
Output : touch_ret_t: 
Notes  : qtm_cvd_init_acquisition module must be called ONLY once with a pointer to each config set
============================================================================*/
touch_ret_t qtm_cvd_init_acquisition_module(qtm_acquisition_control_t* qtm_acq_control_ptr);

/*============================================================================
touch_ret_t qtm_cvd_qtlib_assign_signal_memory(uint32_t* qtm_signal_raw_data_ptr);
------------------------------------------------------------------------------
Purpose: Assign raw signals pointer to array defined in application code
Input  : pointer to raw data array
Output  : touch_ret_t: 
Notes  : none
============================================================================*/
touch_ret_t qtm_cvd_qtlib_assign_signal_memory(uint32_t* qtm_signal_raw_data_ptr );

/*============================================================================
touch_ret_t qtm_enable_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number)
------------------------------------------------------------------------------
Purpose: Enables a sensor node for measurement
Input  : Node configurations pointer, node number
Output : touch_ret_t:
Notes  :
============================================================================*/
touch_ret_t qtm_enable_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number);

/*============================================================================
touch_ret_t qtm_disable_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number)
------------------------------------------------------------------------------
Purpose: Disables a sensor node for measurement
Input  : Node configurations pointer, node number
Output : touch_ret_t:
Notes  :
============================================================================*/
touch_ret_t qtm_disable_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number);

/*============================================================================
touch_ret_t qtm_calibrate_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number)
------------------------------------------------------------------------------
Purpose:  Marks a sensor node for calibration
Input  :  Node configurations pointer, node number
Output : touch_ret_t:
Notes  :
============================================================================*/
touch_ret_t qtm_calibrate_sensor_node(qtm_acquisition_control_t* qtm_acq_control_ptr, uint16_t qtm_which_node_number);

/*============================================================================
void qtm_cvd_de_init(void)
------------------------------------------------------------------------------
Purpose: Clear ADC Pin registers, set TOUCH_STATE_NULL
Input  : none
Output : none
Notes  : none
============================================================================*/
void qtm_cvd_de_init(void);

/*============================================================================
touch_ret_t qtm_cvd_start_measurement_seq(qtm_acquisition_control_t* qtm_acq_control_pointer, void (*measure_complete_callback) (void));
------------------------------------------------------------------------------
Purpose:  Loads touch configurations for first channel and start,  
Input  :  Node configurations pointer, measure complete callback pointer
Output : touch_ret_t:
Notes  :
============================================================================*/
touch_ret_t qtm_cvd_start_measurement_seq(qtm_acquisition_control_t* qtm_acq_control_pointer, void (*measure_complete_callback) (void));

/*============================================================================
touch_ret_t qtm_acquisition_process(void)
------------------------------------------------------------------------------
Purpose: Signal capture and processing
Input  : (Measured signals, config)
Output : touch_ret_t
Notes  : none
============================================================================*/
touch_ret_t qtm_acquisition_process(void);

/*============================================================================
touch_ret_t qtm_update_acq_freq_delay(uint8_t freq_hop_delay, uint8_t scale_factor)
------------------------------------------------------------------------------
Purpose: Sets the delay used as part of freqency hop
Input  : Frequency hop delay(adc clock cycles), multiplication factor for frequency hop delay
Output : touch_ret_t
Notes  : none
============================================================================*/
touch_ret_t qtm_update_acq_freq_delay(uint8_t freq_hop_delay, uint8_t scale_factor);

/*============================================================================
touch_ret_t qtm_cvd_config_update(qtm_acquisition_control_t* qtm_acq_control_ptr)
------------------------------------------------------------------------------
Purpose: To update the CVD Configurations
Input  : acquisition control pointer
Output : touch_ret_t
Notes  : If the PTG or DMA or SCCP or PPS is updated by application then application must call this API to update the CVD configurations before next measurement.    
       : This API shall not be called if acquisition is in progress
============================================================================*/
touch_ret_t qtm_cvd_config_update( qtm_acquisition_control_t *qtm_acq_control_ptr );
 
/*============================================================================
void qtm_dspic33c_touch_handler_eoc(void)
------------------------------------------------------------------------------
Purpose:  Captures  the  measurement,  starts  the  next  or  End  Of  Sequence  handler
Input    :  none
Output  :  touch_ret_t
Notes    :  none
============================================================================*/
void qtm_dspic33c_touch_handler_eoc(void);
 
/*============================================================================
void qtm_disable_touch(void)
------------------------------------------------------------------------------
Purpose:  Turn off touch module
Input  :  none
Output : none
Notes  :
============================================================================*/
void qtm_disable_touch(void);

/*============================================================================
void qtm_enable_touch(void)
------------------------------------------------------------------------------
Purpose:  Turn ON touch module
Input  :  none
Output : none
Notes  :
============================================================================*/
void qtm_enable_touch(void);

 /*============================================================================
uint16_t  qtm_get_current_measure_channel(void)
------------------------------------------------------------------------------
Purpose: get the last measured channel
Input  : none
Output : none
Notes  : none
============================================================================*/
uint16_t  qtm_get_current_measure_channel(void);

/*============================================================================
uint8_t qtm_cvd_get_port_pin_map(uint8_t channel_number)
------------------------------------------------------------------------------
Purpose:  Returns the port and pin info for the ADC channel
Input  :  ADC channel number
Output :  Port (Bits 7:5), Pin (Bits 4:0)
Notes  : This API must not be called before calling the qtm_cvd_init_acquisition_module API
============================================================================*/
uint8_t qtm_cvd_get_port_pin_map(uint8_t channel_number);

/*============================================================================
uint16_t  qtm_cvd_get_scanA_signal()
------------------------------------------------------------------------------
Purpose: get scanA of last measurement
Input  : none
Output : 
Notes  : none
============================================================================*/
uint16_t  qtm_cvd_get_scanA_signal(void);
         
/*============================================================================
uint16_t  qtm_cvd_get_scanB_signal()
------------------------------------------------------------------------------
Purpose: get scanB of last measurement
Input  : none
Output : 
Notes  : none
============================================================================*/        
uint16_t  qtm_cvd_get_scanB_signal(void);

/*============================================================================
uint16_t qtm_dspic33c_acq_module_get_id(void)
------------------------------------------------------------------------------
Purpose: Returns the module ID
Input  : none
Output : Module ID
Notes  : none
============================================================================*/
uint16_t qtm_dspic33c_acq_module_get_id(void);

/*============================================================================
uint8_t qtm_dspic33c_acq_module_get_ver(void)
------------------------------------------------------------------------------
Purpose: Returns the module Firmware version
Input  : none
Output : Module ID - Upper nibble major / Lower nibble minor
Notes  : none
============================================================================*/
uint8_t qtm_dspic33c_acq_module_get_ver(void);

/*----------------------------------------------------------------------------
 *     device identifier definitions
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
/* dsPIC33CK512MPT608 family devices are included here, as there is no difference related to touch */
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

#elif   defined (__dsPIC33CH64MP202__) || defined (__dsPIC33CH64MP203__) || defined (__dsPIC33CH64MP205__) ||defined (__dsPIC33CH64MP206__) ||defined (__dsPIC33CH64MP208__) || \
        defined (__dsPIC33CH64MP502__) || defined (__dsPIC33CH64MP503__) || defined (__dsPIC33CH64MP505__) ||defined (__dsPIC33CH64MP506__) ||defined (__dsPIC33CH64MP508__) || \
        defined (__dsPIC33CH128MP202__) || defined (__dsPIC33CH128MP203__) || defined (__dsPIC33CH128MP205__) ||defined (__dsPIC33CH128MP206__) ||defined (__dsPIC33CH128MP208__) || \
        defined (__dsPIC33CH128MP502__) || defined (__dsPIC33CH128MP503__) || defined (__dsPIC33CH128MP505__) ||defined (__dsPIC33CH128MP506__) ||defined (__dsPIC33CH128MP508__)

#define DSPIC33_DEVICE_IDENTIFIER  DSPIC33_CH128MP_MASTER

#elif   defined (__dsPIC33CH256MP205__) ||  defined (__dsPIC33CH256MP206__) ||  defined (__dsPIC33CH256MP208__)  || \
        defined (__dsPIC33CH256MP505__) ||  defined (__dsPIC33CH256MP506__) ||  defined (__dsPIC33CH256MP508__)  || \
        defined (__dsPIC33CH512MP205__) ||  defined (__dsPIC33CH512MP206__) ||  defined (__dsPIC33CH512MP208__) || \
        defined (__dsPIC33CH512MP505__) ||  defined (__dsPIC33CH512MP506__) ||  defined (__dsPIC33CH512MP508__) 

#define DSPIC33_DEVICE_IDENTIFIER  DSPIC33_CH512MP_MASTER

#endif

#endif /* QTM_ACQ_DSPIC33C_API_H */









