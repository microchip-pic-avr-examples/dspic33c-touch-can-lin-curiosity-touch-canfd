/*******************************************************************************
  Touch Library 4.0.0 Release

  @Company
    Microchip Technology Inc.

  @File Name
    touch.h

  @Summary
    QTouch Modular Library

  @Description
    Provides configuation macros for touch library.
	
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

#ifndef TOUCH_H
#define TOUCH_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

/*----------------------------------------------------------------------------
 *     include files
 *----------------------------------------------------------------------------*/

#include "include/touch_api.h"

/**********************************************************/
/******************* Acquisition controls *****************/
/**********************************************************/
/* Defines the Measurement Time in milli seconds.
 * Range: 1 to 255.
 * Default value: 20.
 */
#define DEF_TOUCH_MEASUREMENT_PERIOD_MS 20

/* Defines the Type of sensor
 * Default value: NODE_MUTUAL.
 */

#define DEF_SENSOR_TYPE NODE_SELFCAP

        
/* The Partner ADC channel for CVD acquisition */ 
/* The pin must be from PortB or PortC or PortD */
/* This pin must be left unconnected */
/* Range - 0u to 31u */
#define CVD_PARTNER_ADC_CHANNEL            12 
/* Set default bootup acquisition frequency.
 * Range: FREQ_SEL_0 - FREQ_SEL_15 , FREQ_SEL_SPREAD
 * Default value: FREQ_SEL_0.
 */
#define DEF_SEL_FREQ_INIT FREQ_SEL_0
/*Set the scaling factor for frequency hop delay
 * Range: 1u - 255u
 * Default : 1u
 */        
#define DEF_FREQ_HOP_TIME_SCALE         1u

/* Set external clock frequency if primary oscillator with PLL is used.
 * Range: NA
 * Default value: None
 */    
#define EXT_CLK_FREQ                 8000000uL  /* External clock freq - 8MHz*/
        
/* 
 * Constants - Do not modify
 */
#define DEF_DMA_TRANSFER_TIME               3u /* DMA transfer time - 3 DMA clocks */
#define DEF_PTG_STROBE_TIME                 3u /* PTG strobe time - 3 ADC core clocks (TADCORE) */        
#define FRC_CLK_FREQ                    8000000uL  /* FRC clock freq - 8MHz*/
#define MAX_SHARED_ADC_CLK_FREQ         50000000uL /* Max allowed ADC shared core clock freq is 50MHz*/
#define MAX_INPUT_ADC_CLK_FREQ         500000000uL /* Max allowed ADC input clock freq is 500MHz */
#define MAX_CORESRC_ADC_CLK_FREQ       250000000uL /* Max allowed ADC core source freq is 250MHz */

/*----------------------------------------------------------------------------
 *     defines
 *----------------------------------------------------------------------------*/

/**********************************************************/
/***************** Node Params   ******************/
/**********************************************************/
/* Acquisition Set 1 */
/* Defines the number of sensor nodes in the acquisition set
 * Range: 1 to 65535.
 * Default value: 1
 */

#define DEF_NUM_CHANNELS (3)

/* Defines node parameter setting
 * {X-line, Y-line, Charge Share Delay(CSD), Digital Gain,
 * filter level}
 */
#define NODE_0_PARAMS                                                                                               \
{                                                                                                                  \
   X_NONE,  Y(22), 30, NODE_GAIN( GAIN_1), FILTER_LEVEL_16                  \
}
#define NODE_1_PARAMS                                                                                               \
{                                                                                                                  \
   X_NONE,  Y(9), 30, NODE_GAIN( GAIN_1), FILTER_LEVEL_16                  \
}
#define NODE_2_PARAMS                                                                                               \
{                                                                                                                  \
   X_NONE,  Y(31), 30, NODE_GAIN( GAIN_1), FILTER_LEVEL_16                  \
}

/**********************************************************/
/***************** Key Params   ******************/
/**********************************************************/
/* Defines the number of key sensors
 * Range: 1 to 65535.
 * Default value: 1
 */
#define DEF_NUM_SENSORS (3)

/* Defines Key Sensor setting
 * {Sensor Threshold, Sensor Hysterisis, Sensor AKS}
 */
#define KEY_0_PARAMS                                                                                            \
{                                                                                                              \
    50, HYST_25, NO_AKS_GROUP                       \
}
#define KEY_1_PARAMS                                                                                            \
{                                                                                                              \
    50, HYST_25, NO_AKS_GROUP                       \
}
#define KEY_2_PARAMS                                                                                            \
{                                                                                                              \
    50, HYST_25, NO_AKS_GROUP                       \
}

/* De-bounce counter for additional measurements to confirm touch detection
 * Range: 0 to 255.
 * Default value: 2.
 */
#define DEF_TOUCH_DET_INT 2

/* De-bounce counter for additional measurements to confirm away from touch signal
 * to initiate Away from touch re-calibration.
 * Range: 0 to 255.
 * Default value: 5.
 */
#define DEF_ANTI_TCH_DET_INT 5

/* Threshold beyond with automatic sensor recalibration is initiated.
 * Range: RECAL_100/ RECAL_50 / RECAL_25 / RECAL_12_5 / RECAL_6_25 / MAX_RECAL
 * Default value: RECAL_100.
 */
#define DEF_ANTI_TCH_RECAL_THRSHLD RECAL_50

/* Rate at which sensor reference value is adjusted towards sensor signal value
 * when signal value is greater than reference.
 * Units: 200ms
 * Range: 0-255
 * Default value: 20u = 4 seconds.
 */
#define DEF_TCH_DRIFT_RATE 20

/* Rate at which sensor reference value is adjusted towards sensor signal value
 * when signal value is less than reference.
 * Units: 200ms
 * Range: 0-255
 * Default value: 5u = 1 second.
 */
#define DEF_ANTI_TCH_DRIFT_RATE 5

/* Time to restrict drift on all sensor when one or more sensors are activated.
 * Units: 200ms
 * Range: 0-255
 * Default value: 20u = 4 seconds.
 */
#define DEF_DRIFT_HOLD_TIME 20

/* Set mode for additional sensor measurements based on touch activity.
 * Range: REBURST_NONE / REBURST_UNRESOLVED / REBURST_ALL
 * Default value: REBURST_UNRESOLVED
 */
#define DEF_REBURST_MODE REBURST_UNRESOLVED

/* Sensor maximum ON duration upon touch.
 * Range: 0-255
 * Default value: 0
 */
#define DEF_MAX_ON_DURATION 0


/**********************************************************/
/***************** Slider/Wheel Parameters ****************/
/**********************************************************/
/* Defines the number of scrollers (sliders or wheels)
 */
#define DEF_NUM_SCROLLERS 1

/* Defines scroller parameter setting
 * {touch_scroller_type, touch_start_key, touch_scroller_size,
 * SCROLLER_RESOL_DEADBAND(touch_scroller_resolution,touch_scroller_deadband), touch_scroller_hysterisis,
 * touch_scr_detect_threshold}
 * Configuring scr_detect_threshold: By default, scr_detect_threshold parameter should be
 * set equal to threshold value of the underlying keys. Then the parameter has to be tuned based on the actual contact
 * size of the touch when moved over the scroller. The contact size of the moving touch can be observed from
 * "contact_size" parameter on scroller runtime data structure.
 */
  	                                  \
#define SCROLLER_0_PARAMS                  \
{                                                                                                              \
     SCROLLER_TYPE_SLIDER, 0, 3,                            \
		SCROLLER_RESOL_DEADBAND(SCR_RESOL_8_BIT, SCR_DB_1_PERCENT), 8, 20\
}




/**********************************************************/
/***************** Communication - Data Streamer ******************/
/**********************************************************/
#define DEF_TOUCH_DATA_STREAMER_ENABLE 0u



/**********************************************************/


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif
// DOM-IGNORE-END
#endif // TOUCH_H
