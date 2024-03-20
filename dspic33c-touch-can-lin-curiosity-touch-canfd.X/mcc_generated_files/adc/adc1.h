/**
 * ADC1 Generated Driver Header File
 * 
 * @file      adc1.h
 *            
 * @ingroup   adcdriver
 *            
 * @brief     This is the generated driver header file for the ADC1 driver
 *            
 * @skipline @version   Firmware Driver Version 1.4.4
 *
 * @skipline @version   PLIB Version 2.4.3
 *            
 * @skipline  Device : dsPIC33CK1024MP710
*/

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

#ifndef ADC1_H
#define ADC1_H

// Section: Included Files

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "adc_types.h"
#include "adc_interface.h"
#ifdef __cplusplus  //Provide C++ Compatibility

    extern "C" {

#endif

// Section: Data Types

/** 
  @ingroup  adcdriver
  @brief    Defines the scan option selection done for the shared channels
*/
#define ADC1_SCAN_MODE_SELECTED true

/** 
  @ingroup  adcdriver
  @brief    Defines the ADC Resolution
*/
#define ADC1_RESOLUTION 12

// Section: Data Type Definitions

/**
 * @ingroup  adcdriver
 * @brief    Structure object of type ADC_INTERFACE with the custom name
 *           given by the user in the Melody Driver User interface. The default name 
 *           e.g. ADC1 can be changed by the user in the ADC user interface. 
 *           This allows defining a structure with application specific name using 
 *           the 'Custom Name' field. Application specific name allows the API Portability.
*/
extern const struct ADC_INTERFACE ADC;

/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Initialize API
 */
#define ADC_Initialize ADC1_Initialize
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Deinitialize API
 */
#define ADC_Deinitialize ADC1_Deinitialize
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Enable API
 */
#define ADC_Enable ADC1_Enable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Disable API
 */
#define ADC_Disable ADC1_Disable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_SoftwareTriggerEnable API
 */
#define ADC_SoftwareTriggerEnable ADC1_SoftwareTriggerEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_SoftwareTriggerDisable API
 */
#define ADC_SoftwareTriggerDisable ADC1_SoftwareTriggerDisable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ChannelSelect API
 */
#define ADC_ChannelSelect ADC1_ChannelSelect
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ConversionResultGet API
 */
#define ADC_ConversionResultGet ADC1_ConversionResultGet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IsConversionComplete API
 */
#define ADC_IsConversionComplete ADC1_IsConversionComplete
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ResolutionSet API
 */
#define ADC_ResolutionSet ADC1_ResolutionSet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptEnable API
 */
#define ADC_InterruptEnable ADC1_InterruptEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptDisable API
 */
#define ADC_InterruptDisable ADC1_InterruptDisable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptFlagClear API
 */
#define ADC_InterruptFlagClear ADC1_InterruptFlagClear
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptPrioritySet API
 */
#define ADC_InterruptPrioritySet ADC1_InterruptPrioritySet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_CommonCallbackRegister API
 */
#define ADC_CommonCallbackRegister ADC1_CommonCallbackRegister
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Tasks API
 */
#define ADC_Tasks ADC1_Tasks
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptEnable API
 */
#define ADC_IndividualChannelInterruptEnable ADC1_IndividualChannelInterruptEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptDisable API
 */
#define ADC_IndividualChannelInterruptDisable ADC1_IndividualChannelInterruptDisable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptFlagClear API
 */
#define ADC_IndividualChannelInterruptFlagClear ADC1_IndividualChannelInterruptFlagClear
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptPrioritySet API
 */
#define ADC_IndividualChannelInterruptPrioritySet ADC1_IndividualChannelInterruptPrioritySet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ChannelCallbackRegister API
 */
#define ADC_ChannelCallbackRegister ADC1_ChannelCallbackRegister
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ComparatorCallbackRegister API
 */
#define ADC_ComparatorCallbackRegister ADC1_ComparatorCallbackRegister
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ChannelTasks API
 */
#define ADC_ChannelTasks ADC1_ChannelTasks
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_CorePowerEnable API
 */
#define ADC_CorePowerEnable ADC1_CorePowerEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_SharedCorePowerEnable API
 */
#define ADC_SharedCorePowerEnable ADC1_SharedCorePowerEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_PWMTriggerSourceSet API
 */
#define ADC_PWMTriggerSourceSet ADC1_PWMTriggerSourceSet

// Section: Driver Interface Functions

/**
 * @ingroup  adcdriver
 * @brief    Initializes ADC1 module, using the given initialization data
 *           This function must be called before any other ADC1 function is called
 * @param    none
 * @return   none  
 */
void ADC1_Initialize (void);

/**
 * @ingroup  adcdriver
 * @brief    Deinitializes the ADC1 to POR values
 * @param    none
 * @return   none  
 */
void ADC1_Deinitialize(void);

/**
 * @ingroup  adcdriver
 * @brief    This inline function enables the ADC1 module
 * @pre      \ref ADC1_Initialize function should have been 
 *           called  before calling this function.
 * @param    none
 * @return   none  
 */
inline static void ADC1_Enable(void)
{
   ADCON1Lbits.ADON = 1;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function disables the ADC1 module
 * @pre      \ref ADC1_Initialize function should have been 
 *           called  before calling this function.
 * @param    none
 * @return   none  
 */
inline static void ADC1_Disable(void)
{
   ADCON1Lbits.ADON = 0;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function sets software common trigger
 * @pre      \ref ADC1_Initialize function should have been 
 *           called before calling this function.
 * @param    none
 * @return   none  
 */
inline static void ADC1_SoftwareTriggerEnable(void)
{
   ADCON3Lbits.SWCTRG = 1;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function resets software common trigger
 * @pre      ADC1_Initialize function should have been 
 *           called before calling this function.
 * @param    none
 * @return   none  
 */
inline static void ADC1_SoftwareTriggerDisable(void)
{
   ADCON3Lbits.SWCTRG = 0;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function allows selection of a channel for conversion
 * @pre        \ref ADC1_Initialize() function should have been
 *             called before calling this function.
 * @param[in]  channel - Channel for conversion  
 * @return     none  
 */
inline static void ADC1_ChannelSelect( enum ADC_CHANNEL channel )
{
    //This function does not have any implementation since 
    //Shared channels are selected from UI.
    //Dedicated channels are selected from UI.
    
    (void)channel;
}

/**
 * @ingroup    adcdriver
 * @brief      Returns the conversion value for the channel selected
 * @pre        This inline function returns the conversion value only after the conversion is complete. 
 *             Conversion completion status can be checked using 
 *             \ref ADC1_IsConversionComplete(channel) function.
 * @param[in]  channel - Selected channel  
 * @return     Returns the analog to digital converted value  
 */
inline static uint16_t ADC1_ConversionResultGet( enum ADC_CHANNEL channel )
{
    uint16_t result = 0x0U;

    switch(channel)
    {
        case potentiometer:
                result = ADCBUF0;
                break;
        default:
                break;
    }
    return result;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function returns the status of conversion.This function is used to 
 *             determine if conversion is completed. When conversion is complete 
 *             the function returns true otherwise false.
 * 
 * @pre        \ref ADC1_SoftwareTriggerEnable() function should have been 
 *             called before calling this function.
 * @param[in]  channel - Selected channel  
 * @return     true - Conversion is complete.
 * @return     false - Conversion is not complete.  
 */
inline static bool ADC1_IsConversionComplete(enum ADC_CHANNEL channel)
{
    bool status = false;

    switch(channel)
    {
        case potentiometer:
                status = ADSTATLbits.AN0RDY;
                break;
        default:
                break;
    }

    return status;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function helps to configure all cores with same resolution
 * 
 * @param[in]  resolution - Resolution type  
 * @return     none 
 */
inline static void ADC1_ResolutionSet(enum ADC_RESOLUTION_TYPE resolution)
{
   ADCORE0Hbits.RES = resolution;
   ADCON1Hbits.SHRRES = resolution;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function enables the ADC1 interrupt
 * @param    none
 * @return   none  
 */
inline static void ADC1_InterruptEnable(void)
{
    IEC5bits.ADCIE = 1;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function disables the ADC1 interrupt
 * @param    none
 * @return   none  
 */
inline static void ADC1_InterruptDisable(void)
{
    IEC5bits.ADCIE = 0;
}

/**
 * @ingroup  adcdriver
 * @brief    Clears interrupt flag manually
 * @param    none
 * @return   none  
 */
inline static void ADC1_InterruptFlagClear(void)
{
    IFS5bits.ADCIF = 0;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function allows selection of priority for interrupt
 * @param[in]  priorityValue  -  The numerical value of interrupt priority
 * @return     none  
 */
inline static void ADC1_InterruptPrioritySet( uint16_t priorityValue )
{
    IPC22bits.ADCIP = (uint16_t)0x7 & priorityValue;
}

/**
 * @ingroup    adcdriver
 * @brief      This function can be used to override default callback and to 
 *             define custom callback for ADC1 Common event
 * @param[in]  callback - Address of the callback function.  
 * @return     none  
 *             
 */
void ADC1_CommonCallbackRegister(void(*callback)(void));

/**
 * @ingroup  adcdriver
 * @brief    This is the default callback with weak attribute. The user can override and implement the default callback without weak attribute
 *           or can register a custom callback function using  ADC1_CommonCallbackRegister
 * @param    none
 * @return   none  
 */
void ADC1_CommonCallback(void);

 
/**
 * @ingroup  adcdriver
 * @brief    This function is used to implement the tasks for polled implementations
 * @pre      \ref ADC1_Initialize() function should have been
 *           called before calling this function.
 * @param    none
 * @return   none
 * @note     This function has to be polled to notify callbacks and clear 
 *           the interrupt flags in non-interrupt mode of ADC
 */
void ADC1_Tasks(void);

/**
 * @ingroup    adcdriver
 * @brief      This inline function enables individual channel interrupt
 * @param[in]  channel - Selected channel  
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptEnable(enum ADC_CHANNEL channel)
{
    switch(channel)
    {
        case potentiometer:
                IEC5bits.ADCAN0IE = 1;
                ADIELbits.IE0 = 1;
                break;
        default:
                break;
    }
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function disables individual channel interrupt
 * @param[in]  channel - Selected channel  
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptDisable(enum ADC_CHANNEL channel)
{
    switch(channel)
    {
        case potentiometer:
                IEC5bits.ADCAN0IE = 0;
                ADIELbits.IE0 = 0;
                break;
        default:
                break;
    }
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function clears individual channel interrupt flag
 * @param[in]  channel - Selected channel  
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptFlagClear(enum ADC_CHANNEL channel)
{
    switch(channel)
    {
        case potentiometer:
                IFS5bits.ADCAN0IF = 0;
                break;
        default:
                break;
    }
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function allows selection of priority for individual channel interrupt
 * @param[in]  channel - Selected channel 
 * @param[in]  priorityValue  -  The numerical value of interrupt priority
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptPrioritySet(enum ADC_CHANNEL channel, enum INTERRUPT_PRIORITY priorityValue)
{
	switch(channel)
	{
		case potentiometer:
				IPC22bits.ADCAN0IP = priorityValue;
				break;
		default:
				break;
	}
}

/**
 * @ingroup    adcdriver
 * @brief      This function can be used to override default callback \ref ADC1_ChannelCallback
 *             and to define custom callback for ADC1 Channel event. 
 *             Read the conversion result of the corresponding channel in the custom callback.
 * @param[in]  callback - Address of the callback function.  
 * @return     none  
 */
void ADC1_ChannelCallbackRegister(void(*callback)(enum ADC_CHANNEL channel, uint16_t adcVal));

/**
 * @ingroup    adcdriver
 * @brief      This is the default callback function for all the analog channels. 
 *             This callback is triggered once the channel conversion is done for a
 *             channel and to read the conversion result of the corresponding channel
 * @param[in]  channel - conversion completed channel
 * @param[in]  adcVal - conversion result of channel  
 * @return     none  
 */
void ADC1_ChannelCallback(enum ADC_CHANNEL channel, uint16_t adcVal);


/**
 * @ingroup    adcdriver
 * @brief      This function can be used to override default callback and to 
 *             define custom callback for ADC1_Comparator event
 * @param[in]  callback - Address of the callback function.  
 * @return     none  
 */
void ADC1_ComparatorCallbackRegister(void(*callback)(enum ADC_CMP comparator));

/**
 * @ingroup    adcdriver
 * @brief      Comparator callback function
 * @param[in]  comparator - comparator in which compare event occurred  
 * @return     none  
 */
void ADC1_ComparatorCallback(enum ADC_CMP comparator);


/**
 * @ingroup    adcdriver
 * @brief      This function call used only in polling mode, if channel 
 *             conversion is done for requested channel, the calls the 
 *             respective callback function
 * @pre        \ref ADC1_Initialize() function should have been  
 *             called before calling this function.
 * @param[in]  channel - Selected channel.  
 * @return     none  
 * @note       This function has to be polled to notify channel callbacks and clear 
 *             the channel interrupt flags in non-interrupt mode of ADC
 */
void ADC1_ChannelTasks(enum ADC_CHANNEL channel);


// Section: Interface functions: Dedicated Core

/**
 * @ingroup    adcdriver
 * @brief      Enables analog and digital power for ADC1 dedicated core
 * @param[in]  core - Selected core  
 * @return     none  
 */
void ADC1_CorePowerEnable(enum ADC_DEDICATED_CORE core);


/**
 * @ingroup  adcdriver
 * @brief    Enables power for ADC1 shared Core
 *           This function is used to set the analog and digital power for 
 *           ADC1 shared Core.
 * @param    none
 * @return   none  
 */
void ADC1_SharedCorePowerEnable(void);


/**
 * @ingroup  adcdriver
 * @brief    Sets PWM trigger source for corresponding analog input 
 * @param[in]  channel - Selected channel  
 * @param[in]  pwmInstance - PWM instance for the trigger source
 * @param[in]  triggerNumber - 1, for PWMx Trigger 1
 * @param[in]  triggerNumber - 2, for PWMx Trigger 2
 * @return   none  
 * @note     Configure PWM trigger value using \ref PWM_TriggerACompareValueSet, \ref PWM_TriggerBCompareValueSet
 *           or \ref PWM_TriggerCCompareValueSet before calling this funcion and enable corresponding 
 *           PWM trigger using \ref PWM_Trigger1Enable or \ref PWM_Trigger2Enable post calling it.
 */
void ADC1_PWMTriggerSourceSet(enum ADC_CHANNEL channel, enum ADC_PWM_INSTANCE pwmInstance, enum ADC_PWM_TRIGGERS triggerNumber);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif //_ADC1_H
    
/**
 End of File
*/

