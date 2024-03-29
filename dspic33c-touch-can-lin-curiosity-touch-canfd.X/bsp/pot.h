/**
 * @file      pot.h
 * 
 * @defgroup  touch_demo
 * 
 * @brief     This is the function header file for potentiometer functions
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

#ifndef POT_H
#define POT_H


#include "stdint.h"

/**
 * @ingroup  touch_demo
 * @brief    This macro defines the ADC max count
 */
#define ADC_MAX_COUNT 4096U

/**
 * @ingroup  touch_demo
 * @brief    This macro defines the reference voltage of ADC
 */
#define ADC_REFERENCE 3.3

/**
 * @ingroup     touch_demo
 * @brief       This function initializes potentiometer channel of ADC
 * @param[in]   none
 * @return      none
 */
void POT_Initialize(void);

/**
 * @ingroup     touch_demo
 * @brief       This function gets the position count of potentiometer
 * @param[in]   none
 * @return      count from 0 to 4096
 */
uint16_t POT_PositionGet(void);

/**
 * @ingroup     touch_demo
 * @brief       This function gets the voltage level of potentiometer
 * @param[in]   none
 * @return      voltage value from 0 to 3.3
 */
float POT_VoltageGet(void);

#endif  //POT_H