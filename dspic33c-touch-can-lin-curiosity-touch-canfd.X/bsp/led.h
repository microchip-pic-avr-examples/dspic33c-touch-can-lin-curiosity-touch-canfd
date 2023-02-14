/**
 * @file      led.h
 * 
 * @defgroup  touch_demo
 * 
 * @brief     This is the function header file for led functions
*/

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

#ifndef LED_H
#define LED_H

#include "stdint.h"
#include "stdbool.h"

/**
 @ingroup  touch_demo
 @enum     LEDS
 @brief    Defines the on-board LEDs
*/
enum LEDS {
    LED_1,
    LED_2,
    LED_3,
    LED_4,
    LED_5
};

/**
 * @ingroup     touch_demo
 * @brief       This function initializes LED pins
 * @param[in]   none
 * @return      none
 */
void LED_Initialize(void);

/**
 * @ingroup     touch_demo
 * @brief       This function sets the specified LED  
 * @param[in]   \ref LEDS
 * @return      none
 */
void LED_Set(enum LEDS led);

/**
 * @ingroup     touch_demo
 * @brief       This function toggles the specified LED  
 * @param[in]   \ref LEDS
 * @return      none
 */
void LED_Toggle(enum LEDS led);

/**
 * @ingroup     touch_demo
 * @brief       This function turns off the specified LED  
 * @param[in]   \ref LEDS
 * @return      none
 */
void LED_Clear(enum LEDS led);

/**
 * @ingroup     touch_demo
 * @brief       This function sets all the on-board LEDs  
 * @param[in]   none
 * @return      none
 */
void LED_SetAll(void);

/**
 * @ingroup     touch_demo
 * @brief       This function turns off all the on-board LEDs
 * @param[in]   none
 * @return      none
 */
void LED_ClearAll(void);

#endif //LED_H