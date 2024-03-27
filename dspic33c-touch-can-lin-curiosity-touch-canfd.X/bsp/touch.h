/**
 * @file      touch.h
 * 
 * @defgroup  touch_demo
 * 
 * @brief     This is the function header file for touch functions
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

#ifndef TOUCH_H
#define TOUCH_H

#include <stdint.h>
#include "../mcc_generated_files/touch/touch.h"
#include "../mcc_generated_files/touch/include/touch_api.h"

/**
 @ingroup  touch_demo
 @enum     TOUCH_PADS
 @brief    Defines the on-board touch pads 
*/
enum TOUCH_PADS {
    TOUCH_NONE,
    TOUCH_PAD_A,
    TOUCH_PAD_A_n_B,
    TOUCH_PAD_B,
    TOUCH_PAD_B_n_C,
    TOUCH_PAD_C
};

/**
 * @ingroup     touch_demo
 * @brief       This function initializes touch application
 * @param[in]   none
 * @return      none
 */
void TOUCH_Initialize(void);

/**
 * @ingroup     touch_demo
 * @brief       This function gets the touch position value of the slider
 * @param[in]   none
 * @return      touch value from 0 to 256
 */
uint16_t TOUCH_PositionValueGet(void);

/**
 * @ingroup     touch_demo
 * @brief       This function gets the touch position on the pads
 * @param[in]   none
 * @return      none
 */
enum TOUCH_PADS TOUCH_PadPositionGet(void);

/**
 * @ingroup     touch_demo
 * @brief       This function performs process tasks of touch application
 * @param[in]   none
 * @return      none
 */
void TOUCH_Tasks(void);

#endif  // TOUCH_H