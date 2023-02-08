/**
 * PINS Generated Driver Header File 
 * 
 * @file      pins.h
 *            
 * @defgroup  pinsdriver Pins Driver
 *            
 * @brief     The Pin Driver directs the operation and function of 
 *            the selected device pins using dsPIC MCUs.
 *
 * @version   Firmware Driver Version 1.0.1
 *
 * @version   PLIB Version 1.1.0
 *
 * @skipline  Device : dsPIC33CK1024MP710
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

#ifndef PINS_H
#define PINS_H
// Section: Includes
#include <xc.h>

// Section: Device Pin Macros

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RB10 GPIO Pin which has a custom name of led_green to High
 * @pre      The RB10 must be set as Output Pin             
 * @return   none  
 */
#define led_green_SetHigh()          (_LATB10 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RB10 GPIO Pin which has a custom name of led_green to Low
 * @pre      The RB10 must be set as Output Pin
 * @return   none  
 */
#define led_green_SetLow()           (_LATB10 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RB10 GPIO Pin which has a custom name of led_green
 * @pre      The RB10 must be set as Output Pin
 * @return   none  
 */
#define led_green_Toggle()           (_LATB10 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RB10 GPIO Pin which has a custom name of led_green
 * @return   none  
 */
#define led_green_GetValue()         _RB10

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RB10 GPIO Pin which has a custom name of led_green as Input
 * @return   none  
 */
#define led_green_SetDigitalInput()  (_TRISB10 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RB10 GPIO Pin which has a custom name of led_green as Output
 * @return   none  
 */
#define led_green_SetDigitalOutput() (_TRISB10 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RB12 GPIO Pin which has a custom name of led_red to High
 * @pre      The RB12 must be set as Output Pin             
 * @return   none  
 */
#define led_red_SetHigh()          (_LATB12 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RB12 GPIO Pin which has a custom name of led_red to Low
 * @pre      The RB12 must be set as Output Pin
 * @return   none  
 */
#define led_red_SetLow()           (_LATB12 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RB12 GPIO Pin which has a custom name of led_red
 * @pre      The RB12 must be set as Output Pin
 * @return   none  
 */
#define led_red_Toggle()           (_LATB12 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RB12 GPIO Pin which has a custom name of led_red
 * @return   none  
 */
#define led_red_GetValue()         _RB12

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RB12 GPIO Pin which has a custom name of led_red as Input
 * @return   none  
 */
#define led_red_SetDigitalInput()  (_TRISB12 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RB12 GPIO Pin which has a custom name of led_red as Output
 * @return   none  
 */
#define led_red_SetDigitalOutput() (_TRISB12 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RD15 GPIO Pin which has a custom name of can_standby to High
 * @pre      The RD15 must be set as Output Pin             
 * @return   none  
 */
#define can_standby_SetHigh()          (_LATD15 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RD15 GPIO Pin which has a custom name of can_standby to Low
 * @pre      The RD15 must be set as Output Pin
 * @return   none  
 */
#define can_standby_SetLow()           (_LATD15 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RD15 GPIO Pin which has a custom name of can_standby
 * @pre      The RD15 must be set as Output Pin
 * @return   none  
 */
#define can_standby_Toggle()           (_LATD15 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RD15 GPIO Pin which has a custom name of can_standby
 * @return   none  
 */
#define can_standby_GetValue()         _RD15

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RD15 GPIO Pin which has a custom name of can_standby as Input
 * @return   none  
 */
#define can_standby_SetDigitalInput()  (_TRISD15 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RD15 GPIO Pin which has a custom name of can_standby as Output
 * @return   none  
 */
#define can_standby_SetDigitalOutput() (_TRISD15 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE6 GPIO Pin which has a custom name of led1 to High
 * @pre      The RE6 must be set as Output Pin             
 * @return   none  
 */
#define led1_SetHigh()          (_LATE6 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE6 GPIO Pin which has a custom name of led1 to Low
 * @pre      The RE6 must be set as Output Pin
 * @return   none  
 */
#define led1_SetLow()           (_LATE6 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE6 GPIO Pin which has a custom name of led1
 * @pre      The RE6 must be set as Output Pin
 * @return   none  
 */
#define led1_Toggle()           (_LATE6 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE6 GPIO Pin which has a custom name of led1
 * @return   none  
 */
#define led1_GetValue()         _RE6

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE6 GPIO Pin which has a custom name of led1 as Input
 * @return   none  
 */
#define led1_SetDigitalInput()  (_TRISE6 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE6 GPIO Pin which has a custom name of led1 as Output
 * @return   none  
 */
#define led1_SetDigitalOutput() (_TRISE6 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE7 GPIO Pin which has a custom name of led2 to High
 * @pre      The RE7 must be set as Output Pin             
 * @return   none  
 */
#define led2_SetHigh()          (_LATE7 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE7 GPIO Pin which has a custom name of led2 to Low
 * @pre      The RE7 must be set as Output Pin
 * @return   none  
 */
#define led2_SetLow()           (_LATE7 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE7 GPIO Pin which has a custom name of led2
 * @pre      The RE7 must be set as Output Pin
 * @return   none  
 */
#define led2_Toggle()           (_LATE7 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE7 GPIO Pin which has a custom name of led2
 * @return   none  
 */
#define led2_GetValue()         _RE7

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE7 GPIO Pin which has a custom name of led2 as Input
 * @return   none  
 */
#define led2_SetDigitalInput()  (_TRISE7 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE7 GPIO Pin which has a custom name of led2 as Output
 * @return   none  
 */
#define led2_SetDigitalOutput() (_TRISE7 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE8 GPIO Pin which has a custom name of led3 to High
 * @pre      The RE8 must be set as Output Pin             
 * @return   none  
 */
#define led3_SetHigh()          (_LATE8 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE8 GPIO Pin which has a custom name of led3 to Low
 * @pre      The RE8 must be set as Output Pin
 * @return   none  
 */
#define led3_SetLow()           (_LATE8 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE8 GPIO Pin which has a custom name of led3
 * @pre      The RE8 must be set as Output Pin
 * @return   none  
 */
#define led3_Toggle()           (_LATE8 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE8 GPIO Pin which has a custom name of led3
 * @return   none  
 */
#define led3_GetValue()         _RE8

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE8 GPIO Pin which has a custom name of led3 as Input
 * @return   none  
 */
#define led3_SetDigitalInput()  (_TRISE8 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE8 GPIO Pin which has a custom name of led3 as Output
 * @return   none  
 */
#define led3_SetDigitalOutput() (_TRISE8 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE9 GPIO Pin which has a custom name of led4 to High
 * @pre      The RE9 must be set as Output Pin             
 * @return   none  
 */
#define led4_SetHigh()          (_LATE9 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE9 GPIO Pin which has a custom name of led4 to Low
 * @pre      The RE9 must be set as Output Pin
 * @return   none  
 */
#define led4_SetLow()           (_LATE9 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE9 GPIO Pin which has a custom name of led4
 * @pre      The RE9 must be set as Output Pin
 * @return   none  
 */
#define led4_Toggle()           (_LATE9 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE9 GPIO Pin which has a custom name of led4
 * @return   none  
 */
#define led4_GetValue()         _RE9

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE9 GPIO Pin which has a custom name of led4 as Input
 * @return   none  
 */
#define led4_SetDigitalInput()  (_TRISE9 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE9 GPIO Pin which has a custom name of led4 as Output
 * @return   none  
 */
#define led4_SetDigitalOutput() (_TRISE9 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RF11 GPIO Pin which has a custom name of led5 to High
 * @pre      The RF11 must be set as Output Pin             
 * @return   none  
 */
#define led5_SetHigh()          (_LATF11 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RF11 GPIO Pin which has a custom name of led5 to Low
 * @pre      The RF11 must be set as Output Pin
 * @return   none  
 */
#define led5_SetLow()           (_LATF11 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RF11 GPIO Pin which has a custom name of led5
 * @pre      The RF11 must be set as Output Pin
 * @return   none  
 */
#define led5_Toggle()           (_LATF11 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RF11 GPIO Pin which has a custom name of led5
 * @return   none  
 */
#define led5_GetValue()         _RF11

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RF11 GPIO Pin which has a custom name of led5 as Input
 * @return   none  
 */
#define led5_SetDigitalInput()  (_TRISF11 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RF11 GPIO Pin which has a custom name of led5 as Output
 * @return   none  
 */
#define led5_SetDigitalOutput() (_TRISF11 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Initializes the PINS module
 * @return   none  
 */
void PINS_Initialize(void);



#endif
