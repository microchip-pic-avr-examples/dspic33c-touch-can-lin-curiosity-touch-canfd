


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

#include "led.h"
#include "../mcc_generated_files/system/pins.h"

void LED_Initialize(void)
{
    LED_ClearAll();
}

void LED_Set(enum LEDS led)
{
    switch(led)
    {
        case LED_1: led1_SetHigh();
                break;
        case LED_2: led2_SetHigh();
                break;
        case LED_3: led3_SetHigh();
                break;
        case LED_4: led4_SetHigh();
                break;
        case LED_5: led5_SetHigh();
                break;
        default:
                break;
    }
}

void LED_Toggle(enum LEDS led)
{
    switch(led)
    {
        case LED_1: led1_Toggle();
                break;
        case LED_2: led2_Toggle();
                break;
        case LED_3: led3_Toggle();
                break;
        case LED_4: led4_Toggle();
                break;
        case LED_5: led5_Toggle();
                break;
        default:
                break;
    }
}

void LED_Clear(enum LEDS led)
{
    switch(led)
    {
        case LED_1: led1_SetLow();
                break;
        case LED_2: led2_SetLow();
                break;
        case LED_3: led3_SetLow();
                break;
        case LED_4: led4_SetLow();
                break;
        case LED_5: led5_SetLow();
                break;
        default:
                break;
    }
}

void LED_SetAll(void)
{
    led1_SetHigh();
    led2_SetHigh();
    led3_SetHigh();
    led4_SetHigh();
    led5_SetHigh();
}

void LED_ClearAll(void)
{
    led1_SetLow();
    led2_SetLow();
    led3_SetLow();
    led4_SetLow();
    led5_SetLow();
}