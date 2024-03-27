

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

#include "demo_config.h"
#include "application.h"
#include "../bsp/led.h"
#include "../bsp/touch.h"
#include "../bsp/pot.h"
#include "../bsp/can_app_layer.h"

static void Operation_Touch_Tasks(void);
static void Operation_Potentiometer_Tasks(void);

void Application_Initialze(void)
{
    LED_Initialize();
    POT_Initialize();
    TOUCH_Initialize();
    CAN_Initialize();
}

void Application_Tasks(void)
{
    Operation_Touch_Tasks();
    Operation_Potentiometer_Tasks();
    CAN_Process();
}

static void Operation_Touch_Tasks(void)
{
    TOUCH_Tasks();
    switch(TOUCH_PadPositionGet())
    {
        case TOUCH_PAD_A:
            LED_ClearAll();
            LED_Set(LED_1);
            break;       
        case TOUCH_PAD_A_n_B:
            LED_ClearAll();
            LED_Set(LED_1);
            LED_Set(LED_2);
            break;
        case TOUCH_PAD_B:
            LED_ClearAll();
            LED_Set(LED_1);
            LED_Set(LED_2);
            LED_Set(LED_3);
            break;
        case TOUCH_PAD_B_n_C:
            LED_ClearAll();
            LED_Set(LED_1);
            LED_Set(LED_2);
            LED_Set(LED_3);
            LED_Set(LED_4);
            break;
        case TOUCH_PAD_C:
            LED_ClearAll();
            LED_Set(LED_1);
            LED_Set(LED_2);
            LED_Set(LED_3);
            LED_Set(LED_4);
            LED_Set(LED_5);
            break;
        case TOUCH_NONE:
            LED_ClearAll();
            break;
        default:
            break;
    }
}

static void Operation_Potentiometer_Tasks(void)
{
    float __attribute__((unused)) potVoltage;
    potVoltage = POT_VoltageGet();
}