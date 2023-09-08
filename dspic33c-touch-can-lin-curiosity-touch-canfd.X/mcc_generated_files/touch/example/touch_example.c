 /*******************************************************************************
  Touch Library 3.0.0 Release

  @Company
    Microchip Technology Inc.

  @File Name
    touch_example.c

  @Summary
    QTouch Modular Library

  @Description
    Provides Initialization, Processing and ISR handler of touch library,
    Simple API functions to get/set the key touch parameters from/to the
    touch library data structures
	
*******************************************************************************/
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
/*----------------------------------------------------------------------------
  include files
----------------------------------------------------------------------------*/

/*
Â© [2022] Microchip Technology Inc. and its subsidiaries.

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


#include "touch_example.h"

void touch_mainloop_example(void){
    
    /* call touch process function */
    touch_process();

    if(measurement_done_touch == 1)
    {
        measurement_done_touch = 0;
        // process touch data
        touch_status_display();
    }

}
/*============================================================================
void touch_status_display(void)
------------------------------------------------------------------------------
Purpose: Sample code snippet to demonstrate how to check the status of the 
         sensors
Input  : none
Output : none
Notes  : none
============================================================================*/
void touch_status_display(void)
{
uint8_t key_status = 0u;
uint8_t scroller_status = 0u;
uint16_t scroller_position = 0u;

    key_status = get_sensor_state(0) & KEY_TOUCHED_MASK;
    if (0u != key_status) {
        //Touch detect
    } else {
        //Touch No detect
    }


    scroller_status = get_scroller_state(0);
    scroller_position = get_scroller_position(0);
    //Example: 8 bit scroller resolution. Modify as per requirement.
    scroller_position = scroller_position  >> 5;
    //LED_OFF
     if ( 0u != scroller_status) {
        switch (scroller_position) {
        case 0:
            //LED0_ON
            break;
        case 1:
            //LED1_ON
            break;
        case 2:
            //LED2_ON
            break;
        case 3:
            //LED3_ON
            break;
        case 4:
            //LED4_ON
            break;
        case 5:
            //LED5_ON
            break;
        case 6:
            //LED6_ON
            break;
        case 7:
            //LED7_ON
            break;
        default:
            //LED_OFF
            break;
        }
    }
    scroller_status = get_scroller_state(1);
    scroller_position = get_scroller_position(1);
    //Example: 8 bit scroller resolution. Modify as per requirement.
    scroller_position = scroller_position  >> 5;
    //LED_OFF
     if ( 0u != scroller_status) {
        switch (scroller_position) {
        case 0:
            //LED0_ON
            break;
        case 1:
            //LED1_ON
            break;
        case 2:
            //LED2_ON
            break;
        case 3:
            //LED3_ON
            break;
        case 4:
            //LED4_ON
            break;
        case 5:
            //LED5_ON
            break;
        case 6:
            //LED6_ON
            break;
        case 7:
            //LED7_ON
            break;
        default:
            //LED_OFF
            break;
        }
    }
    scroller_status = get_scroller_state(2);
    scroller_position = get_scroller_position(2);
    //Example: 8 bit scroller resolution. Modify as per requirement.
    scroller_position = scroller_position  >> 5;
    //LED_OFF
     if ( 0u != scroller_status) {
        switch (scroller_position) {
        case 0:
            //LED0_ON
            break;
        case 1:
            //LED1_ON
            break;
        case 2:
            //LED2_ON
            break;
        case 3:
            //LED3_ON
            break;
        case 4:
            //LED4_ON
            break;
        case 5:
            //LED5_ON
            break;
        case 6:
            //LED6_ON
            break;
        case 7:
            //LED7_ON
            break;
        default:
            //LED_OFF
            break;
        }
    }
}
