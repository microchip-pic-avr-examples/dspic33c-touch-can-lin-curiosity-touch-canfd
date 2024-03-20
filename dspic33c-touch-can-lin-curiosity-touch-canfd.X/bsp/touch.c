
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
#include "touch.h"

static uint8_t  scroller_status   = 0;
static uint16_t scroller_position = 0;
static enum TOUCH_PADS touchPosition;

void TOUCH_Initialize(void)
{
    touch_init();
}

uint16_t TOUCH_PositionValueGet(void)
{
    return scroller_position;
}

enum TOUCH_PADS TOUCH_PadPositionGet(void)
{
    if (0u != scroller_status) {
        touchPosition = TOUCH_PAD_A;
		if (scroller_position > 50) {
            touchPosition = TOUCH_PAD_A_n_B;
		}
		if (scroller_position > 100) {
			touchPosition = TOUCH_PAD_B;
		}
		if (scroller_position > 155) {
			touchPosition = TOUCH_PAD_B_n_C;
		}
		if (scroller_position > 210) {
			touchPosition = TOUCH_PAD_C;
		}
	}
    else
    {
        touchPosition = TOUCH_NONE;
    }
    return touchPosition;
}

void TOUCH_Tasks(void)
{
    touch_process();
    scroller_status   = get_scroller_state(0);
	scroller_position = get_scroller_position(0);
}