
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

#include <stdbool.h>
#include "../mcc_generated_files/adc/adc1.h"
#include "../mcc_generated_files/touch/include/touch_api.h"
#include "pot.h"

void POT_Initialize(void)
{
    ADC.Initialize();
}

uint16_t POT_PositionGet(void)
{
    uint16_t position;
    ADC.InterruptDisable();
    ADC.SoftwareTriggerEnable();
    while(!ADC.IsConversionComplete(potentiometer));
    position = ADC.ConversionResultGet(potentiometer);
    ADC.InterruptEnable();
    return position;
}

float POT_VoltageGet(void)
{
    float potVoltage;
    uint16_t position;
    ADC.InterruptDisable();
    ADC.SoftwareTriggerEnable();
    while(!ADC.IsConversionComplete(potentiometer));
    position = ADC.ConversionResultGet(potentiometer);
    potVoltage = ((float)position*ADC_REFERENCE)/ADC_MAX_COUNT;
    ADC.InterruptEnable();
    return potVoltage;
}

void ADC_CommonCallback(void)
{
    qtm_dspic33c_touch_handler_eoc();
}

void __attribute__ ( ( __interrupt__ , auto_psv ) ) _ADCInterrupt ( void )
{
    ADC_CommonCallback();
        
    // clear the ADC1 interrupt flag
    ADC.InterruptFlagClear();
}