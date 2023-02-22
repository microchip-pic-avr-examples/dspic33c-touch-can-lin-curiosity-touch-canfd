/*
    MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:

    You may use this software, and any derivatives created by any person or
    entity by or on your behalf, exclusively with Microchip's products.
    Microchip and its subsidiaries ("Microchip"), and its licensors, retain all
    ownership and intellectual property rights in the accompanying software and
    in all derivatives hereto.

    This software and any accompanying information is for suggestion only. It
    does not modify Microchip's standard warranty for its products.  You agree
    that you are solely responsible for testing the software and determining
    its suitability.  Microchip has no obligation to modify, test, certify, or
    support the software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP'S
    PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
    (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
    INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
    ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
    FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL
    LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
    THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR
    THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
    THESE TERMS.
 */

/*----------------------------------------------------------------------------
  include files
----------------------------------------------------------------------------*/
#include "datastreamer.h"
#include "../../system/system.h"
#include "../touch_api.h"
#include "../../uart/uart1.h"
#include "../include/qtm_freq_hop_auto_0x0004_api.h"
#include "../include/qtm_acq_cvd_dspic33ck_api.h"
#include "../include/qtm_scroller_0x000b_api.h"


/*----------------------------------------------------------------------------
  manifest constants
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  global variables
----------------------------------------------------------------------------*/
uint8_t data[] = {
    0x5F, 0xB4, 0x00, 0x86, 0x4A, 0x03, 0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, 0x55, 0x01, 0x6E, 0xA0
};
extern qtm_scroller_control_t qtm_scroller_control1;
/*----------------------------------------------------------------------------
  prototypes
----------------------------------------------------------------------------*/
void datastreamer_transmit(uint8_t data);

void datastreamer_init(void)
{
}

void datastreamer_transmit(uint8_t data)
{ 
    UART1_Write(data);
}

void datastreamer_output(void)
{

	int16_t           i, temp_int_calc;
    static uint8_t sequence = 0u;
    uint16_t u16temp_output;
	uint8_t           u8temp_output, send_header;
	volatile uint16_t count_bytes_out;

	send_header = sequence & (0x0f);
	if (send_header == 0) {
		for (i = 0; i < sizeof(data); i++) {
			datastreamer_transmit(data[i]);
		}
	}

    // Start token
    datastreamer_transmit(0x55);

    datastreamer_transmit(sequence);

    /* table - button specific data */
    for (count_bytes_out = 0u; count_bytes_out < DEF_NUM_SENSORS; count_bytes_out++)
    {
        u16temp_output = get_sensor_node_signal(count_bytes_out);
        //        u16temp_output = get_sensor_node_signalA(count_bytes_out);

        datastreamer_transmit((uint8_t) u16temp_output);
        datastreamer_transmit((uint8_t) (u16temp_output >> 8u));

        u16temp_output = get_sensor_node_reference(count_bytes_out);
        //        u16temp_output = get_sensor_node_signalB(count_bytes_out);
        datastreamer_transmit((uint8_t) u16temp_output);
        datastreamer_transmit((uint8_t) (u16temp_output >> 8u));

        temp_int_calc = get_sensor_node_signal(count_bytes_out);
        temp_int_calc -= get_sensor_node_reference(count_bytes_out);
        u16temp_output = (uint16_t) (temp_int_calc);
        datastreamer_transmit((uint8_t) u16temp_output);
        datastreamer_transmit((uint8_t) (u16temp_output >> 8u));

        u16temp_output = 0u;
        datastreamer_transmit((uint8_t) u16temp_output);
        datastreamer_transmit((uint8_t) (u16temp_output >> 8u));


        u8temp_output = get_sensor_state(count_bytes_out);
     
        		if (0u != (u8temp_output & 0x80)) {
        			datastreamer_transmit(0x01);
        		} else {
        			datastreamer_transmit(0x00);
        		}
        
        u8temp_output = get_sensor_threshold(count_bytes_out);
        datastreamer_transmit(u8temp_output);

    }

	for (count_bytes_out = 0u; count_bytes_out < 1;
	     count_bytes_out++) {

		/* State */
		u8temp_output = get_scroller_state(count_bytes_out);
		if (0u != (u8temp_output & 0x01)) {
			datastreamer_transmit(0x01);
		} else {
			datastreamer_transmit(0x00);
		}

		/* Delta */
		u16temp_output = qtm_scroller_control1.qtm_scroller_data[count_bytes_out].contact_size;
		datastreamer_transmit((uint8_t)u16temp_output);
		datastreamer_transmit((uint8_t)(u16temp_output >> 8u));

		/* Threshold */
		u16temp_output = qtm_scroller_control1.qtm_scroller_config[count_bytes_out].contact_min_threshold;
		datastreamer_transmit((uint8_t)u16temp_output);
		datastreamer_transmit((uint8_t)(u16temp_output >> 8u));

		/* filtered position */
		u16temp_output = get_scroller_position(count_bytes_out);
		datastreamer_transmit((uint8_t)(u16temp_output & 0x00FFu));
		datastreamer_transmit((uint8_t)((u16temp_output & 0xFF00u) >> 8u));
	}




    /* Frequency selection - from acq module */
    datastreamer_transmit(get_current_frequency());

    for (uint8_t count = 0u; count < NUM_FREQ_STEPS; count++)
    {
        /* Frequencies */
        datastreamer_transmit(get_filter_frequency(count));
    }


    datastreamer_transmit(0);
    datastreamer_transmit(sequence++);

    // End token
    datastreamer_transmit(~0x55);
}


