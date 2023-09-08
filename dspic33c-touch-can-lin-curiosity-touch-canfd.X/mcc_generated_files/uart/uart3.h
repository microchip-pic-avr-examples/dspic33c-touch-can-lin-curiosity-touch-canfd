/**
 * UART3 Generated Driver Header File
 * 
 * @file      uart3.h
 *            
 * @ingroup   uartdriver
 *            
 * @brief     This is the generated driver header file for the UART3 driver
 *            
 * @skipline @version   Firmware Driver Version 1.6.1
 *
 * @skipline @version   PLIB Version 1.4.1
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

#ifndef UART3_H
#define UART3_H

// Section: Included Files

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "uart_interface.h"

// Section: Data Type Definitions

/**
 @ingroup  uartdriver
 @brief    Structure object of type UART_INTERFACE with the 
           custom name given by the user in the Melody Driver User interface. 
           The default name e.g. UART1 can be changed by the 
           user in the UART user interface. 
           This allows defining a structure with application specific name 
           using the 'Custom Name' field. Application specific name allows the 
           API Portability.
*/
extern const struct UART_INTERFACE DebugUART;

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_Initialize API
 */
#define DebugUART_Initialize UART3_Initialize

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_Deinitialize API
 */
#define DebugUART_Deinitialize UART3_Deinitialize

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_Read API
 */
#define DebugUART_Read UART3_Read

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_Write API
 */
#define DebugUART_Write UART3_Write

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_IsRxReady API
 */
#define DebugUART_IsRxReady UART3_IsRxReady

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_IsTxReady API
 */
#define DebugUART_IsTxReady UART3_IsTxReady

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_IsTxDone API
 */
#define DebugUART_IsTxDone UART3_IsTxDone

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_TransmitEnable API
 */
#define DebugUART_TransmitEnable UART3_TransmitEnable

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_TransmitDisable API
 */
#define DebugUART_TransmitDisable UART3_TransmitDisable

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_AutoBaudSet API
 */
#define DebugUART_AutoBaudSet UART3_AutoBaudSet

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_AutoBaudQuery API
 */
#define DebugUART_AutoBaudQuery UART3_AutoBaudQuery

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_AutoBaudEventEnableGet API
 */
#define DebugUART_AutoBaudEventEnableGet UART3_AutoBaudEventEnableGet

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_ErrorGet API
 */
#define DebugUART_ErrorGet UART3_ErrorGet

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_BRGCountSet API
 */
#define DebugUART_BRGCountSet UART3_BRGCountSet

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_BRGCountGet API
 */
#define DebugUART_BRGCountGet UART3_BRGCountGet

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_BaudRateSet API
 */
#define DebugUART_BaudRateSet UART3_BaudRateSet

/**
 * @ingroup  uartdriver
 * @brief    This macro defines the Custom Name for \ref UART3_BaudRateGet API
 */
#define DebugUART_BaudRateGet UART3_BaudRateGet

// Section: UART3 Driver Routines

/**
 * @ingroup  uartdriver
 * @brief    Initializes the UART driver
 * @param    none
 * @return   none
 */
void UART3_Initialize(void);

/**
 * @ingroup  uartdriver
 * @brief    Deinitializes the UART to POR values
 * @param    none
 * @return   none
 */
void UART3_Deinitialize(void);

/**
 * @ingroup  uartdriver
 * @brief    Reads a byte of data from the UART3
 * @pre      UART3_Initialize function should have been called
 *           before calling this function. The transfer status should be checked
 *           to see  if the receiver is not empty before calling this function.
 * @param    none
 * @return   A data byte received by the driver
 */
uint8_t UART3_Read(void);

/**
 * @ingroup    uartdriver
 * @brief      Writes a byte of data to the UART3
 * @pre        UART3_Initialize function should have been called
 *             before calling this function. The transfer status should be checked
 *             to see if transmitter is not full before calling this function.
 * @param[in]  data - Data byte to write to the UART3
 * @return     none
 */
void UART3_Write(uint8_t data);

/**
 * @ingroup  uartdriver
 * @brief    Returns a boolean value if data is available to read
 * @param    none
 * @return   true  - Data available to read
 * @return   false - Data not available to read
 */
bool UART3_IsRxReady(void);

/**
 * @ingroup  uartdriver
 * @brief    Returns a boolean value if data can be written
 * @param    none
 * @return   true    - Data can be written
 * @return   false   - Data can not be written
 */
bool UART3_IsTxReady(void);

/**
 * @ingroup  uartdriver
 * @brief    Indicates if all bytes have been transferred
 * @param    none
 * @return   true    - All bytes transferred
 * @return   false   - Data transfer is pending
 */
bool UART3_IsTxDone(void);

/**
 * @ingroup  uartdriver
 * @brief    Enables UART3 transmit 
 * @param    none
 * @return   none
 */
void UART3_TransmitEnable(void);

/**
 * @ingroup  uartdriver
 * @brief    Disables UART3 transmit 
 * @param    none
 * @return   none
 */
void UART3_TransmitDisable(void);

/**
 * @ingroup  uartdriver
 * @brief    Enables or disables UART3 Auto-Baud detection
 * @param[in]  enable - true, starts the auto-baud detection  
 * @param[in]  enable - false, disables the auto-baud detection  
 * @return   none
 */
void UART3_AutoBaudSet(bool enable);

/**
 * @ingroup  uartdriver
 * @brief    Returns the status of Auto-Baud detection
 * @param    none
 * @return   true    - Auto-Baud detection in progress or counter overflow occurred
 * @return   false   - Auto-Baud detection is complete or disabled
 */
bool UART3_AutoBaudQuery(void);

/**
 * @ingroup  uartdriver
 * @brief    Returns enable state of the Auto-Baud feature
 * @param    none
 * @return   true    - Auto-Baud is enabled
 * @return   false   - Auto-Baud is disabled
 */
bool UART3_AutoBaudEventEnableGet(void);

/**
 * @ingroup  uartdriver
 * @brief    Sets the BRG value of UART3
 * @param[in]   baudRate - BRG value upto 20 bits   
 * @return   none
 * @note    Make sure the is no transmission in progress using \ref UART3_IsTxDone function
 */
void UART3_BRGCountSet(uint32_t brgValue);

/**
 * @ingroup  uartdriver
 * @brief    Gets the BRG value of UART3
 * @param    none
 * @return   Combined BRG value upto 20 bits
 */
uint32_t UART3_BRGCountGet(void);

/**
 * @ingroup  uartdriver
 * @brief    Sets the calculated Baud-Rate of UART3
 * @param[in]   baudRate - Value of Baud-Rate to be set  
 * @return   none
 * @note    Make sure the is no transmission in progress using \ref UART3_IsTxDone function
 */
void UART3_BaudRateSet(uint32_t baudRate);

/**
 * @ingroup  uartdriver
 * @brief    Gets the actual Baud-Rate of UART3
 * @param    none
 * @return   Actual baud-rate of UART3
 */
uint32_t UART3_BaudRateGet(void);

/**
 * @ingroup  uartdriver
 * @brief    Returns the error status of UART3
 * @param    none
 * @return   Errors with masking as per \ref UART3_ERROR_MASKS
 */
size_t UART3_ErrorGet(void);

#endif  // UART3_H

