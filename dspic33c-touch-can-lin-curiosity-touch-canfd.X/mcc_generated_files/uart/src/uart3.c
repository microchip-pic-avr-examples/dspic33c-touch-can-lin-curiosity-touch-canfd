/**
 * UART3 Generated Driver Source File
 * 
 * @file      uart3.c
 *            
 * @ingroup   uartdriver
 *            
 * @brief     This is the generated driver source file for the UART3 driver.
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

// Section: Included Files
#include <stdint.h>
#include <stddef.h>
#include <xc.h>
#include <stddef.h>
#include "../uart3.h"

// Section: Macro Definitions
#define UART3_CLOCK 4000000U
#define UART3_BAUD_TO_BRG_WITH_FRACTIONAL(x) (UART3_CLOCK/(x))
#define UART3_BAUD_TO_BRG_WITH_BRGH_1(x) (UART3_CLOCK/(4U*(x))-1U)
#define UART3_BAUD_TO_BRG_WITH_BRGH_0(x) (UART3_CLOCK/(16U*(x))-1U)
#define UART3_BRG_TO_BAUD_WITH_FRACTIONAL(x) (UART3_CLOCK/(x))
#define UART3_BRG_TO_BAUD_WITH_BRGH_1(x) (UART3_CLOCK/(4U*((x)+1U)))
#define UART3_BRG_TO_BAUD_WITH_BRGH_0(x) (UART3_CLOCK/(16U*((x)+1U)))

#define UART3_MIN_ACHIEVABLE_BAUD_WITH_FRACTIONAL 4U
#define UART3_MIN_ACHIEVABLE_BAUD_WITH_BRGH_1 1U

// Section: Driver Interface

const struct UART_INTERFACE DebugUART = {
    .Initialize = &UART3_Initialize,
    .Deinitialize = &UART3_Deinitialize,
    .Read = &UART3_Read,
    .Write = &UART3_Write,
    .IsRxReady = &UART3_IsRxReady,
    .IsTxReady = &UART3_IsTxReady,
    .IsTxDone = &UART3_IsTxDone,
    .TransmitEnable = &UART3_TransmitEnable,
    .TransmitDisable = &UART3_TransmitDisable,
    .AutoBaudSet = &UART3_AutoBaudSet,
    .AutoBaudQuery = &UART3_AutoBaudQuery,
    .AutoBaudEventEnableGet = &UART3_AutoBaudEventEnableGet,
    .BRGCountSet = &UART3_BRGCountSet,
    .BRGCountGet = &UART3_BRGCountGet,
    .BaudRateSet = &UART3_BaudRateSet,
    .BaudRateGet = &UART3_BaudRateGet,
    .ErrorGet = &UART3_ErrorGet,
    .RxCompleteCallbackRegister = NULL,
    .TxCompleteCallbackRegister = NULL,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = NULL,
    .OverrunErrorCallbackRegister = NULL,
    .ParityErrorCallbackRegister = NULL,
};

// Section: Private Variable Definitions
static union
{
    struct
    {
        uint16_t frammingError :1;
        uint16_t parityError :1;
        uint16_t overrunError :1;
        uint16_t txCollisionError :1;
        uint16_t autoBaudOverflow :1;
        uint16_t reserved :11;
    };
    size_t status;
} uartError;

// Section: UART3 APIs

void UART3_Initialize(void)
{
/*    
     Set the UART3 module to the options selected in the user interface.
     Make sure to set LAT bit corresponding to TxPin as high before UART initialization
*/
    // URXEN ; RXBIMD ; UARTEN disabled; MOD Asynchronous 8-bit UART; UTXBRK ; BRKOVR ; UTXEN ; USIDL ; WAKE ; ABAUD ; BRGH ; 
    U3MODE = 0x0;
    // STSEL 1 Stop bit sent, 1 checked at RX; BCLKMOD enabled; SLPEN ; FLO ; BCLKSEL FOSC/2; C0EN ; RUNOVF ; UTXINV ; URXINV ; HALFDPLX ; 
    U3MODEH = 0x800;
    // OERIE ; RXBKIF ; RXBKIE ; ABDOVF ; OERR ; TXCIE ; TXCIF ; FERIE ; TXMTIE ; ABDOVE ; CERIE ; CERIF ; PERIE ; 
    U3STA = 0x80;
    // URXISEL ; UTXBE ; UTXISEL ; URXBE ; STPMD ; TXWRE ; 
    U3STAH = 0x2E;
    // BaudRate 9592.33; Frequency 4000000 Hz; BRG 417; 
    U3BRG = 0x1A1;
    // BRG 0; 
    U3BRGH = 0x0;
    
    U3MODEbits.UARTEN = 1;   // enabling UART ON bit
    U3MODEbits.UTXEN = 1;
    U3MODEbits.URXEN = 1;
}

void UART3_Deinitialize(void)
{
    U3MODE = 0x0;
    U3MODEH = 0x0;
    U3STA = 0x80;
    U3STAH = 0x2E;
    U3BRG = 0x0;
    U3BRGH = 0x0;
}

uint8_t UART3_Read(void)
{
    while((U3STAHbits.URXBE == 1))
    {
        
    }

    if ((U3STAbits.OERR == 1))
    {
        U3STAbits.OERR = 0;
    }
    
    return U3RXREG;
}

void UART3_Write(uint8_t txData)
{
    while(U3STAHbits.UTXBF == 1)
    {
        
    }

    U3TXREG = txData;    // Write the data byte to the USART.
}

bool UART3_IsRxReady(void)
{
    return (U3STAHbits.URXBE == 0);
}

bool UART3_IsTxReady(void)
{
    return ((!U3STAHbits.UTXBF) && U3MODEbits.UTXEN);
}

bool UART3_IsTxDone(void)
{
    return (bool)(U3STAbits.TRMT && U3STAHbits.UTXBE);
}

void UART3_TransmitEnable(void)
{
    U3MODEbits.UTXEN = 1;
}

void UART3_TransmitDisable(void)
{
    U3MODEbits.UTXEN = 0;
}

void UART3_AutoBaudSet(bool enable)
{
    U3INTbits.ABDIF = 0U;
    U3INTbits.ABDIE = enable;
    U3MODEbits.ABAUD = enable;
}

bool UART3_AutoBaudQuery(void)
{
    return U3MODEbits.ABAUD;
}

bool UART3_AutoBaudEventEnableGet(void)
{ 
    return U3INTbits.ABDIE; 
}

size_t UART3_ErrorGet(void)
{
    uartError.status = 0;
    if(U3STAbits.FERR == 1U)
    {
        uartError.status = uartError.status|UART_ERROR_FRAMING_MASK;
    }
    if(U3STAbits.PERR== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_PARITY_MASK;
    }
    if(U3STAbits.OERR== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_RX_OVERRUN_MASK;
        U3STAbits.OERR = 0;
    }
    if(U3STAbits.TXCIF== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_TX_COLLISION_MASK;
        U3STAbits.TXCIF = 0;
    }
    if(U3STAbits.ABDOVF== 1U)
    {
        uartError.status = uartError.status|UART_ERROR_AUTOBAUD_OVERFLOW_MASK;
        U3STAbits.ABDOVF = 0;
    }
    
    return uartError.status;
}

void UART3_BRGCountSet(uint32_t brgValue)
{
    U3BRG = brgValue & 0xFFFFU;
    U3BRGH = (brgValue >>16U) & 0x000FU;
}

uint32_t UART3_BRGCountGet(void)
{
    uint32_t brgValue;
    
    brgValue = U3BRGH;
    brgValue = (brgValue << 16U) | U3BRG;
    
    return brgValue;
}

void UART3_BaudRateSet(uint32_t baudRate)
{
    uint32_t brgValue;
    
    if((baudRate >= UART3_MIN_ACHIEVABLE_BAUD_WITH_FRACTIONAL) && (baudRate != 0))
    {
        U3MODEHbits.BCLKMOD = 1;
        U3MODEbits.BRGH = 0;
        brgValue = UART3_BAUD_TO_BRG_WITH_FRACTIONAL(baudRate);
    }
    else
    {
        U3MODEHbits.BCLKMOD = 0;
        U3MODEbits.BRGH = 1;
        brgValue = UART3_BAUD_TO_BRG_WITH_BRGH_1(baudRate);
    }
    U3BRG = brgValue & 0xFFFFU;
    U3BRGH = (brgValue >>16U) & 0x000FU;
}

uint32_t UART3_BaudRateGet(void)
{
    uint32_t brgValue;
    uint32_t baudRate;
    
    brgValue = UART3_BRGCountGet();
    if((U3MODEHbits.BCLKMOD == 1) && (brgValue != 0))
    {
        baudRate = UART3_BRG_TO_BAUD_WITH_FRACTIONAL(brgValue);
    }
    else if(U3MODEbits.BRGH == 1)
    {
        baudRate = UART3_BRG_TO_BAUD_WITH_BRGH_1(brgValue);
    }
    else
    {
        baudRate = UART3_BRG_TO_BAUD_WITH_BRGH_0(brgValue);
    }
    return baudRate;
}

int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) {
    unsigned int numBytesWritten = 0 ;
    while(!UART3_IsTxDone());
    while(numBytesWritten<len)
    {
        while(!UART3_IsTxReady());
        UART3_Write(*((uint8_t *)buffer + numBytesWritten++));
    }
    return numBytesWritten;
}
