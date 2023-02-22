#include "../mcc_generated_files/can/can1.h"
#include "../mcc_generated_files/system/pins.h"
#include "../mcc_generated_files/timer/sccp1.h"
#include "touch.h"
#include "../application/demo_config.h"
#include "pot.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

struct CAN_MSG_OBJ canRxMsg;
struct CAN_MSG_OBJ canTxTouchMsg;
struct CAN_MSG_OBJ canTxPotMsg;

static enum CAN_BUS_ERRORS{
    CAN_ERROR_NONE,
    CAN_ERROR_ACTIVE,
    CAN_ERROR_WARNING,
    CAN_ERROR_PASSIVE
}canPreviousTxState, canPreviousRxState; 

static bool rxOverflowStatus;
static bool msgStatus;
static bool txWriteFail;
static bool sendTouchStatus;
static uint16_t canTouchData;
static uint16_t canPotData;

static void CAN_TouchDataUpdate(void);
static void CAN_PotDataUpdate(void);
static void CAN_CheckRxErrors(void);
static void CAN_CheckTxErrors(void);

static void CAN_RxBufferOverFlowCallback(void);
static void CAN_BusWakeUpActivityCallback(void);


#if defined UARTDEBUG_ON
static uint8_t CAN_DlcToDataBytesGet(const enum CAN_DLC dlc);
static void CAN_PrintMsgObjStruct(struct CAN_MSG_OBJ *rxCanMsg);
static void printCanDatainHex(struct CAN_MSG_OBJ *rxCanMsg);
#endif

void CAN_TouchTransmitCallback (void)
{ 
    sendTouchStatus = true;
} 

void CAN_Initialize(void)
{
    CAN_TransmitTimer.TimeoutCallbackRegister(CAN_TouchTransmitCallback);
    CAN_FD_Driver.RxBufferOverFlowCallbackRegister(&CAN_RxBufferOverFlowCallback);
    CAN_FD_Driver.BusWakeUpActivityCallbackRegister(&CAN_BusWakeUpActivityCallback);
    canPreviousRxState = CAN_ERROR_NONE;
    canPreviousTxState = CAN_ERROR_NONE;
    can_standby_SetLow();
    CAN_TransmitTimer.Start();
}

void CAN_Process(void)
{
    msgStatus = false;
    /*See if there is any data in RX FIFO*/
    if(CAN_FD_Driver.ReceivedMessageCountGet() > 0) 
    {
        CAN_FD_Driver.Receive(&canRxMsg);
        rxOverflowStatus = false;
        led_green_SetHigh();
#if defined UARTDEBUG_ON
        printf("\r\n[*] Received Message Frame:\r\n---------\r\n");
        CAN_PrintMsgObjStruct(&canRxMsg);
#endif
        msgStatus = true;
    }
    if(sendTouchStatus)
    {
        CAN_TouchDataUpdate();
        CAN_PotDataUpdate();
        
//Transmitting Touch Data 
#if defined UARTDEBUG_ON
        printf("\r\n[*] Transmitting Message Frame:\r\n---------\r\n");
        CAN_PrintMsgObjStruct(&canTxTouchMsg);
#endif
        msgStatus = CAN_FD_Driver.Transmit(CAN1_TX_TXQ, &canTxTouchMsg);
        if(msgStatus == CAN_TX_MSG_REQUEST_SUCCESS)
        {
            txWriteFail = false;
            led_green_SetHigh();
#if defined UARTDEBUG_ON
            printf("Touch data fed to transmit FIFO successfully\r\n");
#endif
        }
        else
        {
            txWriteFail = true;
#if defined UARTDEBUG_ON
            printf("Touch data write to FIFO failure\r\n");
#endif
        }
        
//Transmitting Potentiometer Data       
#if defined UARTDEBUG_ON
        printf("\r\n[*] Transmitting Message Frame:\r\n---------\r\n");
        CAN_PrintMsgObjStruct(&canTxPotMsg);
#endif
        msgStatus = CAN_FD_Driver.Transmit(CAN1_TX_TXQ, &canTxPotMsg);
        if(msgStatus == CAN_TX_MSG_REQUEST_SUCCESS)
        {
            txWriteFail = false;
            led_green_SetHigh();
#if defined UARTDEBUG_ON
            printf("Potentiometer data fed to transmit FIFO successfully\r\n");
#endif
        }
        else
        {
            txWriteFail = true;
#if defined UARTDEBUG_ON
            printf("Potentiometer data write to FIFO failure\r\n");
#endif
        }
        sendTouchStatus = false;
    }
    /*Check if any errors while receiving*/
    CAN_CheckRxErrors();

    /*Check if any errors while transmitting*/
    CAN_CheckTxErrors();

    /*Check if write or read FIFO errors*/
    if(rxOverflowStatus || txWriteFail)
    {
        led_red_SetHigh();
    }
    led_green_SetLow();
    
}

static void CAN_TouchDataUpdate(void)
{
    canTouchData = TOUCH_PositionValueGet();
    canTxTouchMsg.data = (uint8_t*)&canTouchData;
    canTxTouchMsg.msgId = CAN_TOUCH_MSG_ID;
    canTxTouchMsg.field.dlc = DLC_2;
    canTxTouchMsg.field.brs = CAN_BRS_MODE;
    canTxTouchMsg.field.formatType = CAN_FD_FORMAT;
    canTxTouchMsg.field.frameType = CAN_FRAME_DATA;
    canTxTouchMsg.field.idType = CAN_FRAME_STD;
}
static void CAN_PotDataUpdate(void)
{
    canPotData = POT_PositionGet();
    canTxPotMsg.data = (uint8_t*)&canPotData;
    canTxPotMsg.msgId = CAN_POT_MSG_ID;
    canTxPotMsg.field.dlc = DLC_2;
    canTxPotMsg.field.brs = CAN_BRS_MODE;
    canTxPotMsg.field.formatType = CAN_FD_FORMAT;
    canTxPotMsg.field.frameType = CAN_FRAME_DATA;
    canTxPotMsg.field.idType = CAN_FRAME_STD;
}

static void CAN_BusWakeUpActivityCallback(void)
{
#if defined UARTDEBUG_ON
    printf("\r\nBus Wake-Up Callback can be used \r\nto handle wake-up activities\r\n");
#endif
}

static void CAN_RxBufferOverFlowCallback(void)
{
    led_red_SetHigh();
#if defined UARTDEBUG_ON
    printf("\r\n\r\nCAN Receive Buffer Overflow Occurred\r\n");
    printf("CAN Receive buffer overflow occurs if receive buffer \r\nis filled faster than it is being read\r\n");   
#endif
}

void CAN_CheckTxErrors(void)
{
    
    /**TX Errors**/
    
    /*If node reached TX Passive Error state*/
    if(CAN_FD_Driver.IsTxErrorActive())
    {
        if(canPreviousTxState != CAN_ERROR_PASSIVE)
        {
#if defined UARTDEBUG_ON
            printf("CAN node is in TX Error Passive state, 127 < Error Count < 256 \r\n");
#endif
            canPreviousTxState = CAN_ERROR_PASSIVE;
        }
    }
    
    /*If node reached TX Warning state*/
    else if(CAN_FD_Driver.IsTxErrorWarning())
    {
        if(canPreviousTxState != CAN_ERROR_WARNING)
        {
#if defined UARTDEBUG_ON
            printf("CAN node is in TX Error Warning state, 94 < Error Count < 128  \r\n");
#endif
            canPreviousTxState = CAN_ERROR_WARNING;
        }
    }
    
    /*If node reached TX Active Error state*/
    else if(CAN_FD_Driver.IsTxErrorActive())
    {
        if(canPreviousTxState != CAN_ERROR_ACTIVE)
        {
#if defined UARTDEBUG_ON
            printf("CAN node is in TX Error Active state, 0 < Error Count < 95  \r\n");
#endif
            canPreviousTxState = CAN_ERROR_ACTIVE;
        }
    }
    
    /*Reset status if no errors*/
    else
    {
        canPreviousTxState = CAN_ERROR_NONE;
        led_red_SetLow();
    }
    
    /*Set status to RED if any errors*/ 
    if(canPreviousTxState != CAN_ERROR_NONE)
    {
        led_red_SetHigh();
    }
    
    /*CAN node reaches Bus-Off state if error count is grater than 255*/
}

static void CAN_CheckRxErrors(void)
{
    
    /**RX Errors */
    
    /*If node reached RX Passive Error state*/
    if(CAN_FD_Driver.IsRxErrorPassive())
    {
        if(canPreviousRxState != CAN_ERROR_PASSIVE)
        {
#if defined UARTDEBUG_ON
            printf("CAN node is in RX Error Passive state, 127 < Error Count < 256\r\n");
#endif
            canPreviousRxState = CAN_ERROR_PASSIVE;
        }
    }
    
    /*If node reached RX Warning state*/
    else if(CAN_FD_Driver.IsRxErrorWarning())
    {
        if(canPreviousRxState != CAN_ERROR_WARNING)
        {
#if defined UARTDEBUG_ON
            printf("CAN node is in RX Error Warning state, 94 < Error Count < 128 \r\n");
#endif
            canPreviousRxState = CAN_ERROR_WARNING;
        }
    }
    
    /*If node reached RX Active Error state*/
    else if(CAN_FD_Driver.IsRxErrorActive())
    {
        if(canPreviousRxState != CAN_ERROR_ACTIVE)
        {
#if defined UARTDEBUG_ON
            printf("CAN node is in RX Error Active state, 0 < Error Count < 95 \r\n");
#endif
            canPreviousRxState = CAN_ERROR_ACTIVE;
        }
    }
    
    /*Reset status if no errors*/
    else
    {
        led_red_SetLow();
        canPreviousRxState = CAN_ERROR_NONE;
    }
    
    /*Set status to RED if any errors*/ 
    if(canPreviousRxState != CAN_ERROR_NONE)
    {
        led_red_SetHigh();
    }
}

#if defined UARTDEBUG_ON
static uint8_t CAN_DlcToDataBytesGet(const enum CAN_DLC dlc)
{
    static const uint8_t dlcByteSize[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};
    return dlcByteSize[dlc];
}


static void printCanDatainHex(struct CAN_MSG_OBJ *rxCanMsg)
{
    uint8_t i=0;
    for(i=0;i<CAN_DlcToDataBytesGet(rxCanMsg->field.dlc);i++)
    {
        printf("0x%X ",rxCanMsg->data[i]);
    }
    printf("\r\n");
}

static void CAN_PrintMsgObjStruct(struct CAN_MSG_OBJ *rxCanMsg)
{
    printf("------------------------------------------------------------------\r\n");
    printf("[*] Msg ID: 0x%lX\r\n", rxCanMsg->msgId);
    printf("[*] Msg Data:");
    printCanDatainHex(rxCanMsg);
    printf("[*] DLC: 0x%X\r\n", CAN_DlcToDataBytesGet(rxCanMsg->field.dlc));
    printf("[*] IdType: %s\r\n", rxCanMsg->field.idType == CAN_FRAME_STD ? "CAN_FRAME_STD" : "CAN_FRAME_EXT");
    printf("[*] FrameType: %s\r\n", rxCanMsg->field.frameType == CAN_FRAME_DATA ? "CAN_FRAME_DATA" : "CAN_FRAME_RTR");
    printf("[*] BRS: %s\r\n", rxCanMsg->field.brs == CAN_NON_BRS_MODE ? "CAN_NON_BRS_MODE" : "CAN_BRS_MODE");
    printf("[*] FormateType: %s\r\n", rxCanMsg->field.formatType == CAN_2_0_FORMAT ? "CAN_2_0_FORMAT" : "CAN_FD_FORMAT");
    printf("------------------------------------------------------------------\r\n");
}
#endif