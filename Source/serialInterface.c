#include "hal_uart.h"
#include "serialInterface.h"
#include "Ketamine.h"
#include "bcomdef.h"
#include "hal_led.h"
#include "hal_i2c.h"

#include "string.h"

#include "gatt.h"

//local function
static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );

uint8 serialInterface_TaskID;   // Task ID for internal task/event processing

#define SERIAL_MSG_START_ID        0xAB //indicate the start of serial message
#define SERIAL_ACK                 0xA5 //inidicate an ACK
#define SERIAL_DATA                0xAD //indiciate data from central device

#define RX_BUFF_BIO_SIZE    50
#define RX_BUFF_ECG_SIZE    10


uint8 pktBuf[RX_BUFF_BIO_SIZE];
uint8 preamble = FALSE; // preamble includes FF, 7F, Type
uint8 pktLength = 0; // target packet length
uint8 pktRxByteOffset = 0; // current received bytes offset
uint8 numBytes;
uint8 RxByte;
uint8 pktSeq = 0;
uint8 ecgPktBuf[RX_BUFF_BIO_SIZE];
uint8 accBuf[1];
uint8 tempBuf[2];
uint8 ecgBufOffset = 1;
uint8 accReady = FALSE;
uint8 tempReady = FALSE;

//static uint8 temp_buf[48];

void SerialInterface_Init( uint8 task_id )
{
  serialInterface_TaskID = task_id;
    
  //NPI_InitTransport(cSerialPacketParser);
  
  // Set regulator to low to save power; Turn off pic32 regulator and debug LED on P0_3
  // P0SEL &= ~0x48;
  // P0DIR |= 0x48;
  // P0_6 = 0;
  // P0_3 = 0;
  P0SEL &= ~0x38;
  P0DIR |= 0x38;
  P0_5 = 0;
  P0_3 = 0;
  P0_4 = 0;
}

uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( serialInterface_TaskID )) != NULL )
    {
      SerialInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // Discard unknown events
  return 0;
}

static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}


// void serialPacketHandler() {
// //  sendAckMessage(pktBuf[1]);
//   ecgPktBuf[ecgBufOffset] = pktBuf[2];
//   ecgPktBuf[++ecgBufOffset] = pktBuf[3];
//   ecgBufOffset++;
//   switch(pktBuf[0]) {
//   case 1: //ECG beat
//     if(ecgBufOffset >= RX_BUFF_ECG_SIZE) {
//       
//       if(accReady) {
//         accReady = FALSE;
//         ecgPktBuf[0] = 2;
//         sendNotification((uint8 *)&ecgPktBuf, 17);
//         sendAckMessage(sendNotification((uint8 *)&ecgPktBuf, 17));
// //        sendAckMessage(1);
//       }
//       else if(tempReady) {
//         tempReady = FALSE;
//         ecgPktBuf[0] = 3;
//         ecgPktBuf[11] = tempBuf[0];
//         ecgPktBuf[12] = tempBuf[1];
// //        txPktBuf
//         sendNotification((uint8 *)&ecgPktBuf, 13);
//         sendAckMessage(sendNotification((uint8 *)&ecgPktBuf, 13));
// //        sendAckMessage(2);
//       }
//       else { // only ECG
//         ecgPktBuf[0] = 1;
//         sendNotification((uint8 *)&ecgPktBuf, 11);
//         sendAckMessage(sendNotification((uint8 *)&ecgPktBuf, 11));
//         ecgBufOffset = 1;
// //        sendAckMessage(3);
//       }
//     }
// //    else
// //      sendAckMessage(0);
// 
//     break;
//   case 2: //ACC
//     accBuf[0] = pktBuf[4];
//     accReady = TRUE;
//     break;
//   case 3: //Temp
//     tempBuf[0] = pktBuf[4];
//     tempBuf[1] = pktBuf[5];
//     tempReady = TRUE;
//     break;
//   }
// }


void cSerialPacketParser( uint8 port, uint8 events )
{  
  if((preamble) ) {
    if(pktRxByteOffset != 0) {
      numBytes = NPI_RxBufLen();
      if(numBytes < pktRxByteOffset) {
        (void)NPI_ReadTransport(pktBuf+(pktLength-pktRxByteOffset+1), numBytes);
        pktRxByteOffset -= numBytes;
        return;
      }
      else {
        (void)NPI_ReadTransport(pktBuf+(pktLength-pktRxByteOffset+1), pktRxByteOffset);
        pktRxByteOffset = 0;
      }
    }
    if(pktRxByteOffset == 0) {
      //HalLedSet( HAL_LED_1, HAL_LED_MODE_TOGGLE );
      // got sufficient bytes in a packet
//      serialPacketHandler();
      (void)sendNotification((uint8 *)&pktBuf, pktLength+1);
      preamble = FALSE;
      return;
    }
  }
  
  numBytes = NPI_RxBufLen();
   
  if(numBytes < 3)
    return;
   
//   HalLedSet( HAL_LED_1, HAL_LED_MODE_TOGGLE );
   (void)NPI_ReadTransport((uint8 *)&RxByte, 1);
   if(RxByte != 0xFF)
     return;
   else{
     (void)NPI_ReadTransport((uint8 *)&RxByte, 1);
     if(RxByte != 0x7F)
       return;
     else { // Then handle the packet depending on type
              
       (void)NPI_ReadTransport((uint8 *)&pktBuf, 1);
       preamble = TRUE;
       switch(pktBuf[0]) {
        case 1: //ECG beat
          pktRxByteOffset = 1;
          break;
        case 2: //ACC
          pktRxByteOffset = 1;
          break;
        case 3: //Temp
          pktRxByteOffset = 2;
          break;
        default:
          preamble = FALSE;
       }
       pktLength = pktRxByteOffset;
     }
   }
}

uint8 sendNotification(uint8* bytes_sent, uint8 len)
{
  attHandleValueNoti_t noti;
  //dummy handle
  noti.handle = 0x2E;
  noti.len = len;
  uint8 i;
  
  for (i= 0; i < len; i++)
  {
    noti.value[i] = bytes_sent[i];
  }
  if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
  {
    return 0xFF;
  } 
  return 0xAA;
}

uint8 sendAckMessage(uint8 bytes_sent)
{
  uint8 data[1] = {0};
  
  //data[0]= SERIAL_MSG_START_ID;
  //data[1]= SERIAL_ACK;
  data[0]= bytes_sent;
  uint8 success_len = HalUARTWrite(NPI_UART_PORT, (uint8*)data, 1);
  if (success_len == 3)
  {
    return SUCCESS;
  }
  else
  {
    return 1;   //ack wasn't sent over UAR
  }
}

uint16 circular_diff(uint16 offset, uint16 tail)
{
  if (offset > tail)
  {
    return (offset - tail);
  }
  else
  {
    return (RX_BUFF_SIZE - tail) + offset;
  }    
}

uint16 circular_add(uint16 x, uint16 y)
{
  uint16 sum = x + y;
  if (sum != RX_BUFF_SIZE)
  {
    sum = sum % RX_BUFF_SIZE;
  }
  else
  {
    sum = 0;
  }
  return sum;
}

void HalTmpSelect(void)
{
  //Set up I2C that is used to communicate with TMP102
  HalI2CInit(TMP102_ADDRESS, i2cClock_267KHZ);
}