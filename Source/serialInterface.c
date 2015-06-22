#include "hal_uart.h"
#include "serialInterface.h"
#include "Ketamine.h"
#include "bcomdef.h"
#include "hal_led.h"
#include "hal_sensor.h"
#include "hal_i2c.h"

#include "string.h"
#include "gatt.h"
#include "simpleGATTprofile.h"

//local function
static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );

uint8 serialInterface_TaskID;   // Task ID for internal task/event processing

#define SERIAL_MSG_START_ID        0xAB //indicate the start of serial message
#define SERIAL_ACK                 0xA5 //inidicate an ACK
#define SERIAL_DATA                0xAD //indiciate data from central device

uint8 pktBuf[RX_BUFF_SIZE];
uint8 debug[5];
uint8 preamble = FALSE; // preamble includes FF, 7F, Type
uint8 pktLength = 0; // target packet length
uint8 pktRxByteOffset = 0; // current received bytes offset
uint8 numBytes;
uint8 pktSeq = 0;

uint8 cameraAddr = (CAM_ADDR << 5);  	// addr
uint16 picTotalLen = 0;            	// picture length
uint16 pktCnt = 0;
uint16 tmpPktIdx = 0;
uint16 lastPktLen = 0;
uint8 isLastPkt = 0;
uint16 seqNum = 0;
uint8 waitBLEAck = 0;

void SerialInterface_Init( uint8 task_id )
{
  serialInterface_TaskID = task_id;
  //NPI_InitTransport(cSerialPacketParser);
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

void cSerialPacketParser( uint8 port, uint8 events )
{
  numBytes = NPI_RxBufLen();
  if(serialCameraState == 0x30){
    uint8 pktRemain ;
    if(isLastPkt == 1){
      pktRemain = lastPktLen-pktRxByteOffset;
    }
    else{
      pktRemain = PIC_PKT_LEN-pktRxByteOffset;
    }
    uint8 cnt = 0;
    cnt = NPI_ReadTransport(pktBuf + pktRxByteOffset, pktRemain);
    pktRxByteOffset += cnt;
    if(pktRxByteOffset < PIC_PKT_LEN && isLastPkt == 0){
      return;
    }
    else if(pktRxByteOffset < lastPktLen && isLastPkt == 1){
      return;
    }
//    uint8 cnt = 0;
//    if(isLastPkt == 1){
//      cnt = NPI_ReadTransport(pktBuf, lastPktLen);
//    }
//    else{
//      cnt = NPI_ReadTransport(pktBuf, PIC_PKT_LEN);
//    }
//    if(cnt < PIC_PKT_LEN && isLastPkt == 0){
//      return;
//    }
//    else if(cnt < lastPktLen && isLastPkt == 1){
//      return;
//    }
    else{
      //sendNotification(&pktRxByteOffset, 1);
      uint8 sum = 0;
      int y;
      for (y = 0; y < pktRxByteOffset - 2; y++)
      {
        sum += pktBuf[y];
        sum = sum & 0xFF;
      }
      if (sum == pktBuf[ pktRxByteOffset-2 ])
      {
        tmpPktIdx++;
        pktRxByteOffset = 0;
        waitCamera = 0;
        if(tmpPktIdx == pktCnt-1)
          isLastPkt = 1;
        else if(tmpPktIdx == pktCnt){
          globalState = 7;
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof(globalState), &globalState );
          picTotalLen = 0;            	// picture length
          pktCnt = 0;
          tmpPktIdx = 0;
          lastPktLen = 0;
          isLastPkt = 0;
          serialCameraState = 0;
        }
      }
      debug[0] = tmpPktIdx & 0xFF;
      debug[1] = (tmpPktIdx >> 8) & 0xFF;
      debug[2] = pktCnt & 0xFF;
      debug[3] = (pktCnt >> 8)  & 0xFF;
      sendNotification(debug, 4);
    }
  }
  else{
    if(numBytes < 6)
      return;
    NPI_ReadTransport(pktBuf, 6);
    //pktBuf[6] = numBytes;
    //sendNotification(pktBuf, 7);
    sendNotification(pktBuf, 6);
    switch (serialCameraState){
    case 0:{
      if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0e | cameraAddr) && pktBuf[2] == 0x0d && pktBuf[4] == 0 && pktBuf[5] == 0){
        
        ST_HAL_DELAY(1250);
        
        if( NPI_ReadTransport(pktBuf, 6) != 6)
          break;
        
        if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0d | cameraAddr) && pktBuf[2] == 0x00 && pktBuf[3] == 0 && pktBuf[4] == 0 && pktBuf[5] == 0){
          uint8 cmd[] = {0xaa,0x0d|cameraAddr,0x00,0x00,0x00,0x00};
          cmd[1] = 0x0e | cameraAddr;
          cmd[2] = 0x0d;
          sendCmd(cmd, 6);
          serialCameraState = 0x10;
        }
      }
      break;
    }
    case 0x10:{
      if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0e | cameraAddr) && pktBuf[2] == 0x01 && pktBuf[4] == 0 && pktBuf[5] == 0)
        serialCameraState = 0x20;
      break;
    }
    case 0x20:{
      if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0e | cameraAddr) && pktBuf[2] == 0x06 && pktBuf[4] == 0 && pktBuf[5] == 0)
        serialCameraState = 0x21;
      break;
    }
    case 0x21:{
      if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0e | cameraAddr) && pktBuf[2] == 0x05 && pktBuf[4] == 0 && pktBuf[5] == 0)
        serialCameraState = 0x22;
      break;
    }
    case 0x22:{
      if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0e | cameraAddr) && pktBuf[2] == 0x04 && pktBuf[4] == 0 && pktBuf[5] == 0){
//        int i = 0;
//        while(NPI_RxBufLen() < 6 && i < 87000){
//          i++;
//          if(i % 125 == 0)
//            sendNotification(pktBuf, 6);
//        }
//        NPI_ReadTransport(pktBuf, 6);
//        if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0a | cameraAddr) && pktBuf[2] == 0x01){
//          picTotalLen = (pktBuf[3]) | (pktBuf[4] << 8) | (pktBuf[5] << 16);
//          pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
//          if ((picTotalLen % (PIC_PKT_LEN-6)) != 0){
//            pktCnt += 1;
//            lastPktLen =  picTotalLen % (PIC_PKT_LEN-6);
//          }
//          serialCameraState = 0x30;
//        }
        serialCameraState = 0x23;
      }
      break;
    }
    case 0x23:{
      if (pktBuf[0] == 0xaa && pktBuf[1] == (0x0a | cameraAddr) && pktBuf[2] == 0x01){
        picTotalLen = (pktBuf[3]) | (pktBuf[4] << 8) | (pktBuf[5] << 16);
        pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
        if ((picTotalLen % (PIC_PKT_LEN-6)) != 0){
          pktCnt += 1;
          lastPktLen =  picTotalLen % (PIC_PKT_LEN-6) + 6;
        }
        serialCameraState = 0x23;
        waitCamera = 0;
      }
      break;
    }
    case 0x30:{
      break;
    }
    default:{
      break;
    }
    }
  }
//  if((preamble) ) {
//    if(pktRxByteOffset != 0) {
//      numBytes = NPI_RxBufLen();
//      if(numBytes < pktRxByteOffset) {
//        (void)NPI_ReadTransport(pktBuf+(pktLength-pktRxByteOffset+1), numBytes);
//        pktRxByteOffset -= numBytes;
//        return;
//      }
//      else {
//        (void)NPI_ReadTransport(pktBuf+(pktLength-pktRxByteOffset+1), pktRxByteOffset);
//        pktRxByteOffset = 0;
//      }
//    }
//    if(pktRxByteOffset == 0) {
//      //HalLedSet( HAL_LED_1, HAL_LED_MODE_TOGGLE );
//      // got sufficient bytes in a packet
////      serialPacketHandler();
//      (void)sendNotification((uint8 *)&pktBuf, pktLength+1);
//      preamble = FALSE;
//      return;
//    }
//  }
//  
//  numBytes = NPI_RxBufLen();
//   
//  if(numBytes < 3)
//    return;
//   
////   HalLedSet( HAL_LED_1, HAL_LED_MODE_TOGGLE );
//   (void)NPI_ReadTransport((uint8 *)&RxByte, 1);
//   if(RxByte != 0xFF)
//     return;
//   else{
//     (void)NPI_ReadTransport((uint8 *)&RxByte, 1);
//     if(RxByte != 0x7F)
//       return;
//     else { // Then handle the packet depending on type
//              
//       (void)NPI_ReadTransport((uint8 *)&pktBuf, 1);
//       preamble = TRUE;
//       switch(pktBuf[0]) {
//        case 1: //ECG beat
//          pktRxByteOffset = 1;
//          break;
//        case 2: //ACC
//          pktRxByteOffset = 1;
//          break;
//        case 3: //Temp
//          pktRxByteOffset = 2;
//          break;
//        default:
//          preamble = FALSE;
//       }
//       pktLength = pktRxByteOffset;
//     }
//   }
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

void clearRxBuf(void)
{
  NPI_ReadTransport(pktBuf, NPI_UART_RX_BUF_SIZE);
//  uint8 tempBuf;
//  while (NPI_RxBufLen())
//  {
//    NPI_ReadTransport(&tempBuf, 1);
//  }
}

void sendCmd(uint8* cmd, int cmd_len)
{
  HalUARTWrite(NPI_UART_PORT, (uint8*)cmd, cmd_len);
}

void notifyPicInfo(void){
  uint8 tempBuf[20];
  uint8 i;
  for(i = 0; i < 20; i++){
    tempBuf[i] = 0;
  }
  tempBuf[0] = 0xAA;
  tempBuf[1] = seqNum & 0xFF;
  tempBuf[2] = (seqNum >> 8) & 0xFF;
  tempBuf[3] = picTotalLen & 0xFF;
  tempBuf[4] = (picTotalLen >> 8) & 0xFF;
  uint8 sum = 0;
  for(i = 0; i < 19; i++){
    sum += tempBuf[i];
  }
  tempBuf[19] = sum;
  if( 0xFF == sendNotification(tempBuf, 20) )
    waitBLEAck = 1;
  // set send data timer
}

