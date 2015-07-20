#include "hal_uart.h"
#include "OSAL.h"
#include "npi.h"

#define MAX_PKT_SIZE    128
#define RX_BUFF_SIZE    500

#define PIC_PKT_LEN    128					//data length of each read, dont set this too big because ram is limited
#define PIC_FMT_VGA    5
#define PIC_FMT_CIF    5
#define PIC_FMT_OCIF   3
#define CAM_ADDR       0
#define PIC_FMT        PIC_FMT_VGA

//===================================================

/* States for CRC parser */
typedef enum {
  SERIAL_STATE_START = 0,
  SERIAL_STATE_TYPE,
  SERIAL_STATE_LEN,
  SERIAL_STATE_DATA,
  SERIAL_STATE_COMPLETE //received complete serial message
} serial_state_t;

void cSerialPacketParser(uint8 port, uint8 events);
void parseCmd(void);
void sendSerialEvt(void);
void notifyPicInfo(void);
uint8 sendData(uint16 diff);


/**********************************************************************
 * MACROS
 */

//global

extern uint16 serialBufferOffset;
extern uint8 serialBuffer[RX_BUFF_SIZE];
extern uint8 pktBuf[RX_BUFF_SIZE];
extern uint8 cameraAddr;
extern uint8 bleAck;
extern uint16 picTotalLen;            	// picture length
extern uint16 pktCnt;
extern uint16 tmpPktIdx;
extern uint16 lastPktLen;
extern uint8 pktRxByteOffset;
extern uint8 waitBLEAck;
extern uint8 isLastPkt;

extern uint16 retransmitSize;
extern uint16 tmpRetransmitIdx;
extern uint16 retransmitBuf[18];


/*********************************************************************
 * FUNCTIONS
 */

/**********************************************************************
 * Task Initialization for the BLE Application
 */
extern void SerialInterface_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */

extern uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events );
extern uint8 sendNotification(uint8* bytes_sent, uint8 len);
extern uint8 sendAckMessage(uint8 bytes_sent);
extern uint8 sendDataToHost(uint8* data, uint8 len);  
extern uint16 circular_add(uint16 x, uint16 y);
extern uint16 circular_diff(uint16 offset, uint16 tail);
extern void clearRxBuf(void);
extern void sendCmd(uint8* cmd, int cmd_len);
extern void requestDataFrom(uint8 idx);
