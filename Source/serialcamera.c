#include "serialcamera.h"
#include "att.h"
#include "hal_uart.h"
#include "hal_led.h"
#include "hal_sensor.h"




uint16 buffer_tail = 0;                         //last data byte sent from SerialBuffer
uint16 buffer_head = 0;

//uint8 uartPkt[PIC_PKT_LEN];



/*
void cameraInitialize(void)
{
    uint8 resp[6];
    uint8 cmd[] = {0xaa,0x0d|cameraAddr,0x00,0x00,0x00,0x00};

    //Serial.setTimeout(500);  
    while (1)
    {
        clearRxBuf();
        
        sendCmd(cmd, 6);
        if(HalUARTRead(NPI_UART_PORT, resp, 6) != 6){          
          continue;
        }
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x0d && resp[4] == 0 && resp[5] == 0)
        {
            if (HalUARTRead(NPI_UART_PORT, resp, 6) != 6)
                continue;
            if (resp[0] == 0xaa && resp[1] == (0x0d | cameraAddr) && resp[2] == 0 && resp[3] == 0 && resp[4] == 0 && resp[5] == 0)
                break;
        }
    }
    cmd[1] = 0x0e | cameraAddr;
    cmd[2] = 0x0d;
    sendCmd(cmd, 6);
}

void preCapture(void)
{
    uint8 cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x00, PIC_FMT };
    uint8 resp[6];

    //Serial.setTimeout(100);
    while (1)
    {
        clearRxBuf();
        sendCmd(cmd, 6);
        if (HalUARTRead(NPI_UART_PORT, resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x01 && resp[4] == 0 && resp[5] == 0) break;
    }
}

void Capture(void)
{
    uint8 cmd[] = { 0xaa, 0x06 | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN>>8) & 0xff ,0};
    uint8 resp[6];

    //Serial.setTimeout(100);
    while (1)
    {
        clearRxBuf();
        sendCmd(cmd, 6);
        if (HalUARTRead(NPI_UART_PORT, resp, 6) != 6)continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x06 && resp[4] == 0 && resp[5] == 0) break;
    }
    cmd[1] = 0x05 | cameraAddr;
    cmd[2] = 0;
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[5] = 0;
    while (1)
    {
        clearRxBuf();
        sendCmd(cmd, 6);
        if (HalUARTRead(NPI_UART_PORT, resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x05 && resp[4] == 0 && resp[5] == 0) break;
    }
    cmd[1] = 0x04 | cameraAddr;
    cmd[2] = 0x1;
    while (1)
    {
        clearRxBuf();
        sendCmd(cmd, 6);
        if (HalUARTRead(NPI_UART_PORT, resp, 6) != 6) continue;
        if (resp[0] == 0xaa && resp[1] == (0x0e | cameraAddr) && resp[2] == 0x04 && resp[4] == 0 && resp[5] == 0)
        {
            //Serial.setTimeout(1000);
            if (HalUARTRead(NPI_UART_PORT, resp, 6) != 6) continue;
            if (resp[0] == 0xaa && resp[1] == (0x0a | cameraAddr) && resp[2] == 0x01)
            {
                picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
                pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
                if ((picTotalLen % (PIC_PKT_LEN-6)) != 0){
                    pktCnt += 1;
                    lastPktLen =  picTotalLen % (PIC_PKT_LEN-6);
                }
                break;
            }
        }
    }
}

void sendPicInfo(void){
}

void getData(void)
{
    uint8 cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };

    //Serial.setTimeout(1000);
    //for (unsigned int i = 0; i < pktCnt; i++)
    //{
    
    cmd[4] = tmpPktIdx & 0xff;
    cmd[5] = (tmpPktIdx >> 8) & 0xff;

    int retry_cnt = 0;
    retry:
        //delay(10);
    clearRxBuf();
    sendCmd(cmd, 6);
        //uint16 cnt = Serial.readBytes((char *)pkt, PIC_PKT_LEN);
    uint16 cnt = HalUARTRead(NPI_UART_PORT, uartPkt, PIC_PKT_LEN);
    buffer_tail = cnt;
    uint8 sum = 0;
    int y;
    for (y = 0; y < cnt - 2; y++)
    {
        sum += uartPkt[y];
    }
    if (sum != uartPkt[cnt-2])
    {
        if (++retry_cnt < 100) goto retry;
    }

        //myFile.write((const uint8_t *)&pkt[4], cnt-6);
        //if (cnt != PIC_PKT_LEN) break;
    //}


    tmpPktIdx++;
}

void resetSerialCamera(void){
    uint8 cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
    cmd[4] = 0xf0;
    cmd[5] = 0xf0;
    sendCmd(cmd, 6);
    picNameNum ++;
}

uint8 sendData(uint16 diff)
{
    //can send max 4 packets per connection interval
    uint8 packets_sent = 0;
    //ensure queue of notification is successful
    bool send_error = FALSE;
    //return value to update tail and send ack to msp
    uint8 bytes_sent = 0;
  
    attHandleValueNoti_t noti;      
    //dummy handle
    noti.handle = 0x2E;  
  
    //counter
    uint8 i;
  
    while ((packets_sent < 4) &&  (diff >= 20) && (send_error == FALSE))
    {  
        //send 20 bytes
        noti.len = 20;
        for (i = 0; i < 20; i++)
        {
            noti.value[i] = uartPkt[circular_add(buffer_tail , bytes_sent+i)];
        }
        //connection handle currently hardcoded
        if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
        {
            bytes_sent += 20;
            diff -= 20;
            packets_sent++;
        }
        else
        {
            send_error = TRUE;
        }
    }
    //send remaining bytes  
    if ((packets_sent < 4) && (diff > 0) && (send_error == FALSE))
    {
        noti.len = diff;
        for (i = 0; i < diff; i++)
        {
            noti.value[i] = uartPkt[circular_add(buffer_tail, bytes_sent + i)];
        }
        if (!(GATT_Notification(0, &noti, FALSE))) //if sucessful
        {
            bytes_sent += i;
            diff -= i;//amount of data sent
        }
        else
        {
            send_error = TRUE;
        }
    }
    return bytes_sent;
}
*/