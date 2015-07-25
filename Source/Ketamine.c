/**************************************************************************************************
  Filename:       Ketamine.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include <stdio.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_i2c.h"
#include "hal_sensor.h"
#include "hal_uart.h"
#include "npi.h"
#include "TCS3414CS.h"
#include "eeprom.h"
#include "Application.h"


#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "Ketamine.h"
#include "serialInterface.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define KTM_BROADCAST_EVT_PERIOD                        500
#define KTM_PERIODIC_EVT_PERIOD                         1000
#define KTM_DEFAULT_EVT_PERIOD                          1100
#define KTM_CHECKINTERRUPT_PEROID                       500
#define KTM_SENDDATA_PERIOD                             400
#define KTM_DUMMY_TIMER_PERIOD                          3000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         5

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 Ketamine_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x08,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x6b,   // 'k'
  0x65,   // 'e'
  0x74,   // 't'
  0x5f,   // '-'
  0x30,   // '0'
  0x30,   // '0'
  0x33,   // '4'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  //DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

static uint8 version = 3;
static bool isAwake = false;
static int globalCount = 0;
static uint8 directTerminate = 0;
int advCount = 0; 
uint8 globalState = 1;
int advMax = 600;
int globalMax = 120;
uint8 disconnectCnt = 0;
uint8 resetFlag = 0;
uint8 interruptCounter = 0;
int lastInterruptTime = 0;
uint8 lowPowerWarnCount = 0;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Ketamine_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void defaultCheckTask( void );
static void simpleProfileChangeCB( uint8 paramID );
static void systemWakeUp( void );
static void systemSleep( void );
static void takePicture(uint8 mode);
void initialParameter(void);
void getPictureData();
void parseBLECmd(uint8 value);

static void Ketamine_HandleKeys( uint8 shift, uint8 keys );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t Ketamine_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t Ketamine_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t Ketamine_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Ketamine_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Ketamine_Init( uint8 task_id )
{
  Ketamine_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    //uint16 gapRole_AdvertOffTime = 0;
      
    // Advertisement time : 1.9sec
    uint16 gapRole_AdvertOffTime = 0;
    

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 1, 2, 3, 4, 5 };

    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
  }

  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  P0SEL = 0;
  P0DIR = 0x9E;
  P0 = 0x01;

  P1SEL = 0;
  P1DIR = 0xFF;
  P1 = 0x04;
  
  
  
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_ON );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( Ketamine_TaskID );

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &Ketamine_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

//  //turn on overlapped processing
  HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
  HCI_EXT_OverlappedProcessingCmd(HCI_EXT_ENABLE_OVERLAPPED_PROCESSING);
//  
//  //disable halt during RF (needed for UART / SPI)
//  HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE);
  
  // Setup a delayed profile startup
  osal_set_event( Ketamine_TaskID, KTM_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      Ketamine_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 Ketamine_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( Ketamine_TaskID )) != NULL )
    {
      Ketamine_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & KTM_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &Ketamine_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &Ketamine_BondMgrCBs );
    
    isAwake = true;
    osal_start_timerEx( Ketamine_TaskID, KTM_PERIODIC_EVT, KTM_BROADCAST_EVT_PERIOD);
    osal_start_timerEx( Ketamine_TaskID, KTM_DEFAULT_EVT, KTM_DEFAULT_EVT_PERIOD);
    
    HalLedSet( HAL_LED_1| HAL_LED_2| HAL_LED_3| HAL_LED_4, HAL_LED_MODE_OFF );

    return ( events ^ KTM_START_DEVICE_EVT );
  }

  if ( events & KTM_DEFAULT_EVT ){
    defaultCheckTask();
    osal_start_timerEx( Ketamine_TaskID, KTM_DEFAULT_EVT, KTM_DEFAULT_EVT_PERIOD);
    return (events ^ KTM_DEFAULT_EVT);
  }
  
  if ( events & KTM_PERIODIC_EVT )
  {
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      if(resetFlag == 1){
        initialParameter();
        resetFlag = 0;
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof(globalState), &globalState );
      }
      
      if(directTerminate == 1){
        initialParameter();   // 7/26
        directTerminate = 0;
        uint8 disabled = FALSE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &disabled );
        osal_stop_timerEx(Ketamine_TaskID, KTM_DEFAULT_EVT);
        HalLedSet( HAL_LED_1| HAL_LED_2| HAL_LED_3| HAL_LED_4, HAL_LED_MODE_OFF );
        isAwake = false;  
        return (events ^ KTM_PERIODIC_EVT);
      }
   
      advCount = advCount + 1;
      HalLedSet( HAL_LED_2 , HAL_LED_MODE_ON );
      ST_HAL_DELAY(1000);
      HalLedSet( HAL_LED_2 , HAL_LED_MODE_OFF );
      
      if(advCount >= advMax){
        // DEBUG
        int debugCount = 0;
        for(; debugCount < 5; debugCount++){
          HalLedSet( HAL_LED_3 , HAL_LED_MODE_ON );
          ST_HAL_DELAY(1000);
          HalLedSet( HAL_LED_3 , HAL_LED_MODE_OFF );
        }
 
        HalLedSet( HAL_LED_1| HAL_LED_2| HAL_LED_3| HAL_LED_4, HAL_LED_MODE_OFF );
        advCount = 0;
        //globalState = 1;
        globalCount = 0; 
        uint8 disabled = FALSE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &disabled );
        osal_stop_timerEx(Ketamine_TaskID, KTM_DEFAULT_EVT);
        isAwake = false;
        initialParameter();
        InitBoard( OB_READY );
        return (events ^ KTM_PERIODIC_EVT);
      }
      
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
        if ( KTM_PERIODIC_EVT_PERIOD )
        {
          osal_start_timerEx( Ketamine_TaskID, KTM_PERIODIC_EVT, KTM_BROADCAST_EVT_PERIOD );
        }
      }
      else
      {
        new_adv_enabled_status = FALSE;
        if ( KTM_PERIODIC_EVT_PERIOD )
        {
          osal_start_timerEx( Ketamine_TaskID, KTM_PERIODIC_EVT, KTM_BROADCAST_EVT_PERIOD/10  ); // adjust duty cycle 8000
        }
      }
      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }
    
    
    // Restart timer
    else if( KTM_PERIODIC_EVT_PERIOD )
    {
      advCount = 0;
      performPeriodicTask();
      if((globalState == 6 || globalState == 3) && (serialCameraState == 0x30 || serialCameraState == 0x31)){
        osal_start_timerEx( Ketamine_TaskID, KTM_PERIODIC_EVT, KTM_SENDDATA_PERIOD);
      }
      else{
        osal_start_timerEx( Ketamine_TaskID, KTM_PERIODIC_EVT, KTM_PERIODIC_EVT_PERIOD);  // adjust duty cycle
      }
    }

    return (events ^ KTM_PERIODIC_EVT);
  }
  if ( events & KTM_CHECKINTERRUPT_EVT){
    HalAdcInit ();
    HalAdcSetReference (HAL_ADC_REF_AVDD);
    uint16 adcvalue = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_8);
//    if(gapProfileState == GAPROLE_CONNECTED){
//      uint8 buf[2];
//      attHandleValueNoti_t noti;
//      buf[0] = adcvalue & 0xFF;
//      buf[1] = (adcvalue >> 8) & 0xFF;
//      sendReadBuf(&noti, buf, 2, 0xBB);
//    }
    if(adcvalue > 5){
      InitBoard( OB_READY );
      interruptCounter = 0;
      return (events ^ KTM_CHECKINTERRUPT_EVT);
    }
    else{
      interruptCounter++;
      if(interruptCounter < 3){
        osal_start_timerEx( Ketamine_TaskID, KTM_CHECKINTERRUPT_EVT, KTM_CHECKINTERRUPT_PEROID);
      }
      else{
        if(isAwake == true){
          if(globalState != 6 && globalState != 3 && globalState != 2 && globalState != 7){
            systemSleep();
          }
        }
        else{
          systemWakeUp();
        }
        //lastInterruptTime = osal_GetSystemClock();
        //osal_start_timerEx( Ketamine_TaskID, KTM_DUMMY_EVT, KTM_DUMMY_TIMER_PERIOD);
        interruptCounter = 0;
      }
      InitBoard( OB_READY );
      return (events ^ KTM_CHECKINTERRUPT_EVT);
    }
  }
  
  if (events & KTM_DUMMY_EVT){
    return (events ^ KTM_DUMMY_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      Ketamine_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Ketamine_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      Ketamine_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

  default:
    // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      Ketamine_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void Ketamine_HandleKeys( uint8 shift, uint8 keys )
{
    interruptCounter = 0;
  //int currentTime = osal_GetSystemClock();
  //if(currentTime - lastInterruptTime > KTM_DUMMY_TIMER_PERIOD){
    uint8 result =  osal_stop_timerEx(Ketamine_TaskID, KTM_CHECKINTERRUPT_EVT);
    if(result == INVALID_EVENT_ID){
      osal_start_timerEx( Ketamine_TaskID, KTM_CHECKINTERRUPT_EVT, KTM_CHECKINTERRUPT_PEROID);
    }
    InitBoard( OB_READY );
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{ 
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

      }
      break;

    case GAPROLE_ADVERTISING:
      {
      }
      break;

    case GAPROLE_CONNECTED:
      {
        //reset adv counter once connected
        advCount = 0;
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
      }
      break;      
    case GAPROLE_WAITING:
      {
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
      }
      break;

    case GAPROLE_ERROR:
      {
        InitBoard( OB_READY ); // Try
      }
      break;

    default:
      {
      }
      break;

  }

  gapProfileState = newState;

//#if !defined( CC2540_MINIDK )
//  VOID gapProfileState;     // added to prevent compiler warning with
//                            // "CC2540 Slave" configurations
//#endif
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the KTM_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */    

/*Variables for application control*/
bool eepResult = false;
uint16 firstThres = 75;
uint16 secondThres = 60;
bool isfirstSaliva = FALSE;
bool isSecondSaliva = FALSE;
uint8 waitCamera = 0;
uint8 serialCameraState = 0;


static void performPeriodicTask( void )
{
  attHandleValueNoti_t noti;
  uint8 buf[20];
  int i;
  for(i = 0; i < 20; i++){
    buf[i] = 0;
  }
  
  globalCount++;
  
//  if(globalCount > globalMax){
//    // Terminate after globalState is not changed for 2 min.
//    initialParameter();
//    GAPRole_TerminateConnection();
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof(globalState), &globalState );
//    globalCount = 0;
//  }

  
  switch (globalState){
  case 1:
    initialParameter();
    break;
    
  case 2:
    //HalLedSet( HAL_LED_1 , HAL_LED_MODE_ON );
    //ST_HAL_DELAY(1000);
    //HalLedSet( HAL_LED_1 , HAL_LED_MODE_OFF );
    HalAdcInit ();
    HalAdcSetReference (HAL_ADC_REF_AVDD);
    uint16 adcvalue = HalAdcRead (HAL_ADC_CHANNEL_5, HAL_ADC_RESOLUTION_8);
    buf[0] = adcvalue & 0xFF;
    buf[1] = (adcvalue >> 8) & 0xFF;
    if(isSecondSaliva == FALSE){
      if(isfirstSaliva == FALSE){
        sendReadBuf(&noti, buf, 2, 0xFC);
        if(adcvalue > firstThres)
          isfirstSaliva = TRUE;
      }else{
        sendReadBuf(&noti, buf, 2, 0xFD);
        if(adcvalue < secondThres)
          isSecondSaliva = TRUE;
      }
    }else{
      sendReadBuf(&noti, buf, 2, 0xFE);
    }
    break;
    
  case 3:{
    HalUARTInit(); 
    NPI_InitTransport(cSerialPacketParser);
    P1_3 = 1;
    ST_HAL_DELAY(1250);  
    takePicture(KTM_PIC_PRECAPTURE);
    break;
  } 
  
  case 4:
    if( disconnectCnt == 0){
      GAPRole_TerminateConnection();
      disconnectCnt++;
      resetFlag = 1;
    }
    
    break;
    
  case 5:
    directTerminate = 1;
    if( disconnectCnt == 0){
      GAPRole_TerminateConnection();
      disconnectCnt++;
      resetFlag = 1;
    }

    break;
    
  case 6:{
    HalUARTInit(); 
    NPI_InitTransport(cSerialPacketParser);
    P1_3 = 1;
    ST_HAL_DELAY(1250);
    takePicture(KTM_PIC_CAPTURE);
    
    break;
  }
  case 7:{
    HalUARTSuspend();
    P1_3 = 0;
    osal_pwrmgr_device(PWRMGR_BATTERY);
    HalLedSet( HAL_LED_1 | HAL_LED_2 , HAL_LED_MODE_ON );
    ST_HAL_DELAY(1000);
    HalLedSet( HAL_LED_1 | HAL_LED_2, HAL_LED_MODE_OFF );
    break;
  }
  case 8:{
    osal_pwrmgr_device( PWRMGR_BATTERY );
    break;
  }
  case 9:{
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
    break;
  }
  case 10:{
    sendReadBuf(&noti, &version, 1, 0xDD);
    break;
  }
  default:
    break;
  }
}

static void defaultCheckTask( void ){
  attHandleValueNoti_t noti;
  uint8 buf[20];
  eepResult = i2c_eeprom_read_buffer(EEPROM_ADDR, 0, buf, 4);
  if(eepResult == TRUE ){
    if(gapProfileState == GAPROLE_CONNECTED ){
      sendReadBuf(&noti, buf, 4, 0xFB);
    }
    if( globalState == 2 || globalState == 3 || globalState == 6 || globalState == 7){
      HalLedSet( HAL_LED_4, HAL_LED_MODE_TOGGLE );
    }
    else{
      HalLedSet( HAL_LED_4 , HAL_LED_MODE_ON );
    }
  }
  else{
    if(gapProfileState == GAPROLE_CONNECTED ){
      sendReadBuf(&noti, buf, 0, 0xFA);
    }
    HalLedSet( HAL_LED_4, HAL_LED_MODE_OFF);
  }
  
  if(isAwake == true){
    if(P0_3 == 0){
      HalLedSet( HAL_LED_3 , HAL_LED_MODE_ON);
    }
    HalAdcInit ();
    HalAdcSetReference (HAL_ADC_REF_AVDD);
    uint16 adcvalue = HalAdcRead (HAL_ADC_CHANNEL_6, HAL_ADC_RESOLUTION_10);
    //buf[0] = adcvalue & 0xFF;
    //buf[1] = (adcvalue >> 8) & 0xFF;
    //sendReadBuf(&noti, buf, 2, 0xBB);
    if(adcvalue > 12){
      lowPowerWarnCount = 0;
    }
    else{
      lowPowerWarnCount++;
      ST_HAL_DELAY(1000);
      HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );
      if(lowPowerWarnCount > 15){
        systemSleep();
      }
    }
  }
  else{
    HalLedSet( HAL_LED_3 , HAL_LED_MODE_OFF);
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{ 
  uint8 newValue;
  globalCount = 0;
  
  switch( paramID )
  {
  case SIMPLEPROFILE_CHAR1:{
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      globalState = newValue;
      
      if(newValue == 6 || newValue == 3){
        serialCameraState = 0;
        waitCamera = 0;
        waitBLEAck = 0;
      }

      break;
  }
  case SIMPLEPROFILE_CHAR3:{
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );
      parseBLECmd(newValue);
      
      break;
  }
  case SIMPLEPROFILE_CHAR6:{
      uint8 command[SIMPLEPROFILE_CHAR6_LEN];
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, command);
      if(command[0] == 0xA1){
        writeTestPaperId(command+1, 4);
      }
      else if(command[0] == 0xA3){
        retransmitSize = command[1];
        tmpRetransmitIdx = 0;
        int i;
        for(i = 0; i < retransmitSize; i++){
          retransmitBuf[i] = command[2+i];
        }
        tmpPktIdx = retransmitBuf[0];
        if(tmpPktIdx == pktCnt-1){
          isLastPkt = 1;
        }
      }
      break;
  }
  default:{
      // should not reach here!
    break;
  }
  }
}

static void systemWakeUp( void ){
  uint8 result1 = osal_stop_timerEx(Ketamine_TaskID, KTM_PERIODIC_EVT);
  uint8 result2 = osal_stop_timerEx(Ketamine_TaskID, KTM_DEFAULT_EVT);
  if(result1 == INVALID_EVENT_ID){
    initialParameter();
  }
  isAwake = true;
  osal_start_timerEx( Ketamine_TaskID, KTM_PERIODIC_EVT, KTM_BROADCAST_EVT_PERIOD);
  osal_set_event( Ketamine_TaskID, KTM_DEFAULT_EVT);
}

static void systemSleep( void ){
  uint8 disabled = FALSE;
  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &disabled );
  osal_stop_timerEx(Ketamine_TaskID, KTM_PERIODIC_EVT);
  osal_stop_timerEx(Ketamine_TaskID, KTM_DEFAULT_EVT);
  if( gapProfileState == GAPROLE_CONNECTED )
  {
    GAPRole_TerminateConnection();
  }
  HalLedSet( HAL_LED_1| HAL_LED_2| HAL_LED_3| HAL_LED_4, HAL_LED_MODE_OFF );
  initialParameter();
  isAwake = false;  
}

/*********************************************************************
*********************************************************************/

void initialParameter(void){
  isfirstSaliva = FALSE;
  isSecondSaliva = FALSE;
  eepResult = false;
  globalState = 1;
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof(globalState), &globalState );
  disconnectCnt = 0;
  P1_3 = 0;
  //InitBoard( OB_READY );
}

void parseBLECmd(uint8 value){
  if( (value & 0xF0) == 0){
    if(waitBLEAck != 0)
      waitBLEAck = value;
  }
  else{
    //requestDataFrom(value & 0x0F);
    tmpPktIdx = value & 0x0F;
    getPictureData();
    //waitBLEAck = 0xF0;
  }
}

static void takePicture(uint8 mode){
  attHandleValueNoti_t noti;
  uint8 buf[20];
  
  switch(mode){
  case KTM_PIC_PRECAPTURE:{
    HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
    HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
    break;
  }
  case KTM_PIC_CAPTURE:
  default:{
    HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
    HalLedSet(HAL_LED_2, HAL_LED_MODE_OFF);
    break;
  }
  }
  
  switch (serialCameraState){
    case 0:{
      osal_pwrmgr_device(PWRMGR_ALWAYS_ON);
      uint8 cmd[] = {0xaa,0x0d|cameraAddr,0x00,0x00,0x00,0x00};
      clearRxBuf();
      sendCmd(cmd, 6);
      break;
    }
    case 0x10:{
      uint8 cmd[] = { 0xaa, 0x01 | cameraAddr, 0x00, 0x07, 0x00, PIC_FMT };
      clearRxBuf();
      sendCmd(cmd, 6);
      break;
    }
    case 0x20:{
      uint8 cmd[] = { 0xaa, 0x06 | cameraAddr, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN>>8) & 0xff ,0};
      clearRxBuf();
      sendCmd(cmd, 6);
      break;
    }
    case 0x21:{
      uint8 cmd[] = { 0xaa, 0x05 | cameraAddr, 0, 0, 0, 0};
      clearRxBuf();
      sendCmd(cmd, 6);
      break;
    }
    case 0x22:{
      uint8 cmd[] = { 0xaa, 0x04 | cameraAddr, 0x1, 0, 0, 0};
      clearRxBuf();
      sendCmd(cmd, 6);
      break;
    }
    case 0x23:{
      break;
    }
    case 0x24:{
      HalLedSet(HAL_LED_1, HAL_LED_2| HAL_LED_MODE_OFF);
      if(waitBLEAck == 5){
        serialCameraState = 0x30;
        waitBLEAck = 0;
      }
      else{
        notifyPicInfo();
      }
      break;
    }
    case 0x30:{
      if(tmpPktIdx < pktCnt){
        getPictureData();
      }
      
      if(tmpPktIdx == pktCnt-1){
        isLastPkt = 1;
      }
      else if(tmpPktIdx == pktCnt){
        sendReadBuf(&noti, buf, 0, 0xA9);
        serialCameraState = 0x31;
      }
      else{
        isLastPkt = 0;
      }
     
      break;
    }
    case 0x31:{
      if(retransmitSize != 0){
        if(tmpRetransmitIdx < retransmitSize){
          if(tmpPktIdx == pktCnt-1){
            isLastPkt = 1;
          }
          getPictureData();
        }
        else{
          sendReadBuf(&noti, buf, 0, 0xA9);
          retransmitSize = 0;
          tmpRetransmitIdx = 0;
        }
      }
      if(waitBLEAck == 5){
        waitBLEAck = 0;
        waitCamera = 0;
        globalState = 7;
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof(globalState), &globalState );
        serialCameraState = 0;
      }
      
      waitCamera++;
      waitCamera %= 20;
      if(waitCamera == 19){
        waitBLEAck = 0;
        waitCamera = 0;
        globalState = 7;
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof(globalState), &globalState );
        serialCameraState = 0;
      }
      
      break;
    }
  }
}

void getPictureData(){
  uint8 cmd[] = { 0xaa, 0x0e | cameraAddr, 0x00, 0x00, 0x00, 0x00 };
  cmd[4] = tmpPktIdx & 0xff;
  cmd[5] = (tmpPktIdx >> 8) & 0xff;
          
  clearRxBuf();
  pktRxByteOffset = 0;
  sendCmd(cmd, 6);
}
