/******************************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PICmicro(r) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PICmicro Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
********************************************************************/
#include "HA_DefinedValues.h"
#include "bt_app.h"
#include "bt_command_send.h"
#include "bt_command_decode.h"
#include "em_gpio.h"
#include "ble.h"
#include "line_in.h"
#include "HA300_BSP.h"
#include "key.h"

#if 0
//===============================================
//  VARIABLES CLAIM
//===============================================
// @ bluetooth task state define
enum {
    BT_STATE_INITIALIZE_START = 0,
    BT_STATE_INIT_MFB_HIGH,
    BT_STATE_INIT_RESET_HIGH,
    BT_STATE_INIT_RESET_HIGH_WAIT,
    BT_STATE_INIT_COMMAND_START,    //send first command
    BT_STATE_INIT_SETUP_GPIO,
    BT_STATE_INIT_WAIT_GPIO_EVENT,
    BT_STATE_INIT_READ_DEVICE_ADDR,
    BT_STATE_INIT_READ_DEVICE_ADDR_WAIT,
	BT_STATE_INIT_READ_DEVICE_NAME,
	BT_STATE_INIT_READ_DEVICE_NAME_WAIT,
	BT_STATE_INIT_READ_5506_FEATURE,
	BT_STATE_INIT_READ_5506_FEATURE_WAIT,
    BT_STATE_INIT_BLE_ADV_START,
	BT_STATE_INIT_BLE_ADV_WAIT,

    BT_STATE_POWER_ON_START,        //wait 500ms. after 500ms, send SYSTEM_ON and CSB_PAGE command   15
    BT_STATE_POWER_ON_BUTTON_PRESSED,
    BT_STATE_POWER_ON,          //system on finished, wait for BT or AUX instruction
    BT_STATE_BLE_ADV_WAIT,		// 18
    BT_STATE_VOL_SYNC,
	BT_STATE_INIT_NSPK_STATUS,
	BT_STATE_INIT_NSPK_STATUS_WAIT,    
    BT_STATE_READ_PAIR_RECORD,
    BT_STATE_READ_PAIR_RECORD_WAIT,
    BT_STATE_READ_LINKED_MODE,
    BT_STATE_READ_LINKED_MODE_WAIT,
    BT_STATE_LINKBACK_START,
    BT_STATE_LINKBACK_TO_LAST_DEVICE,
    BT_STATE_BT_RUNNING,	//28

    BT_STATE_POWER_OFF_START,   // 29
    BT_STATE_POWER_OFF_START_NSPK,
    BT_STATE_POWER_OFF_WAIT,
    BT_STATE_POWER_OFF_WAIT_NSPK,
    BT_STATE_POWER_OFF_WAIT2,
    BT_STATE_POWER_OFF_WAIT2_NSPK,
    BT_STATE_POWER_OFF_WAIT_NSPK_EVENT,
    BT_STATE_POWER_OFF	//36
} BTAPP_TaskState;
#endif 

extern uint8_t	BT_nspk_link;
extern uint8_t	BT_nspk_event;
extern uint8_t BT_nspk_status;

extern uint8_t ang_input;   // 0 : Line In connect, 1 : Line In Disconnect
	


static uint8_t BTAPP_Request;

uint16_t BTAPP_timer1ms;
bool KEY_timer1ms=false;
//uint16_t KEY_count_timer1ms=0;

uint16_t KEY_Button_timer1ms=0;

uint16_t BTEQ_timer1ms;
uint16_t BTMute_timer1ms;

uint8_t BTEQ_SendIndex;
uint8_t BTEQ_RecIndex;

uint8_t BTEQ_Debug;


uint8_t BT_LinkStatus;          // link status

uint8_t	BT_nspk_link;
uint8_t	BT_nspk_event;
uint8_t BT_nspk_status;

uint8_t BTAPP_task_status;

uint8_t Print_BT_CMD_SendState;

typedef struct {
    uint8_t LinkedProfileStatus;
    uint8_t DeviceID_LinkIndex;
} CONNECTION_STATUS;
#define CONNECTION_MAX 3
struct {
    uint8_t activeIndex;
    CONNECTION_STATUS allConnection[CONNECTION_MAX];
} BT_AllConnections;

enum {
    BT_LINKBACK_INIT,       //init
    BT_LINKBACK_CONNECTING, //sent link_back command but no event yet
    BT_LINKBACK_OK,         //link back success, event
    BT_LINKBACK_FAILED,     //linkback failed, event
    BT_PAIRING_START,       //sent pair command but no event yet
    BT_PAIRING,             //pair event
    BT_PAIRING_OK,          //pair event
    BT_PAIRING_FAILED,      //pair event
    BT_LINK_CONNECTED,         //other event
} BT_LinkbackStatus;       // linkback status




//@
enum {
    LINE_IN_INACTIVE = 0,
    LINE_IN_ACTIVE,
    LINE_IN_ACTIVE_WITH_AUDIO,
    LINE_IN_WITH_SILENCE_AUDIO
} LineInStatus;
//@
enum {
    BT_A2DP_IDLE = 0,
    BT_A2DP_ACTIVE = 1
} BT_A2DPStatus;
//@
enum {
    BT_PLAYING_STOPPED,
    BT_PLAYING_PLAYING,
    BT_PLAYING_PAUSED,
    BT_PLAYING_FF,
    BT_PLAYING_FR,
    BT_PLAYING_ERROR
} BT_PlayingStatus;

uint8_t BT_pairedRecordNumber;
extern uint8_t BT_linkIndex;
uint8_t BT_linkedMode;
uint8_t BT_nSPKLinkedCounter;
//some flags
union{
    uint16_t value;
    struct {
        uint16_t updateA2DPGainReq           : 1;
        uint16_t updateHFPGainReq            : 1;
        uint16_t updateLineInGainReq         : 1;
        uint16_t linkbackToDevAddr           : 1;
        uint16_t musicCtrlReq_00             : 1;
        uint16_t musicCtrlReq_01             : 1;
        uint16_t musicCtrlReq_03             : 1;
        uint16_t musicCtrlReq_07             : 1;
        uint16_t SPPLinkBackReq              : 1;
    };
} NextCommandReq;

union{
    uint16_t value;
    struct {
        uint16_t SpeakerAddCommandReq        : 1;       //0xE2(NSPK)
        uint16_t BroadcastModeCommandReq     : 1;       //0xE2(broadcast)
        uint16_t NSpeakerTriggerMasterReq    : 1;       //0xE0
        uint16_t NSpeakerTriggerSlaveReq     : 1;       //0xE1
        uint16_t BroadcastTriggerMasterReq   : 1;       //0xE0
        uint16_t BroadcastTriggerSlaveReq    : 1;       //0xE1
        uint16_t PairReq                    : 1;        //ANY_MODE_ENTERING_PAIRING
        uint16_t ResetEEPROMReq             : 1;        //RESET_EEPROM_SETTING
        uint16_t NextSongReq                : 1;        //NEXT_SONG
        uint16_t PreviousSongReq            : 1;        //PREVIOUS_SONG
        uint16_t IncSpkGainReq              : 1;        //INC_SPK_GAIN
        uint16_t DecSpkGainReq              : 1;        //DEC_SPK_GAIN
        uint16_t AcceptCallReq              : 1;        //ACCEPT_CALL
        uint16_t ForceEndCallReq            : 1;        //FORCE_END_CALL
        uint16_t LastNumberRedialReq        : 1;        //LAST_NUMBER_REDIAL
        uint16_t RejectCallReq              : 1;        //REJECT_CALL
    };
} NextMMIActionReq;

union{
    uint16_t value;
    struct {
        uint16_t TransferCallReq            : 1;        //0x0E
        uint16_t VoiceDialReq               : 1;        //0x0A
        uint16_t cancelVoiceDialReq         : 1;        //0x0B
        uint16_t PlayPauseReq               : 1;        //0x32
        uint16_t CancelNSPKReq              : 1;        //0xE3
        uint16_t TerminateNSPKReq           : 1;        //0xE4
        uint16_t TerminateCancelNSPKReq     : 1;        //0xE5
        uint16_t switchNSPKChannel          : 1;        //0xEC
        uint16_t enterNSPKModeReq           : 1;        //0xF4
        uint16_t enterBroadcastModeReq      : 1;        //0xF5
        uint16_t MMI_F6_Req                 : 1;        //0xF6
        uint16_t MMI_F7_Req                 : 1;        //0xF7
    };
} NextMMIActionReq2;

//timer and request
/*struct {
    uint16_t ReqDisableDiscoverableTimer;
    //...
    union{
        uint16_t allReq;
        struct {
            uint16_t    ReqDisableDiscoverable  : 1;
            //...
        };
    };
} TimeCommandReq;*/

uint8_t BT_localBDAddr[6];
uint8_t BT_masterBDAddr[6];
uint8_t linkbackBDAddr[6];

typedef enum {
    A2DP = 0,
    HFP,
    LINE_IN,
    ALL_MODE = 0xff
} VOLUME_MODE;
#define ALL_MUTE		0xff
#define ALL_UNMUTE		0xfe

struct {
    uint8_t a2dpVol;
    uint8_t hfpVol;
    uint8_t lineInVol;
    VOLUME_MODE currentVolMode;
} BTAPP_Volume;

union {
    uint8_t PORT_1;
    struct{
        uint8_t PORT_1_0    : 1;
        uint8_t PORT_1_1    : 1;
        uint8_t PORT_1_2    : 1;
        uint8_t PORT_1_3    : 1;
        uint8_t PORT_1_4    : 1;
        uint8_t PORT_1_5    : 1;
        uint8_t PORT_1_6    : 1;
        uint8_t PORT_1_7    : 1;
    };
} BT_Port1;

union {
    uint8_t PORT_3;
    struct{
        uint8_t PORT_3_0    : 1;
        uint8_t PORT_3_1    : 1;
        uint8_t PORT_3_2    : 1;
        uint8_t PORT_3_3    : 1;
        uint8_t PORT_3_4    : 1;
        uint8_t PORT_3_5    : 1;
        uint8_t PORT_3_6    : 1;
        uint8_t PORT_3_7    : 1;
    };
} BT_Port3;
enum {
    BLE_CONN_STANDBY = 0,
    BLE_CONN_ADV,
    BLE_CONN_SCAN,
    BLE_CONN_CONNECTED
} BLEConnectedStatus;

#define BT_SPP_RxFifoSize 50                                       //receive buffer max. size
 uint8_t BT_SPP_RxFifo[BT_SPP_RxFifoSize];                          //receive buffer
 uint32_t BT_SPP_RxFifoHead, BT_SPP_RxFifoTail, BT_SPP_RxCounter;
 uint32_t BT_SPP_State=0;           //this is test variable, don't need to migrate the code
 uint8_t ble_connect_state=0;
 uint8_t dspEQ[DSP_EQ_SIZE+6];

 
 enum {
	 MULTI_SPK_EQ_ALL = 0,
	 MULTI_SPK_EQ_MASTER,
	 MULTI_SPK_EQ_SLAVE,
 };
 uint8_t multiSpk_eq_route;
 BT_SYS_STATUS BT_SystemStatus;

///////////////////////////////////////////////////////////////////////////////
 //=======================================================================
//  FUNCTION IMPLEMENTION
//========================================================================
 /*------------------------------------------------------------*/
 /*below are functions for multi-connections*/
  /*
   * static void initConnection(uint8_t index)
   * static void initAllConnections(void)
   * static uint8_t returnCurrentActiveConnectionIndex(void)
   * static void setCurrentActiveConnectionIndex(uint8_t index)
   * static uint8_t findAvailableConnectionIndex(void)
   * static uint8_t IsConnectionsFull(void)
   * static uint8_t findExistConnectionByInfo(uint8_t deviceID_databaseIndex)
   * static void updateWhenProfileConnected(uint8_t profile_status, uint8_t deviceID_databaseIndex)
   * static void updateWhenProfileDisconnected(uint8_t profile_status, uint8_t deviceID_databaseIndex)
   */
  /*------------------------------------------------------------*/
 static void initConnection(uint8_t index)
{
    BT_AllConnections.allConnection[index].DeviceID_LinkIndex = 0;
    BT_AllConnections.allConnection[index].LinkedProfileStatus = 0;
}

/*------------------------------------------------------------*/
static void initAllConnections(void)
{
    uint8_t i;
    BT_AllConnections.activeIndex = 0;
    for(i=0; i<CONNECTION_MAX; i++)
        initConnection(i);
}

 /*------------------------------------------------------------*/
static uint8_t returnCurrentActiveConnectionIndex(void)
{
    return BT_AllConnections.activeIndex;
}

 /*------------------------------------------------------------*/
static void setCurrentActiveConnectionIndex(uint8_t index)
{
    BT_AllConnections.activeIndex = index;
}

 /*------------------------------------------------------------*/
static uint8_t findAvailableConnectionIndex(void)
{
    uint8_t i;
    for(i=0; i<CONNECTION_MAX; i++)
    {
        if(BT_AllConnections.allConnection[i].LinkedProfileStatus == 0)
            return i;
    }
    return 0xff;              //-1: not available
}

 /*------------------------------------------------------------*/
static uint8_t IsConnectionsFull(void)
{
    if(findAvailableConnectionIndex() == 0xff)
        return 1;                       //1: full, not available
    else
        return 0;                       //0: still available
}

 /*------------------------------------------------------------*/
static uint8_t findExistConnectionByInfo(uint8_t deviceID_databaseIndex)
{
    uint8_t i;
    for(i=0; i<CONNECTION_MAX; i++)
    {
        if(BT_AllConnections.allConnection[i].DeviceID_LinkIndex == deviceID_databaseIndex)
            return i;
    }
    return 0xff;            //-1: not found
}

 /*------------------------------------------------------------*/
static void updateWhenProfileConnected(uint8_t profile_status, uint8_t deviceID_databaseIndex)
{
    uint8_t index = findExistConnectionByInfo(deviceID_databaseIndex);
    if(index != 0xff)
    {
        BT_AllConnections.allConnection[index].LinkedProfileStatus |= profile_status;
    }
    else
    {
        index = findAvailableConnectionIndex();
        if(index != 0xff)
        {
            //new added a connection
            BT_AllConnections.allConnection[index].DeviceID_LinkIndex = deviceID_databaseIndex;     //save new info(device_id and linked database index)
            BT_AllConnections.allConnection[index].LinkedProfileStatus = 0;
            BT_AllConnections.allConnection[index].LinkedProfileStatus |= profile_status;           //save profile linked status
            setCurrentActiveConnectionIndex(index);
        }
        else
        {
            //ignore
        }
    }
}

/*------------------------------------------------------------*/
static void updateWhenProfileDisconnected(uint8_t profile_status, uint8_t deviceID_databaseIndex)
{
    uint8_t index = findExistConnectionByInfo(deviceID_databaseIndex);
    if(index != 0xff)
    {
        BT_AllConnections.allConnection[index].LinkedProfileStatus &= (profile_status^0xff);
        if(BT_AllConnections.allConnection[index].LinkedProfileStatus == 0)
        {
            //erase a connection
            BT_AllConnections.allConnection[index].DeviceID_LinkIndex = 0;
            if(index == returnCurrentActiveConnectionIndex())
                setCurrentActiveConnectionIndex(0xff);          //set active index to NONE (-1)
        }
    }
    else
    {
        //ignore
    }
}

/*------------------------------------------------------------*/
/*below functions are for volume controlling*/
/*
 * static uint8_t volumeFormatTo4bits(uint8_t volume)
 * static uint8_t volumeFormatTo7bits(uint8_t volume)
 * static uint8_t volumeFormatFrom4bits(uint8_t vol_4bits)
 * static uint8_t volumeFormatFrom7bits(uint8_t vol_7bits)
 * static void volumeUp(VOLUME_MODE mode)
 * static void volumeDown(VOLUME_MODE mode)
 * static uint8_t get4bitsVol(VOLUME_MODE mode)
 * static uint8_t get7bitsVol(VOLUME_MODE mode)
 * static void set4bitVol(uint8_t vol, VOLUME_MODE mode)
 * static void set7bitVol(uint8_t vol, VOLUME_MODE mode)
 * static void volumeUpCurrentMode( void )
 * static void volumeDownCurrentMode( void )
 * static uint8_t get4bitsVolCurrentMode( void )
 * static uint8_t get7bitsVolCurrentMode( void )
 * static void set4bitVolCurrentMode(uint8_t vol)
 * static void set7bitVolCurrentMode(uint8_t vol)
 * static void DSP_setVol(VOLUME_MODE mode)
 * static void DSP_setVolCurrentMode( void )
 */
 /*------------------------------------------------------------*/
static uint8_t volumeFormatTo4bits(uint8_t volume)
{
    return volume>>1;
}

/*------------------------------------------------------------*/
static uint8_t volumeFormatTo7bits(uint8_t volume)
{
    return volume*127/30;
}

/*------------------------------------------------------------*/
static uint8_t volumeFormatFrom4bits(uint8_t vol_4bits)
{
    vol_4bits &= 0x0f;
    return (uint8_t)(vol_4bits<<1);
}

/*------------------------------------------------------------*/
static uint8_t volumeFormatFrom7bits(uint8_t vol_7bits)
{
    vol_7bits &= 0x7f;
    return (uint8_t)(vol_7bits*30/127);
}

/*------------------------------------------------------------*/
static void volumeUp(VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    if(*p < 30)
    {
        (*p)++;
    }
}

/*------------------------------------------------------------*/
static void volumeDown(VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    if(*p > 0)
    {
        (*p)--;
    }
}

/*------------------------------------------------------------*/
static uint8_t get4bitsVol(VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    return volumeFormatTo4bits(*p);
}

/*------------------------------------------------------------*/
static uint8_t get7bitsVol(VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    return volumeFormatTo7bits(*p);
}

/*------------------------------------------------------------*/
static void set4bitVol(uint8_t vol, VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    vol &= 0x0f;
    *p = vol<<1;
}

/*------------------------------------------------------------*/
static void set7bitVol(uint8_t vol, VOLUME_MODE mode)
{
    uint8_t* p;
    switch(mode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    vol &= 0x7f;
    *p = vol*30/127;
}

/*------------------------------------------------------------*/
static void volumeUpCurrentMode( void )
{
    uint8_t* p;
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    if(*p < 30)
    {
        (*p)++;
    }
}

/*------------------------------------------------------------*/
static void volumeDownCurrentMode( void )
{
    uint8_t* p;
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    if(*p > 0)
    {
        (*p)--;
    }
}

/*------------------------------------------------------------*/
static uint8_t get4bitsVolCurrentMode( void )
{
    uint8_t* p;
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    return volumeFormatTo4bits(*p);
}

/*------------------------------------------------------------*/
static uint8_t get7bitsVolCurrentMode( void )
{
    uint8_t* p;
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    return volumeFormatTo7bits(*p);
}

/*------------------------------------------------------------*/
static void set4bitVolCurrentMode(uint8_t vol)
{
    uint8_t* p;
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    vol &= 0x0f;
    *p = vol<<1;
}

/*------------------------------------------------------------*/
static void set7bitVolCurrentMode(uint8_t vol)
{
    uint8_t* p;
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    vol &= 0x7f;
    *p = vol*30/127;
}

/*------------------------------------------------------------*/
static void DSP_setVol(VOLUME_MODE mode)
{
     uint8_t* p;
    switch(mode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
    //yda174_SetVol(*p);
 }

/*------------------------------------------------------------*/
static void DSP_setVolCurrentMode( void )
{
     uint8_t* p;
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            p = &BTAPP_Volume.a2dpVol;
            break;
        case HFP:
            p = &BTAPP_Volume.hfpVol;
            break;
        case LINE_IN:
            p = &BTAPP_Volume.lineInVol;
            break;
    }
   // yda174_SetVol(*p); //lsh_test
 }

/*------------------------------------------------------------*/
/*
 *  static void initStatus(void)
 */
/*------------------------------------------------------------*/
static void initStatus(void)
{
    BT_LinkStatus = 0;
    BT_SystemStatus = BT_SYSTEM_POWER_OFF;
    BT_eCSBStatus.nspk_link = BT_NSPK_NO_LINK;
    BT_eCSBStatus.snpk_event = BT_CSB_EVENT_STANDBY;
    BT_eCSBStatus.nspk_status = BT_CSB_STATUS_STANDBY;
    BT_LinkbackStatus = BT_LINKBACK_INIT;
    BT_linkIndex = 0;
    initAllConnections();
    BT_CallStatus = BT_CALL_IDLE;
    LineInStatus = LINE_IN_INACTIVE;
    BT_A2DPStatus = BT_A2DP_IDLE;
	ble_connect_state = BLE_CONN_STANDBY;
}

/*------------------------------------------------------------*/
/*
 * static void initSPP(void)
 */
/*------------------------------------------------------------*/
static void initSPP(void)
{
    BT_SPP_RxFifoHead = 0;
    BT_SPP_RxFifoTail = 0;
    BT_SPP_RxCounter = 0;
}

/*------------------------------------------------------------*/
/*
 * static void nextCommandReqCheck(void)
 */
/*------------------------------------------------------------*/
static void nextCommandReqCheck(void)
{
    if(NextCommandReq.value)
    {
        if (NextCommandReq.updateA2DPGainReq) {     //from event, button
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.updateA2DPGainReq = 0;
                BT_updateA2DPGain(volumeFormatTo7bits(BTAPP_Volume.a2dpVol), CMD_INFO_MCU);
            }
        }
        if (NextCommandReq.updateHFPGainReq) {      //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.updateHFPGainReq = 0;
                BT_updateHFPGain(volumeFormatTo4bits(BTAPP_Volume.hfpVol), CMD_INFO_MCU);
            }
        }
        if (NextCommandReq.updateLineInGainReq) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.updateLineInGainReq = 0;
                BT_updateLineInGain(volumeFormatTo4bits(BTAPP_Volume.lineInVol), CMD_INFO_MCU);
            }
        }
        if (NextCommandReq.linkbackToDevAddr) {     //from event
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.linkbackToDevAddr = 0;
                BT_LinkBackToDeviceByBTAddress(linkbackBDAddr);
            }
        }
        if (NextCommandReq.musicCtrlReq_00) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.musicCtrlReq_00 = 0;
                BT_MusicControlCommand(0x00);
            }
        }
        if (NextCommandReq.musicCtrlReq_01) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.musicCtrlReq_01 = 0;
                BT_MusicControlCommand(0x01);
            }
        }
        if (NextCommandReq.musicCtrlReq_03) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.musicCtrlReq_03 = 0;
                BT_MusicControlCommand(0x03);
            }
        }
        if (NextCommandReq.musicCtrlReq_07) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.musicCtrlReq_07 = 0;
                BT_MusicControlCommand(0x07);
            }
        }
        if (NextCommandReq.SPPLinkBackReq) {        //fro event
            if (BT_IsCommandSendTaskIdle()) {
                NextCommandReq.SPPLinkBackReq = 0;
                    BT_ProfileLinkBack ( 0x02, BT_linkIndex);
                }
        }
    }
    if(NextMMIActionReq.value)
    {
        if (NextMMIActionReq.BroadcastModeCommandReq) {     //from sequential command
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.BroadcastModeCommandReq = 0;
                BT_MMI_ActionCommand(NSPK_ADD_SPEAKER, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.SpeakerAddCommandReq) {    //from sequential command
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.SpeakerAddCommandReq = 0;
                BT_MMI_ActionCommand(NSPK_ADD_SPEAKER, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.PairReq) {     //from button, event
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.PairReq = 0;
                BT_MMI_ActionCommand(ANY_MODE_ENTERING_PAIRING, BT_linkIndex);
                BT_LinkbackStatus = BT_PAIRING_START;
            }
        }
        if (NextMMIActionReq.ResetEEPROMReq) {  //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.ResetEEPROMReq = 0;
                BT_MMI_ActionCommand(RESET_EEPROM_SETTING, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.NextSongReq) {     //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.NextSongReq = 0;
                BT_MMI_ActionCommand(NEXT_SONG, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.PreviousSongReq) {     //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.PreviousSongReq = 0;
                BT_MMI_ActionCommand(PREVIOUS_SONG, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.IncSpkGainReq) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.IncSpkGainReq = 0;
                BT_MMI_ActionCommand(INC_SPK_GAIN, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.DecSpkGainReq ) {      //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.DecSpkGainReq  = 0;
                BT_MMI_ActionCommand(DEC_SPK_GAIN, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.AcceptCallReq) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.AcceptCallReq = 0;
                BT_MMI_ActionCommand(ACCEPT_CALL, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.ForceEndCallReq) {     //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.ForceEndCallReq = 0;
                BT_MMI_ActionCommand(FORCE_END_CALL, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.LastNumberRedialReq) {     //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.LastNumberRedialReq = 0;
                BT_MMI_ActionCommand(LAST_NUMBER_REDIAL, BT_linkIndex);
            }
        }
        if (NextMMIActionReq.RejectCallReq) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq.RejectCallReq = 0;
                BT_MMI_ActionCommand(REJECT_CALL, BT_linkIndex);
            }
        }
    }
    if(NextMMIActionReq2.value)
    {
        if (NextMMIActionReq2.TransferCallReq) {        //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.TransferCallReq = 0;
                BT_MMI_ActionCommand(0x0E, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.VoiceDialReq) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.VoiceDialReq = 0;
                BT_MMI_ActionCommand(0x0A, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.cancelVoiceDialReq ) {        //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.cancelVoiceDialReq  = 0;
                BT_MMI_ActionCommand(0x0B, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.PlayPauseReq) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.PlayPauseReq = 0;
                BT_MMI_ActionCommand(0x32, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.CancelNSPKReq) {      //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.CancelNSPKReq = 0;
                BT_MMI_ActionCommand(0xE3, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.TerminateNSPKReq) {       //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.TerminateNSPKReq = 0;
                BT_MMI_ActionCommand(0xE4, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.TerminateCancelNSPKReq) {     //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.TerminateCancelNSPKReq = 0;
                BT_MMI_ActionCommand(0xE5, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.switchNSPKChannel) {      //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.switchNSPKChannel = 0;
                BT_MMI_ActionCommand(0xEC, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.enterNSPKModeReq) {       //...
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.enterNSPKModeReq = 0;
                BT_MMI_ActionCommand(0xF4, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.enterBroadcastModeReq) {      //...
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.enterBroadcastModeReq = 0;
                BT_MMI_ActionCommand(0xF5, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.MMI_F6_Req) {     //from event, button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.MMI_F6_Req = 0;
                BT_MMI_ActionCommand(0xF6, BT_linkIndex);
            }
        }
        if (NextMMIActionReq2.MMI_F7_Req) {     //from button
            if (BT_IsCommandSendTaskIdle()) {
                NextMMIActionReq2.MMI_F7_Req = 0;
                BT_MMI_ActionCommand(0xF7, BT_linkIndex);
            }
        }
    }
    
    //unused ...
    /*if (TimeCommandReq.ReqDisableDiscoverableTimer == 0 && TimeCommandReq.ReqDisableDiscoverable) {
        if (BT_IsAllowedToSendCommand()) {
            TimeCommandReq.ReqDisableDiscoverable = 0;
            BT_SendDiscoverableCommand(0); //disable discoverable when time is out.
        }
    }*/
}
//================================================
// Init
//================================================
void BTAPP_Init( void )
{
    BTAPP_TaskState = BT_STATE_INITIALIZE_START;
    initStatus();
    BT_CommandDecodeInit();
    BT_CommandSendInit();
    BLE_advTaskInit();
    initSPP();
    BTAPP_Volume.a2dpVol = 20;
    BTAPP_Volume.hfpVol = 20;
    BTAPP_Volume.lineInVol = 20;
    BTAPP_Volume.currentVolMode = A2DP;
    BT_Port1.PORT_1 = 0;
    BT_Port3.PORT_3 = 0;
    NextCommandReq.value = 0;
    NextMMIActionReq.value = 0;
    NextMMIActionReq2.value = 0;
    /*TimeCommandReq.allReq = 0;*/
	BTEQ_timer1ms = 0;
	BTEQ_SendIndex = BTEQ_MAX_INDEX+1;	// assum it finish the packet sending
//	BTEQ_SendIndex = BTEQ_MAX_INDEX;	// assum it finish the packet sending

	BTEQ_RecIndex = 0;		// expect to see first packet
	BTEQ_Debug = 0;
}

//================================================
// Task
//================================================
void BTAPP_Task(void) {
}

//================================================
//BT status checking
//================================================
uint8_t BTAPP_GetStatus(void)
{
    if(BTAPP_TaskState == BT_STATE_POWER_OFF)
        return BT_STATUS_OFF;
    else if(BTAPP_TaskState >= BT_STATE_POWER_ON && BTAPP_TaskState < BT_STATE_BT_RUNNING)
        return BT_STATUS_ON;
    else if(BTAPP_TaskState == BT_STATE_BT_RUNNING)
        return BT_STATUS_READY;
    else
        return BT_STATUS_NONE;
}

//================================================
// BT Event Handler
//================================================
void BTAPP_EventHandler(uint8_t event, uint16_t para, uint8_t* para_full)
{
    uint8_t lowByte, highByte;
	uint8_t i=0;

	if(event == NSPK_EVENT_VENDOR_CMD)
	{
		BTEQ_Debug++;
		if(para_full[0x03] & 0x80)
		{
			// Vendor EQ commands
			para_full[0x03] -= 0x80;
			if(para_full[0x03] == 0x00)
			{
				for(i=0; i<DSP_EQ_SIZE; i++)
					dspEQ[i] = 0;
	
				dspEQ[0] = 0xAA;	 //header byte 0
				dspEQ[1] = 0x00;	 //header byte 1
				dspEQ[2] = 84+2;	 //length
				dspEQ[3] = 0x30;		//command ID
				dspEQ[4] = 0x13;	   //link_index, set to 0
	
				BTEQ_RecIndex = 0;
			}
			else if(para_full[0x03] >= BTEQ_MAX_INDEX)
			{
				// Seq out range, do nothing.
				//break;
			}
			
			if(para_full[0x03] == BTEQ_RecIndex)
			{
				if(para_full[2] == 9)
				{
					for(i=0; i<8;i++)
						dspEQ[5 + (BTEQ_RecIndex * (VENDOR_CMD_SIZE-1)) + i] = para_full[4 +i]; 
				}
				else 
				{
					for(i=0; i<4;i++)
						dspEQ[5 + (BTEQ_RecIndex * (VENDOR_CMD_SIZE-1)) + i] = para_full[4 +i]; 
				}	
				BTEQ_RecIndex++;
				if(BTEQ_RecIndex == BTEQ_MAX_INDEX)
				{
					BTEQ_RecIndex = 0;
					// the EQ data completed
					BT_Send_EQCmd(dspEQ, 90);				
				}			
			}
			else
			{
				// Get the wrong seq-packet.
				// Ignore all the packet until see the first packet.
				
			}
			return;
		}
		else if(event == APPS_SET_INDIV_MMI_ACTION)
		{
			// should not enter here
			return;	
		}
		else
		{
			event = para_full[0x03];
			para = para_full[0x05];
			// Allow the below "switch" case to handle
		}
	}

	
    switch(event)
    {
        case BT_EVENT_CMD_SENT_ACK_OK:
            break;
            
        case BT_EVENT_CMD_SENT_ACK_ERROR:
            BT_GiveUpThisCommand();
            break;
            
        case BT_EVENT_CMD_SENT_NO_ACK:
            if(para == 0x04 || para == 0x02)        //MMI_ACTION or MUSIC_CTRL
                BT_GiveUpThisCommand();
            break;
            
		case NSPK_EVENT_EQ_MODE:
			if(BT_eCSBStatus.nspk_link == BT_BROADCAST_SLAVE || 
			   BT_eCSBStatus.nspk_link == BT_NSPK_SLAVE_LINK_TO_MASTER)
			{
				BT_Send_EQMode(para);
			}
			break;
		case NSPK_EVENT_VOL_MODE:
			if(BT_eCSBStatus.nspk_link == BT_BROADCAST_SLAVE)
			{
				if(para == ALL_MUTE)
				{
					//yda174_ForceMute(1);
				}
				else if(para == ALL_UNMUTE)
				{
					//yda174_ForceMute(0);
				}	
				else				
				{
//			BTAPP_Volume.a2dpVol = para;		// volume not change by Auto by jaehoon
//			DSP_setVolCurrentMode();	// volume not change by Auto by jaehoon
			        if (BT_IsCommandSendTaskIdle()) {
//	            BT_updateA2DPGain(volumeFormatTo7bits(BTAPP_Volume.a2dpVol), CMD_INFO_MCU);	// volume not change by Auto by jaehoon
			        }
			        else {
			            NextCommandReq.updateA2DPGainReq = 1;
			        }
				}
			}
		break;
        case BT_EVENT_NSPK_STATUS:
            BT_eCSBStatus.nspk_link = (uint8_t)(para>>8);
			i =0;
			if(BT_eCSBStatus.nspk_link == 7)
			{
				BT_eCSBStatus.nspk_link = BT_BROADCAST_MASTER;
				i = 1;
			}
            BT_eCSBStatus.snpk_event = (uint8_t)(para&0x00ff);
			if(BTAPP_TaskState == BT_STATE_INIT_NSPK_STATUS_WAIT)
			{
				BTAPP_TaskState = BT_STATE_READ_PAIR_RECORD;
				break;
			}	
			BLE_advUpdateLinkStatus(BT_eCSBStatus.snpk_event);
				
            if(BT_eCSBStatus.nspk_link)     //NSPK exist
            {
                switch(BT_eCSBStatus.nspk_link)                     {
                    case BT_NSPK_MASTER_LINK_TO_SLAVE2:// = 1,
                    case BT_NSPK_MASTER_LINK_TO_SLAVE3:// = 2,
                    case BT_NSPK_MASTER_LINK_TO_BOTH:// = 3,
                        if (BT_eCSBStatus.snpk_event == BT_CSB_EVENT_CONNECTED) {
                            BT_eCSBStatus.nspk_status = BT_CSB_STATUS_CONNECTED_AS_NSPK_MASTER;

                            BLE_advUpdateGroupStatus(SPEAKER_nSPK_MASTER);
							BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
							BLE_advUpdateBTconnectable(BT_CONNECTABLE);  //enable BLE and it is connectable

                        }
                        else if(BT_eCSBStatus.snpk_event == BT_CSB_EVENT_CONTINUE_CONNECTING) {
                            BT_eCSBStatus.nspk_status = BT_CSB_STATUS_NSPK_MASTER_CONNECTING;
                        }

                        break;

                    case BT_NSPK_SLAVE_LINK_TO_MASTER:// = 4,
                        if (BT_eCSBStatus.snpk_event == BT_CSB_EVENT_CONNECTED) {
                            BT_eCSBStatus.nspk_status = BT_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE;

							//printf("The number of connected slave speaker = %d\r\n",  para_full[2]); //The number of connected slave speaker
                            

                            BT_masterBDAddr[0] = para_full[3];
                            BT_masterBDAddr[1] = para_full[4];
                            BT_masterBDAddr[2] = para_full[5];
                            BT_masterBDAddr[3] = para_full[6];
                            BT_masterBDAddr[4] = para_full[7];
                            BT_masterBDAddr[5] = para_full[8];
							BLE_SaveMasterAddr(BT_masterBDAddr);
                            BLE_advUpdateGroupStatus(SPEAKER_nSPK_SLAVE);
							BTMute_timer1ms = 1000;
							//yda174_ForceMute(1);							
                        }

                        BLE_UpdateAdvType(SCANNABLE_UNDIRECT_ADV); //non-connectable
                        BLE_advUpdateBTconnectable(BT_NON_CONNECTABLE);
                        break;

                    case BT_BROADCAST_MASTER:// = 5,
                        if (BT_eCSBStatus.snpk_event == BT_CSB_EVENT_CONNECTED) {
                            BT_eCSBStatus.nspk_status = BT_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER;
                            BLE_advUpdateGroupStatus(i ? SPEAKER_BROADCAST_MASTER : SPEAKER_BROADCAST_MASTER_ADDING);
							BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
							BLE_advUpdateBTconnectable(BT_CONNECTABLE);  //enable BLE and it is connectable

							// Synchonize the current volume to the slave
							BT_SendVolInfo();
                        }
                        else if(BT_eCSBStatus.snpk_event == BT_CSB_EVENT_CONTINUE_CONNECTING) {
                            BT_eCSBStatus.nspk_status = BT_CSB_STATUS_BROADCAST_MASTER_CONNECTING;
                        }
                        break;

                    case BT_BROADCAST_SLAVE:// = 6
                        if (BT_eCSBStatus.snpk_event == BT_CSB_EVENT_CONNECTED) {
                            BT_eCSBStatus.nspk_status = BT_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE;
                            BT_masterBDAddr[0] = para_full[3];
                            BT_masterBDAddr[1] = para_full[4];
                            BT_masterBDAddr[2] = para_full[5];
                            BT_masterBDAddr[3] = para_full[6];
                            BT_masterBDAddr[4] = para_full[7];
                            BT_masterBDAddr[5] = para_full[8];
							BLE_SaveMasterAddr(BT_masterBDAddr);
                            BLE_advUpdateGroupStatus(SPEAKER_BROADCAST_SLAVE);	
							BTMute_timer1ms = 1000;
							//yda174_ForceMute(1);
                        }

				NSPK_Voice_timer_count = 0;


                        BLE_UpdateAdvType(SCANNABLE_UNDIRECT_ADV); //non-connectable
                        BLE_advUpdateBTconnectable(BT_NON_CONNECTABLE);

                        break;
                }
            }
            else
            {
                if(BT_eCSBStatus.snpk_event == BT_CSB_EVENT_STANDBY || 
				   BT_eCSBStatus.snpk_event == BT_CSB_EVENT_LOSS
				   ){
                    BT_eCSBStatus.nspk_status = BT_CSB_STATUS_STANDBY;
				    BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
				    BLE_advUpdateBTconnectable(BT_CONNECTABLE);	//enable BLE and it is connectable
				    BLE_advUpdateGroupStatus(SPEAKER_SINGLE);					
                }
                else if(BT_eCSBStatus.snpk_event == BT_CSB_EVENT_CONNECTING){
                    BT_eCSBStatus.nspk_status = BT_CSB_STATUS_CONNECTING;
                }

            }
            break;

        case NSPK_EVENT_CHANNEL_SETTING:
            //BLE_advUpdateOutputChannel((uint8_t)(para&0x00ff));
            break;

        case BT_EVENT_LINE_IN_STATUS:
            LineInStatus = (uint8_t)(para&0x00ff);
            if(LineInStatus != LINE_IN_INACTIVE)
            {
                BTAPP_Volume.currentVolMode = LINE_IN;
				isAudioSelectUnmute();
                DSP_setVolCurrentMode();
            }
            break;

        case BT_EVENT_A2DP_STATUS:
            BT_A2DPStatus = (uint8_t)(para&0x00ff);
            if(BT_A2DPStatus == BT_A2DP_ACTIVE)
            {
                BTAPP_Volume.currentVolMode = A2DP;
				isAudioSelectUnmute();													
                DSP_setVolCurrentMode();
            }
            break;

        case BT_EVENT_CALL_STATUS_CHANGED:
            BT_CallStatus = (uint8_t)para;
            if(BT_CallStatus == BT_CALL_IDLE)
            {
                BTAPP_Volume.currentVolMode = A2DP;
                DSP_setVolCurrentMode();
            }
            break;

        case BT_EVENT_HFP_CONNECTED:
            BT_LinkStatus |= BIT_HFP_LINK_STATUS;
            updateWhenProfileConnected(BIT_HFP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            BT_LinkbackStatus = BT_LINK_CONNECTED;
            BT_SystemStatus = BT_SYSTEM_CONNECTED;
            break;
        case BT_EVENT_HFP_DISCONNECTED:
            BT_LinkStatus &= (BIT_HFP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(BIT_HFP_LINK_STATUS, (uint8_t)para);
            break;
        case BT_EVENT_SPP_CONNECTED:
            BT_LinkStatus |= BIT_SPP_LINK_STATUS;//BT_SPP_Connected = true;
            updateWhenProfileConnected(BIT_SPP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case BT_EVENT_IAP_CONNETED:
            BT_LinkStatus |= BIT_IAP_LINK_STATUS;
            updateWhenProfileConnected(BIT_IAP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case BT_EVENT_SPP_IAP_DISCONNECTED:
            BT_LinkStatus &= (BIT_SPP_LINK_STATUS^0xff);//BT_SPP_Connected = false;
            BT_LinkStatus &= (BIT_IAP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(BIT_SPP_LINK_STATUS, (uint8_t)para);
            updateWhenProfileDisconnected(BIT_IAP_LINK_STATUS, (uint8_t)para);
            break;
        case BT_EVENT_A2DP_CONNECTED:
            BT_LinkStatus |= BIT_A2DP_LINK_STATUS;
            if(!(BT_LinkStatus & BIT_SPP_LINK_STATUS || BT_LinkStatus & BIT_IAP_LINK_STATUS))
            {
#if 0
                if (BT_IsAllowedToSendCommand()) {
                    BT_ProfileLinkBack ( 0x02, BT_linkIndex);
                }
                else {
                    NextCommandReq.SPPLinkBackReq = 1;
                }
#endif
            }
            updateWhenProfileConnected(BIT_A2DP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            BT_LinkbackStatus = BT_LINK_CONNECTED;
            BT_SystemStatus = BT_SYSTEM_CONNECTED;
            break;
        case BT_EVENT_A2DP_DISCONNECTED:
            BT_LinkStatus &= (BIT_A2DP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(BIT_A2DP_LINK_STATUS, (uint8_t)para);
            break;
        case BT_EVENT_AVRCP_CONNECTED:
            BT_LinkStatus |= BIT_AVRCP_LINK_STATUS;
            updateWhenProfileConnected(BIT_AVRCP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case BT_EVENT_AVRCP_DISCONNECTED:
            BT_LinkStatus &= (BIT_AVRCP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(BIT_AVRCP_LINK_STATUS, (uint8_t)para);
            break;
        case BT_EVENT_ACL_CONNECTED:
            BT_LinkStatus |= BIT_ACL_LINK_STATUS;
            updateWhenProfileConnected(BIT_ACL_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case BT_EVENT_ACL_DISCONNECTED:
            BT_LinkStatus &= (BIT_ACL_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(BIT_ACL_LINK_STATUS, (uint8_t)para);
            break;
        case BT_EVENT_SCO_CONNECTED:
            BT_LinkStatus |= BIT_SCO_LINK_STATUS;
            updateWhenProfileConnected(BIT_SCO_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case BT_EVENT_SCO_DISCONNECTED:
            BT_LinkStatus &= (BIT_SCO_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(BIT_SCO_LINK_STATUS, (uint8_t)para);
            break;
        case BT_EVENT_MAP_CONNECTED:
            BT_LinkStatus |= BIT_MAP_LINK_STATUS;
            updateWhenProfileConnected(BIT_MAP_LINK_STATUS, (uint8_t)para);
            //if(IsConnectionsFull())
            //    BT_SendDiscoverableCommand(0);
            break;
        case BT_EVENT_MAP_DISCONNECTED:
            BT_LinkStatus &= (BIT_MAP_LINK_STATUS^0xff);
            updateWhenProfileDisconnected(BIT_MAP_LINK_STATUS, (uint8_t)para);
            break;

        case BT_EVENT_SYS_POWER_ON:
            BT_SystemStatus = BT_SYSTEM_POWER_ON;
            break;
        case BT_EVENT_SYS_POWER_OFF:
            BT_SystemStatus = BT_SYSTEM_POWER_OFF;
            break;
        case BT_EVENT_SYS_STANDBY:
            BT_SystemStatus = BT_SYSTEM_STANDBY;
            break;

        case BT_EVENT_SYS_PAIRING_START:
            BT_SystemStatus = BT_SYSTEM_PAIRING;
            BT_LinkbackStatus = BT_PAIRING;
            break;
        case BT_EVENT_SYS_PAIRING_OK:
            BT_LinkbackStatus = BT_PAIRING_OK;
            break;
        case BT_EVENT_SYS_PAIRING_FAILED:
            BT_LinkbackStatus = BT_PAIRING_FAILED;
            break;

        case BT_EVENT_LINKBACK_SUCCESS:
            BT_LinkbackStatus = BT_LINKBACK_OK;
            break;

        case BT_EVENT_LINKBACK_FAILED:
            BT_LinkbackStatus = BT_LINKBACK_FAILED;
            break;

        case BT_EVENT_BD_ADDR_RECEIVED:
            if(BTAPP_TaskState == BT_STATE_INIT_READ_DEVICE_ADDR_WAIT)
            {
                BTAPP_timer1ms = 0;
			  	BTAPP_TaskState = BT_STATE_INIT_READ_DEVICE_NAME;
            }
            break; 
	   case BT_EVENT_NAME_RECEIVED:
			if(BTAPP_TaskState == BT_STATE_INIT_READ_DEVICE_NAME_WAIT)
			{
				BTAPP_timer1ms = 0;
				BTAPP_TaskState = BT_STATE_INIT_READ_5506_FEATURE;
			}
			break;
			case FEATURE_LIST_REPLY:
		    if(BTAPP_TaskState == BT_STATE_INIT_READ_5506_FEATURE_WAIT)
			{
				 BTAPP_timer1ms = 0;
                BTAPP_TaskState = BT_STATE_INIT_BLE_ADV_START;
            }
            break;
        case BT_EVENT_PAIR_RECORD_RECEIVED:
            BT_pairedRecordNumber = (uint8_t)para;
            if(BTAPP_TaskState == BT_STATE_READ_PAIR_RECORD_WAIT)
            {
                    //BTAPP_timer1ms = 100;
                    BTAPP_TaskState = BT_STATE_READ_LINKED_MODE;
            }
            break;

        case BT_EVENT_LINK_MODE_RECEIVED:
            BT_linkedMode = (uint8_t)para;
            if(BT_linkedMode == 2 || BT_linkedMode == 4)        //NSPK master, Broadcast master
            {
                para >>= 8;
                BT_nSPKLinkedCounter = (uint8_t)para;
            }
            if(BTAPP_TaskState == BT_STATE_READ_LINKED_MODE_WAIT)
            {
                BTAPP_timer1ms = 0; //clear time out timer
                BTAPP_TaskState = BT_STATE_LINKBACK_START;
            }
            break;

        case BT_EVENT_PLAYBACK_STATUS_CHANGED:
            BT_PlayingStatus = (uint8_t)para;
            break;

        case NSPK_EVENT_SYNC_POWER_OFF:
            //BTAPP_timer1ms = 0;
            BTAPP_TaskState = BT_STATE_POWER_OFF_START_NSPK;
            break;

        case BT_EVENT_HFP_VOLUME_CHANGED:
            set4bitVol((uint8_t)para, HFP);
            DSP_setVolCurrentMode();
            break;

        case BT_EVENT_AVRCP_VOLUME_CTRL:        //AVRCP 1.0
            if(para == 0)       //vol up
            {
                volumeUp(A2DP);
            }
            else if(para == 1)      //vol down
            {
                volumeDown(A2DP);
            }
            DSP_setVolCurrentMode();
            break;

        case BT_EVENT_AVRCP_ABS_VOLUME_CHANGED: //AVRCP > 1.0
			if((BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_NSPK_MASTER) ||			   
  			   (BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER)||
			   (BT_eCSBStatus.nspk_status == BT_CSB_STATUS_NSPK_MASTER_CONNECTING) ||
			   (BT_eCSBStatus.nspk_status == BT_CSB_STATUS_BROADCAST_MASTER_CONNECTING)
			)
			BT_Vendor_SendVol(event, para);
        
            set7bitVol((uint8_t)para, A2DP);
            DSP_setVolCurrentMode();

            break;

        case NSPK_EVENT_SYNC_VOL_CTRL:
			if((BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_NSPK_MASTER) ||			   
  			   (BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER)||
			   (BT_eCSBStatus.nspk_status == BT_CSB_STATUS_NSPK_MASTER_CONNECTING) ||
			   (BT_eCSBStatus.nspk_status == BT_CSB_STATUS_BROADCAST_MASTER_CONNECTING)
			)
			BT_Vendor_SendVol(event, para);
			
            if(para == 0)       //vol up
            {
                if(BTAPP_Volume.currentVolMode == A2DP)
                {
					volumeUp(A2DP);//volumeUpCurrentMode();
                    DSP_setVolCurrentMode();
                    //BT_SetOverallGainCommand(4, volumeFormatTo7bits(BTAPP_Volume.a2dpVol), volumeFormatTo7bits(BTAPP_Volume.hfpVol), volumeFormatTo7bits(BTAPP_Volume.lineInVol));
                    if (BT_IsCommandSendTaskIdle()) {
                        BT_updateA2DPGain(volumeFormatTo7bits(BTAPP_Volume.a2dpVol), CMD_INFO_MCU);
                    }
                    else {
                        NextCommandReq.updateA2DPGainReq = 1;
                    }
                }
				else if(BTAPP_Volume.currentVolMode == LINE_IN)
				{
		            volumeUp(LINE_IN);
		            DSP_setVolCurrentMode();
		            if(BTAPP_TaskState >= BT_STATE_POWER_ON)
		            {
		                if (BT_IsCommandSendTaskIdle()) {
		                    BT_updateLineInGain(volumeFormatTo4bits(BTAPP_Volume.lineInVol), CMD_INFO_MCU);
		                }
		                else {
		                    NextCommandReq.updateLineInGainReq = 1;
		                }
		            }				
				}
            }
            else if(para == 1)      //vol down
            {
                if(BTAPP_Volume.currentVolMode == A2DP)
                {
					volumeDown(A2DP);//volumeDownCurrentMode();
                    DSP_setVolCurrentMode();
                    //BT_SetOverallGainCommand(4, volumeFormatTo7bits(BTAPP_Volume.a2dpVol), volumeFormatTo7bits(BTAPP_Volume.hfpVol), volumeFormatTo7bits(BTAPP_Volume.lineInVol));
                    if (BT_IsCommandSendTaskIdle()) {
                        BT_updateA2DPGain(volumeFormatTo7bits(BTAPP_Volume.a2dpVol), CMD_INFO_MCU);
                    }
                    else {
                        NextCommandReq.updateA2DPGainReq = 1;
                    }
                }
				else if(BTAPP_Volume.currentVolMode == LINE_IN)
				{
					volumeDown(LINE_IN);
					DSP_setVol(LINE_IN);
					if(BTAPP_TaskState >= BT_STATE_POWER_ON)
					{
					   if (BT_IsCommandSendTaskIdle()) {
						   BT_updateLineInGain(volumeFormatTo4bits(BTAPP_Volume.lineInVol), CMD_INFO_MCU);
					   }
					   else {
						   NextCommandReq.updateLineInGainReq = 1;
					   }
					}
				}

            }
            break;

        case NSPK_EVENT_SYNC_INTERNAL_GAIN:
            set4bitVol((uint8_t)(para&0x000f), A2DP);               //bit3~0 indicates A2DP gain
            set4bitVol((uint8_t)((para>>4) & 0x000f), LINE_IN);     //bit7~4 indicates LINE IN gain
            break;
            
        case NSPK_EVENT_SYNC_ABS_VOL:
            set7bitVol((uint8_t)para, A2DP);//set7bitVolCurrentMode((uint8_t)para);
            if(BTAPP_Volume.currentVolMode == A2DP)
            {
                DSP_setVolCurrentMode();
                //BT_SetOverallGainCommand(4, volumeFormatTo7bits(BTAPP_Volume.a2dpVol), volumeFormatTo7bits(BTAPP_Volume.hfpVol), volumeFormatTo7bits(BTAPP_Volume.lineInVol));
                if (BT_IsCommandSendTaskIdle()) {
                    BT_updateA2DPGain(volumeFormatTo7bits(BTAPP_Volume.a2dpVol), CMD_INFO_MCU);
                } else {
                    NextCommandReq.updateA2DPGainReq = 1;
                }
            }
            break;

        case NSPK_EVENT_ADD_SPEAKER3:
            if(BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE2     //this is master, it connects to speaker 2 only
            || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE3)    //this is master, it connects to speaker 3 only
            {
                //go to add 3rd speaker in stereo mode
                if (BT_IsCommandSendTaskIdle())
                    BT_MMI_ActionCommand ( 0xF6, BT_linkIndex);
                else
                    NextMMIActionReq2.MMI_F6_Req = 1;
            }
            break;

        case LE_STATUS_CHANGED:
            lowByte = (uint8_t)para;
            para >>= 8;
            highByte = (uint8_t)para;
		    ble_connect_state = highByte;			
            switch(highByte)
            {
                case 0x00:      //LE standby
                    if(BTAPP_TaskState > BT_STATE_POWER_ON && BTAPP_TaskState <= BT_STATE_BT_RUNNING
                            ||BTAPP_TaskState == BT_STATE_POWER_OFF)
                    {
						if(BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE ||
						BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE)
						{
							BLE_UpdateAdvType(SCANNABLE_UNDIRECT_ADV); //non-connectable
							BLE_advUpdateBTconnectable(BT_NON_CONNECTABLE);						}
						else
						{
	                        BLE_UpdateAdvType(CONNECTABLE_UNDIRECT_ADV);
	                        BLE_advUpdateBTconnectable(BT_CONNECTABLE);
                    }
                    }
					BLE_reAdvConnectable();
                    break;
                case 0x01:      //advertising
                    break;
                case 0x02:      //scanning
                    break;
                case 0x03:      //connected
                    if(getDatabase3Jumper())        //check jumper setting to determine if need to power on when BLE is connected
                    {
                        if(BTAPP_GetStatus() == BT_STATUS_OFF)
                        {
                            BTAPP_TaskReq(BT_REQ_SYSTEM_ON);
                        }
                    }
					BT_SendAppsPowerStatus(BTAPP_GetStatus() == BT_STATUS_OFF ? 0 : 2);
                    break;
                default:
                    break;
            }
            break;
        case LE_ADV_CONTROL_REPORT:
            break;
        case LE_CONNECTION_PARA_REPORT:
            break;
        case LE_CONNECTION_PARA_UPDATE_RSP:
            break;
        case GATT_ATTRIBUTE_DATA:
            break;

        case PORT0_INPUT_CHANGED:
            break;
        case PORT1_INPUT_CHANGED:
            lowByte = (uint8_t)para;
            para >>= 8;
            highByte = (uint8_t)para;
            if(highByte & 0x20)                 //P1-5 indicator
            {
                if(lowByte&0x20)
                    BT_Port1.PORT_1_5 = 0;      //save P1-5
                else
                    BT_Port1.PORT_1_5 = 1;      //save P1-5
            }
            break;
        case PORT2_INPUT_CHANGED:
            break;
        case PORT3_INPUT_CHANGED:
            lowByte = (uint8_t)para;
            para >>= 8;
            highByte = (uint8_t)para;
            if(highByte & 0x40)                 //P3-6 indicator
            {
                if(lowByte&0x40)
                    BT_Port3.PORT_3_6 = 0;      //save P3-6
                else
                    BT_Port3.PORT_3_6 = 1;      //save P3-6
            }
            if(BTAPP_TaskState == BT_STATE_INIT_WAIT_GPIO_EVENT)
            {
                BTAPP_TaskState = BT_STATE_INIT_READ_DEVICE_ADDR;
            }
            break;
		case BT_EVENT_TYPE_CODEC:
			// SCO (HF) mode
			if(para_full[1] == 0x06 )
			{
			  BTAPP_Volume.currentVolMode = HFP;
			  DSP_setVolCurrentMode();
			}
			break;
        default:
            break;
    }
}

//================================================
// BT Task Request
//================================================
void BTAPP_TaskReq(uint8_t request)
{
    BTAPP_Request = request;
}

//================================================
//1ms Timer
//================================================
void BTAPP_Timer1MS_event( void )
{
    if(BTAPP_timer1ms)
        --BTAPP_timer1ms;

	if(BTEQ_timer1ms)
		--BTEQ_timer1ms;

	if(BTMute_timer1ms)
	{
		//if(--BTMute_timer1ms==0)  //jhlee 20171111
			--BTMute_timer1ms;
			//yda174_ForceMute(0);
	}
    /*if(TimeCommandReq.ReqDisableDiscoverableTimer)
        --TimeCommandReq.ReqDisableDiscoverableTimer;*/
}


/*-----------------------------------------------------------------------------*/
//EXTERNAL API interface(for UI)
/*-----------------------------------------------------------------------------*/
/*below functions are external API interface, are majorly for buttons operation*/
/*
 * void BTAPP_EnterBTPairingMode( void )
 * void BTAPP_SetBTDiscoverable( void )
 * void BTAPP_ResetEEPROMtoDefault( void )
 * void BTAPP_PlayNextSong( void )
 * void BTAPP_PlayPreviousSong( void )
 * void BTAPP_PlayPause( void )
 * void BTAPP_StartFastForward( void )
 * void BTAPP_StartFastRewind( void )
 * void BTAPP_CancelForwardOrRewind( void )
 * void BTAPP_NSPKAdd( void )
 * void BTAPP_NSPKBtnLongPress( void )
 * void BTAPP_NSPKBtnDbClick( void )
 * void BTAPP_NSPKBtnShortPress( void )
 * void BTAPP_BroadcastAdd( void )
 * void BTAPP_BroadcastBtnLongPress(void)
 * void BTAPP_BroadcastBtnShortPress(void)
 * void BTAPP_BroadcastBtnDbClick(void)
 * void BTAPP_ExitBroadcastRegisterMode( void )
 * void BTAPP_GroupSpeakerSoundSync( void )
 * void BTAPP_CallEventShort( void )
 * void BTAPP_CallEventLong( void )
 * void BTAPP_VolUp( void )
 * void BTAPP_VolDown( void )
 */
/*-----------------------------------------------------------------------------*/
void BTAPP_EnterBTPairingMode( void )
{
    if(BTAPP_TaskState == BT_STATE_BT_RUNNING)
    {
        if (BT_IsCommandSendTaskIdle()) {
            BT_MMI_ActionCommand(ANY_MODE_ENTERING_PAIRING, BT_linkIndex);
            BT_LinkbackStatus = BT_PAIRING_START;
        }
        else {
            NextMMIActionReq.PairReq = 1;
        }
    }
}

/*-----------------------------------------------------------------------------*/
/*void BTAPP_SetBTDiscoverable( void )        //unused...
{
    if(BTAPP_GetStatus() == BT_STATUS_READY)
    {
        if(!IsConnectionsFull())
        {
            BT_SendDiscoverableCommand(1);      //discoverable
            TimeCommandReq.ReqDisableDiscoverableTimer = 60000;       //set to 60 seconds
            TimeCommandReq.ReqDisableDiscoverable = 1;
        }
    }
}*/

/*-----------------------------------------------------------------------------*/
void BTAPP_ResetEEPROMtoDefault( void )
{
    if (BT_IsCommandSendTaskIdle()) {
        BT_MMI_ActionCommand ( RESET_EEPROM_SETTING, BT_linkIndex);
    }
    else {
        NextMMIActionReq.ResetEEPROMReq = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_PlayNextSong( void )
{
    if (BT_IsCommandSendTaskIdle()) {
        BT_MMI_ActionCommand ( NEXT_SONG, BT_linkIndex);
    }
    else {
        NextMMIActionReq.NextSongReq = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_PlayPreviousSong( void )
{
    if (BT_IsCommandSendTaskIdle()) {
        BT_MMI_ActionCommand ( PREVIOUS_SONG, BT_linkIndex);
    }
    else{
        NextMMIActionReq.PreviousSongReq = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_PlayPause( void )
{
    if(BT_eCSBStatus.nspk_link == BT_NSPK_SLAVE_LINK_TO_MASTER || BT_eCSBStatus.nspk_link == BT_BROADCAST_SLAVE)    //slave
    {
        if (BT_IsCommandSendTaskIdle())
            BT_MMI_ActionCommand ( 0x32, BT_linkIndex);
        else
            NextMMIActionReq2.PlayPauseReq = 1;
    }
    else
    {
        if (BT_IsCommandSendTaskIdle())
        {
            BT_MusicControlCommand(0x07);
        }
        else
        {
            NextCommandReq.musicCtrlReq_07 = 1;
        }
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_StartFastForward( void )
{
    if (BT_IsCommandSendTaskIdle())
    {
        BT_MusicControlCommand(0x01);
    }
    else
    {
        NextCommandReq.musicCtrlReq_01 = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_StartFastRewind( void )
{
    if (BT_IsCommandSendTaskIdle())
    {
        BT_MusicControlCommand(0x03);
    }
    else
    {
        NextCommandReq.musicCtrlReq_03 = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_CancelForwardOrRewind( void )
{
    if (BT_IsCommandSendTaskIdle())
    {
        BT_MusicControlCommand(0x00);
    }
    else
    {
        NextCommandReq.musicCtrlReq_00 = 1;
    }
}

/*-----------------------------------------------------------------------------*/
//below are functions for NSPK or broadcast link creating or canceling
/*-----------------------------------------------------------------------------*/
void BTAPP_NSPKAdd( void )
{
    BT_MMI_ActionCommand ( 0xF4, BT_linkIndex);
    NextMMIActionReq.SpeakerAddCommandReq = 1;
}

/*-----------------------------------------------------------------------------*/
void BTAPP_NSPKBtnLongPress( void )
{
    if(BTAPP_TaskState != BT_STATE_BT_RUNNING)
        return;

    switch(BT_eCSBStatus.nspk_status)
    {
        case BT_CSB_STATUS_STANDBY:
            BTAPP_NSPKAdd();
            break;
        case BT_CSB_STATUS_CONNECTING:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE3, BT_linkIndex);     //cancel NSPK creation
            else
                NextMMIActionReq2.CancelNSPKReq = 1;
            break;
        case BT_CSB_STATUS_CONNECTED_AS_NSPK_MASTER:
        case BT_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE5, BT_linkIndex);     //cancel/terminate NSPK link
            else
                NextMMIActionReq2.TerminateCancelNSPKReq = 1;
            break;
        case BT_CSB_STATUS_NSPK_MASTER_CONNECTING:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE3, BT_linkIndex);     //cancel NSPK creation
            else
                NextMMIActionReq2.CancelNSPKReq = 1;
            break;
        case BT_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER:
        case BT_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE4, BT_linkIndex);     //terminate NSPK link
            else
                NextMMIActionReq2.TerminateNSPKReq = 1;
            break;
        case BT_CSB_STATUS_BROADCAST_MASTER_CONNECTING:
            //N/A
            break;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_NSPKBtnDbClick( void )
{
    if(BTAPP_GetStatus() != BT_STATUS_READY)
        return;
    if(BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_NSPK_MASTER ||
    BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE)
    {
        if (BT_IsCommandSendTaskIdle())
            BT_MMI_ActionCommand(0xF6, BT_linkIndex);
        else
            NextMMIActionReq2.MMI_F6_Req = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_NSPKBtnShortPress( void )
{
    if(BTAPP_GetStatus() != BT_STATUS_READY)
        return;

    if(BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE2     //connect to speaker 2 only
            || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE3    //connect to speaker 3 only
            || BT_eCSBStatus.nspk_link == BT_NSPK_SLAVE_LINK_TO_MASTER)      //this is slave
    {
        //go to switch audio channel in stereo mode
        if (BT_IsCommandSendTaskIdle())
            BT_MMI_ActionCommand ( 0xEC, BT_linkIndex);
        else
            NextMMIActionReq2.switchNSPKChannel = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastAdd( void )
{
    BT_MMI_ActionCommand ( 0xF5, BT_linkIndex);
    NextMMIActionReq.BroadcastModeCommandReq = 1;
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastBtnLongPress(void)
{
    if(BTAPP_TaskState != BT_STATE_BT_RUNNING)
        return;
	
    switch(BT_eCSBStatus.nspk_status)
    {
        case BT_CSB_STATUS_STANDBY:
            BTAPP_BroadcastAdd();
            break;
        case BT_CSB_STATUS_CONNECTING:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE3, BT_linkIndex);     //cancel NSPK creation
            else
                NextMMIActionReq2.CancelNSPKReq = 1;
            break;
        case BT_CSB_STATUS_CONNECTED_AS_NSPK_MASTER:
        case BT_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE4, BT_linkIndex);     //terminate NSPK link
            else
                NextMMIActionReq2.TerminateNSPKReq = 1;
            break;
        case BT_CSB_STATUS_NSPK_MASTER_CONNECTING:
            //N/A
            break;
        case BT_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER:
        case BT_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE5, BT_linkIndex);     //cancel/terminate NSPK link
            else
                NextMMIActionReq2.TerminateCancelNSPKReq = 1;
            break;
        case BT_CSB_STATUS_BROADCAST_MASTER_CONNECTING:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE3, BT_linkIndex);     //cancel NSPK creation
            else
                NextMMIActionReq2.CancelNSPKReq = 1;
            break;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastBtnShortPress(void)
{
    //NA
}

/*-----------------------------------------------------------------------------*/
void BTAPP_BroadcastBtnDbClick(void)
{
    if(BTAPP_GetStatus() != BT_STATUS_READY)
        return;
    if(BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER ||
    BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE)
    {
        if (BT_IsCommandSendTaskIdle())
            BT_MMI_ActionCommand(0xF6, BT_linkIndex);
        else
            NextMMIActionReq2.MMI_F6_Req = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_ExitBroadcastRegisterMode( void )
{
    if(BT_eCSBStatus.nspk_link == BT_BROADCAST_MASTER)      //broadcast mode master speaker only
    {
        if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0xE3, BT_linkIndex);     //cancel NSPK creation
            else
                NextMMIActionReq2.CancelNSPKReq = 1;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_GroupSpeakerSoundSync( void )
{
    if(BT_eCSBStatus.nspk_link != BT_NSPK_NO_LINK && BT_eCSBStatus.nspk_link != BT_BROADCAST_SLAVE)     //NSPK exist & not broadcast slave
    {
        if (BT_IsCommandSendTaskIdle())
            BT_MMI_ActionCommand ( 0xF7, BT_linkIndex);
        else
            NextMMIActionReq2.MMI_F7_Req = 1;
    }
}

/*-----------------------------------------------------------------------------*/
//below is calling handler for short button pressing event
void BTAPP_CallEventShort( void )
{
    switch(BT_CallStatus)
    {
        case BT_CALL_IDLE:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0x0A, BT_linkIndex);     //voice dial
            else
                NextMMIActionReq2.VoiceDialReq = 1;
            break;
        case BT_VOICE_DIAL:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0x0B, BT_linkIndex);     //cancel voice dial
            else
                NextMMIActionReq2.cancelVoiceDialReq  = 1;
            break;
        case BT_CALL_INCOMMING:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( ACCEPT_CALL, BT_linkIndex);
            else
                NextMMIActionReq.AcceptCallReq = 1;
            break;
        case BT_CALL_OUTGOING :
        case BT_CALLING:
        case BT_CALLING_WAITING:
        case BT_CALLING_HOLD:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( FORCE_END_CALL, BT_linkIndex);
            else
                NextMMIActionReq.ForceEndCallReq = 1;
            break;
        default:
            break;
    }

}

/*-----------------------------------------------------------------------------*/
//below is calling handler for long button pressing event
void BTAPP_CallEventLong(void)
{
    switch(BT_CallStatus)
    {
        case BT_CALL_IDLE:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( LAST_NUMBER_REDIAL, BT_linkIndex);
            else
                NextMMIActionReq.LastNumberRedialReq = 1;
            
            break;
        case BT_VOICE_DIAL:
            //do nothing
            break;
        case BT_CALL_INCOMMING:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( REJECT_CALL, BT_linkIndex);
            else
                NextMMIActionReq.RejectCallReq = 1;
            break;
        case BT_CALL_OUTGOING :
            //do nothing
            break;
        case BT_CALLING:
            if (BT_IsCommandSendTaskIdle())
                BT_MMI_ActionCommand ( 0x0E, BT_linkIndex);     //transfer call
            else
                NextMMIActionReq2.TransferCallReq = 1;
            break;
        case BT_CALLING_WAITING:
        case BT_CALLING_HOLD:
            //do nothing
            break;
        default:
            break;
    }

}

/*-----------------------------------------------------------------------------*/
//Below are for volume exchange function
uint8_t pressVolUp = 0;



void BTAPP_DebugOnApp(void)
{

dspEQ[0] = BTEQ_SendIndex;
dspEQ[1] = BTEQ_Debug;
dspEQ[2] = BT_eCSBStatus.nspk_status ;
switch(pressVolUp)
{
	case 0:
	BT_SendSPPData(dspEQ, 30, 0);
	break;
	case 1:
		BT_SendSPPData(dspEQ+30, 30, 0);
	break;
	case 2:
		BT_SendSPPData(dspEQ+60, 30, 0);
	break;
}	
if(++pressVolUp > 2)
	pressVolUp=0;



}



void BTAPP_VolUp( uint8_t cmdInfo )
{
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            if ((BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE2
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE3
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_BOTH
                    || BT_eCSBStatus.nspk_link == BT_NSPK_SLAVE_LINK_TO_MASTER
					|| BT_eCSBStatus.nspk_link == BT_BROADCAST_MASTER
				)
                    && BTAPP_TaskState >= BT_STATE_POWER_ON ) //NSPK mode
            {
                    BT_Send_ActionCommand(INC_SPK_GAIN, BT_linkIndex, cmdInfo);
            }
            else //single speaker mode or NSPK connecting
            {
                volumeUp(A2DP);
                DSP_setVol(A2DP);
                if(BTAPP_TaskState >= BT_STATE_POWER_ON)
                {
                    if (BT_IsCommandSendTaskIdle()) {
                        BT_updateA2DPGain(volumeFormatTo7bits(BTAPP_Volume.a2dpVol), cmdInfo);
                    }
                    else {
                        NextCommandReq.updateA2DPGainReq = 1;
                    }
                }
            }
            break;
        case HFP:
            volumeUp(HFP);
            DSP_setVol(HFP);
            if(BTAPP_TaskState >= BT_STATE_POWER_ON)
            {
                if (BT_IsCommandSendTaskIdle()) {
                    BT_updateHFPGain(volumeFormatTo4bits(BTAPP_Volume.hfpVol), cmdInfo);
                }
                else {
                    NextCommandReq.updateHFPGainReq = 1;
                }
            }
            break;
        case LINE_IN:
            if ((BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE2
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE3
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_BOTH
                    || BT_eCSBStatus.nspk_link == BT_NSPK_SLAVE_LINK_TO_MASTER
					|| BT_eCSBStatus.nspk_link == BT_BROADCAST_MASTER
				)
                    && BTAPP_TaskState >= BT_STATE_POWER_ON ) //NSPK mode
            {
                    BT_Send_ActionCommand(INC_SPK_GAIN, BT_linkIndex, cmdInfo);
            }
			else
			{
            volumeUp(LINE_IN);
            DSP_setVol(LINE_IN);
            if(BTAPP_TaskState >= BT_STATE_POWER_ON)
            {
	                if (BT_IsCommandSendTaskIdle()) {
	                    BT_updateLineInGain(volumeFormatTo4bits(BTAPP_Volume.lineInVol), cmdInfo);
                }
                else {
                    NextCommandReq.updateLineInGainReq = 1;
                }
            }
			}
            break;
    }
}

/*-----------------------------------------------------------------------------*/
void BTAPP_VolDown( uint8_t cmdInfo )
{
    switch(BTAPP_Volume.currentVolMode)
    {
        case A2DP:
            if ((BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE2
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE3
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_BOTH
                    || BT_eCSBStatus.nspk_link == BT_NSPK_SLAVE_LINK_TO_MASTER
					|| BT_eCSBStatus.nspk_link == BT_BROADCAST_MASTER
				)
                    && BTAPP_TaskState >= BT_STATE_POWER_ON ) //NSPK mode
            {
                    BT_Send_ActionCommand(DEC_SPK_GAIN, BT_linkIndex, cmdInfo);
            }
            else //single speaker mode or NSPK connecting
            {
                volumeDown(A2DP);
                DSP_setVol(A2DP);
                if(BTAPP_TaskState >= BT_STATE_POWER_ON)
                {
                    if (BT_IsCommandSendTaskIdle()) {
                        BT_updateA2DPGain(volumeFormatTo7bits(BTAPP_Volume.a2dpVol), cmdInfo);
                    }
                    else {
                        NextCommandReq.updateA2DPGainReq = 1;
                    }
                }
            }
            break;
        case HFP:
            volumeDown(HFP);
            DSP_setVol(HFP);
            if(BTAPP_TaskState >= BT_STATE_POWER_ON)
            {
                if (BT_IsCommandSendTaskIdle()) {
                    BT_updateHFPGain(volumeFormatTo4bits(BTAPP_Volume.hfpVol), cmdInfo);
                }
                else {
                    NextCommandReq.updateHFPGainReq = 1;
                }
            }
            break;
        case LINE_IN:
            if ((BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE2
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE3
                    || BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_BOTH
                    || BT_eCSBStatus.nspk_link == BT_NSPK_SLAVE_LINK_TO_MASTER
					|| BT_eCSBStatus.nspk_link == BT_BROADCAST_MASTER
				)
                    && BTAPP_TaskState >= BT_STATE_POWER_ON ) //NSPK mode
            {
                    BT_Send_ActionCommand(DEC_SPK_GAIN, BT_linkIndex, cmdInfo);
            }
			else
			{
            volumeDown(LINE_IN);
            DSP_setVol(LINE_IN);
            if(BTAPP_TaskState >= BT_STATE_POWER_ON)
            {
	                if (BT_IsCommandSendTaskIdle()) {
	                    BT_updateLineInGain(volumeFormatTo4bits(BTAPP_Volume.lineInVol), cmdInfo);
                }
                else {
                    NextCommandReq.updateLineInGainReq = 1;
                }
            }
			}
            break;
    }
}

/*-----------------------------------------------------------------------------*/
/*
 * bool getI2SAuxInJumper( void )
 * bool getDatabase3Jumper( void )
 */
/*-----------------------------------------------------------------------------*/
bool getI2SAuxInJumper( void )
{

#if 0	
    if(BT_Port1.PORT_1_5)       //HIGH: I2S     LOW: ANALOG
        return true;
    else
        return false;
#else 	
        return false;
#endif 
}

/*-----------------------------------------------------------------------------*/
bool getDatabase3Jumper( void )
{
#if 0	
    if(BT_Port3.PORT_3_6)       //HIGH: use BLE APP to power on system and enable BLE when power off
        return true;
    else
        return false;
#else 	
        return true;
#endif 
}

/*-----------------------------------------------------------------------------*/
/*below functions are for SPP process*/
/*
 * bool BTAPP_SendDataOverSPP(uint8_t* addr, uint32_t size)
 * bool BTAPP_SendCharOverSPP(uint8_t byte)
 * void BT_SPPBuffClear( void )
 * bool BT_AddBytesToSPPBuff(uint8_t* data, uint8_t size)
 * bool BT_ReadOneByteFromSPPBuff( uint8_t* byte )
 */
/*-----------------------------------------------------------------------------*/
//Below are for SPP data exchange function
bool BTAPP_SendDataOverSPP(uint8_t* addr, uint32_t size)
{
    BT_SendSPPData(addr, (uint16_t)size, BT_linkIndex);
    return true;
}

/*-----------------------------------------------------------------------------*/
bool BTAPP_SendCharOverSPP(uint8_t byte)
{
    BT_SendSPPData(&byte, 1, BT_linkIndex);
    return true;
}

/*-----------------------------------------------------------------------------*/
void BT_SPPBuffClear( void )
{
    uint16_t i;
    BT_SPP_RxFifoHead = 0;
    BT_SPP_RxFifoTail = 0;
    BT_SPP_RxCounter = 0;
    for(i = 0; i< sizeof(BT_SPP_RxFifo); i++)
        BT_SPP_RxFifo[i] = 0;
}

/*-----------------------------------------------------------------------------*/
bool BT_AddBytesToSPPBuff(uint8_t* data, uint8_t size)        //TRUE: data added ok, FALSE: data added fail, buffer is overflow
{
    uint8_t i;
    for(i=0; i<size; i++)
    {
        if(BT_SPP_RxCounter < BT_SPP_RxFifoSize)
        {
            BT_SPP_RxFifo[BT_SPP_RxFifoTail] = *data++;
            BT_SPP_RxCounter++;
            if(BT_SPP_RxFifoTail < BT_SPP_RxFifoSize-1)
                BT_SPP_RxFifoTail++;
            else
                BT_SPP_RxFifoTail = 0;
        }
        else
        {
            return false;
        }
    }
    return true;
}

/*-----------------------------------------------------------------------------*/
bool BT_ReadOneByteFromSPPBuff( uint8_t* byte )
{
    if(BT_SPP_RxCounter)
    {
        *byte = BT_SPP_RxFifo[BT_SPP_RxFifoHead];
        BT_SPP_RxCounter--;
        if(BT_SPP_RxFifoHead < BT_SPP_RxFifoSize-1)
            BT_SPP_RxFifoHead++;
        else
            BT_SPP_RxFifoHead = 0;
        return true;
    }
    else
        return false;
}

/*-----------------------------------------------------------------------------*/
//this is test function, don't need to migrate the code.
void SPPTask(void)
{
    switch (BT_SPP_State)
    {
        case 0:     //IDLE
            break;
        case 1:     //SENDING RAW DATA
            if (BT_IsCommandSendTaskIdle())
            {
                BT_LoopBackSPPData(BT_SPP_RxFifo, BT_SPP_RxCounter);
                BT_SPP_State = 2;
            }
            break;
        case 2:     //STATUS CHECKING(for RAW data checking)
            if(BT_GetAckStatusSendSPPData() == STS_OK)
            {
                BT_SendAckToEvent(0x22);        //ACK to event 0x22, moved this from bt_command_decode.c to here
                BT_SPP_State = 0;       //ok, set to IDLE
            }
//            else if(BT_GetAckStatusSendSPPData() == ACK_BTM_BUSY || BT_GetAckStatusSendSPPData() == ACK_OUT_OF_MEMORY)
//                BT_SPP_State = 1;       //retransmit data
            break;
        case 3:     //SENDING COMMAND
            if (BT_IsCommandSendTaskIdle())
            {
                BT_SendBytesAsCompleteCommand(BT_SPP_RxFifo, BT_SPP_RxCounter);   //send payload as a complete command directly
                BT_SendAckToEvent(0x22);        //ACK to event 0x22, moved this from bt_command_decode.c to here
                BT_SPP_State = 0;
            }
            break;
    }
}

/*-----------------------------------------------------------------------------*/
uint8_t IsSPPTaskIdle(void)
{
    if(BT_SPP_State == 0)
        return true;
    else
        return false;
}

/*-----------------------------------------------------------------------------*/
//this is test function, don't need to migrate the code.
bool BT_AddBytesToSPPBuffFromBeginning(uint8_t* data, uint8_t size, bool data_True_Command_False)
{
    uint8_t i;
    if(size > BT_SPP_RxFifoSize)
        return false;
    BT_SPP_RxCounter = size;
    BT_SPP_RxFifoTail = 0;
    BT_SPP_RxFifoHead = 0;
    for(i=0; i<size; i++)
    {
        BT_SPP_RxFifo[BT_SPP_RxFifoTail++] = *data++;
        //BT_SPP_RxCounter++;
    }
    if(data_True_Command_False)
        BT_SPP_State = 1;       //strat to send these RAW DATA, test function.
    else
        BT_SPP_State = 3;
    return true;
}

/*-----------------------------------------------------------------------------*/

bool BT_AddBytesToSPPBuff_Fragmented_Pkt(uint8_t* data, uint8_t size, uint8_t pkt_type)
{
    uint8_t i;
    uint16_t cmd_len;
    if(size > BT_SPP_RxFifoSize)
        return false;
    if (pkt_type == BT_FRAGMENTED_START_PKT)
    {
        BT_SPP_RxCounter = size;
        BT_SPP_RxFifoTail = 0;
        BT_SPP_RxFifoHead = 0;
    }
    else
    {
        //BT_SPP_RxCounter += size;
        BT_SPP_RxCounter += data[2];
    }

    if ((pkt_type == BT_FRAGMENTED_END_PKT) || (pkt_type == BT_FRAGMENTED_CONT_PKT))
    {
        *data++;
        *data++;
        size = *data++;
    }
    
    for(i=0; i<size; i++)
    {
        BT_SPP_RxFifo[BT_SPP_RxFifoTail++] = *data++;
        //BT_SPP_RxCounter++;
    }
    if ((pkt_type == BT_FRAGMENTED_START_PKT) || (pkt_type == BT_FRAGMENTED_CONT_PKT))
        BT_SPP_State = 4;       //strat to send these RAW DATA, test function.
    else if (pkt_type == BT_FRAGMENTED_END_PKT)
    {
        
        cmd_len = BT_SPP_RxCounter - 4;  // minus the first 3 bytes and the checksum byte
        BT_SPP_RxFifo[1] = (cmd_len >> 8) & 0xFF;   
        BT_SPP_RxFifo[2] = cmd_len & 0xFF;
        if(ParsePayloadAsCommand(BT_SPP_RxFifo, BT_SPP_RxCounter))
            BT_SPP_State = 3;
        else
            BT_SPP_State = 1;
    }   
    return true;
}

/*-----------------------------------------------------------------------------*/
/*
 * void BT_SaveLocalBDAddress(uint8_t* address)
 * void BT_SaveMasterBDAddress(uint8_t* address)
 */
/*-----------------------------------------------------------------------------*/
void BT_SaveLocalBDAddress(uint8_t* address)
{
    uint8_t i;
    for(i=0; i<6; i++)
        BT_localBDAddr[i] = *address++;
}
void BT_SaveDeviceName(uint8_t* name)
{
    BLE_SaveDeviceName(name);
}

void BT_SaveMasterBDAddress(uint8_t* address)
{
    uint8_t i;
    for(i=0; i<6; i++)
        BT_masterBDAddr[i] = *address++;
}

/*---------------------------------------------------------------------------------------------------------------*/
/*
 * bool BT_CustomerGATT_AttributeData(uint8_t attributeIndex, uint8_t* attributeData, uint8_t attributeDataLength)
 */
/*---------------------------------------------------------------------------------------------------------------*/
bool BT_CustomerGATT_AttributeData(uint8_t attributeIndex, uint8_t* attributeData, uint8_t attributeDataLength)
{
    switch(attributeIndex)
    {
        case 1:
            if (BT_IsCommandSendTaskIdle()) {
                BT_LinkBackToDeviceByBTAddress(attributeData);
            }
            else {
                //save BD address
                linkbackBDAddr[0] = *attributeData++;
                linkbackBDAddr[1] = *attributeData++;
                linkbackBDAddr[2] = *attributeData++;
                linkbackBDAddr[3] = *attributeData++;
                linkbackBDAddr[4] = *attributeData++;
                linkbackBDAddr[5] = *attributeData;
                NextCommandReq.linkbackToDevAddr = 1;
            }
            break;
        case 2:
            if(*attributeData == 0x01)
            {
                if (BT_IsCommandSendTaskIdle()) {
                    BT_MMI_ActionCommand(ANY_MODE_ENTERING_PAIRING, BT_linkIndex);
                    BT_LinkbackStatus = BT_PAIRING_START;
                }
                else {
                    NextMMIActionReq.PairReq = 1;
                }
            }
            break;
        default:
            break;
    }
    return true;
}
bool BT_isBLEConnected(void)
{
	return (ble_connect_state == BLE_CONN_CONNECTED) ? 1 :0;
}

void BT_SendEQBytes(void)
{

	if(BTEQ_SendIndex == (BTEQ_MAX_INDEX+1) ||  BTEQ_timer1ms !=0)
	{
		return;
	}
	else if(BTEQ_SendIndex == BTEQ_MAX_INDEX)
	{
		BT_Send_EQCmd(dspEQ, 90);
		BTEQ_SendIndex++;
	}
	else
	{
		BT_Vendor_EQCmd(dspEQ+5, BTEQ_SendIndex,(BTEQ_SendIndex == (BTEQ_MAX_INDEX - 1)) ? (DSP_EQ_SIZE % (VENDOR_CMD_SIZE-1))+1 : VENDOR_CMD_SIZE);
		BTEQ_SendIndex++;
		if(BTEQ_SendIndex == BTEQ_MAX_INDEX)
		{
			if(multiSpk_eq_route == MULTI_SPK_EQ_SLAVE)
				BTEQ_SendIndex = BTEQ_MAX_INDEX+1;
			else
				BTEQ_timer1ms = DSP_EQ_SEND_INTERVAL;
		}
		else
			BTEQ_timer1ms = DSP_EQ_SEND_INTERVAL;
		
	}
}

void BT_SetMultiSpkEQRoute(uint8_t route)
{
	multiSpk_eq_route = route;
}

bool BT_SendMultiSpkEQBytes(uint8_t* data, uint8_t size)
{
    uint8_t i;
    uint16_t cmd_len;
	uint8_t *ptr = dspEQ;
	
    if((BT_eCSBStatus.nspk_status == BT_CSB_STATUS_STANDBY ||
    	BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTING ||
    	BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE ||
    	BT_eCSBStatus.nspk_status == BT_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE) ||
	   (multiSpk_eq_route == MULTI_SPK_EQ_MASTER))
        return false;

	*(ptr++) = 0xaa;
	*(ptr++) = 0x00;
	*(ptr++) = 84+2;
	*(ptr++) = 0x30;
	*(ptr++) = 0x13;

    for(i=0; i<size; i++)
    {
        *(ptr++) = *(data++);
    }
	BTEQ_SendIndex = 0;
	BTEQ_timer1ms = DSP_EQ_SEND_INTERVAL;  //send later to avoid master eq corrupt
    return true;
}

bool BT_isA2DPPlaying(void)
{
	return ((BT_LinkStatus & BIT_A2DP_LINK_STATUS));
}
bool BT_isA2DPPaused(void)
{
	return ((BT_LinkStatus & BIT_A2DP_LINK_STATUS) && 
		    (BT_PlayingStatus == BT_PLAYING_PAUSED || 
		     BT_PlayingStatus == BT_PLAYING_STOPPED));
}

void BT_ToggleAudioPath(void)
{
	if(AnalogIsPlugIn() == true &&
		(BT_LinkStatus & BIT_A2DP_LINK_STATUS))
	{
		AudioSelectToggle();	
	}		
}
void BT_A2DPAppStatus(void)
{
	BT_SendAppBTMStatus((BT_LinkStatus & BIT_A2DP_LINK_STATUS) ? 0x06 : 0x08);
}

bool BT_isMaster(void)
{
	return (
			BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE2
			|| BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_SLAVE3
			|| BT_eCSBStatus.nspk_link == BT_NSPK_MASTER_LINK_TO_BOTH
			|| BT_eCSBStatus.nspk_link == BT_BROADCAST_MASTER);

}
bool BT_isStandalone(void)
{
	return ( BT_eCSBStatus.nspk_link == BT_NSPK_NO_LINK);
}
void BT_SendMuteInfo(uint8_t mute)
{
	// All Vol mode to Mute
	BT_Vendor_SendVolInfo(ALL_MODE, mute == 1 ? ALL_MUTE : ALL_UNMUTE);
}

void BT_SendVolInfo(void)
{
	if(AudioSelected() == AUDIO_AUXIN)
		BT_Vendor_SendVolInfo(LINE_IN, BTAPP_Volume.lineInVol);
	else
		BT_Vendor_SendVolInfo(A2DP, BTAPP_Volume.a2dpVol);
}
bool BT_isCallIdle(void)
{
	return (BT_CallStatus == BT_CALL_IDLE);
}
uint8_t BT_GetSysStatus(void)
{
	return BT_SystemStatus;
}
uint8_t BT_GetLinkStatus(void)
{
	return BT_LinkStatus;
}
void BTAPP_MFBOnMode(void)
{

	switch(BT_CallStatus)
	{		
		case BT_CALL_IDLE:
			BTAPP_TaskReq(BT_REQ_SYSTEM_OFF);
		break;
		case BT_CALL_INCOMMING:
			if (BT_IsCommandSendTaskIdle())
				BT_MMI_ActionCommand ( ACCEPT_CALL, BT_linkIndex);
			else
				NextMMIActionReq.AcceptCallReq = 1;
		break;
		case BT_CALL_OUTGOING :
		case BT_CALLING:
		case BT_CALLING_WAITING:
		case BT_CALLING_HOLD:
			if (BT_IsCommandSendTaskIdle())
				BT_MMI_ActionCommand ( FORCE_END_CALL, BT_linkIndex);
			else
				NextMMIActionReq.ForceEndCallReq = 1;
		break;
	}	
}

typedef enum {
    BT_CMD_SEND_STATE_IDLE = 0,         //no data in the queue, MFB is low
    BT_CMD_SEND_MFB_HIGH_WAITING,       //set MFB to high, and wait for 2ms
    BT_CMD_SEND_DATA_SENDING,           //UART interface is working
    //BT_CMD_SEND_DATA_SENT_WAITING,      //wait for another 10ms after all data are sent out     ...//no longer need
    BT_CMD_SEND_ACK_WAITING,        //new...waiting for ack
    BT_CMD_SEND_ACK_ERROR,
    BT_CMD_SEND_ACK_OK,
} BT_CMD_SEND_STATE;

extern BT_CMD_SEND_STATE BT_CMD_SendState;

void print_nspk_status() {

	BT_nspk_link = BT_eCSBStatus.nspk_link;
	BT_nspk_event = BT_eCSBStatus.snpk_event;
	BT_nspk_status = BT_eCSBStatus.nspk_status;

	BTAPP_task_status = BTAPP_TaskState;

	Print_BT_CMD_SendState = BT_CMD_SendState;

	//printf("BT_eCSBStatus.nspk_status = %d\r\n",  BT_eCSBStatus.nspk_status);
}
