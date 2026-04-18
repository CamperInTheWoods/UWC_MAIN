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
#ifndef BT_APP_H
#define BT_APP_H

#include <stdbool.h>
#include <stdint.h>

// #define DATABASE2_TEST	// by jaehoon testing 

void BTAPP_Init( void );
void BTAPP_Task( void );
void BTAPP_Timer1MS_event( void );
// @ request define
enum {
    BT_REQ_NONE = 0,
    BT_REQ_SYSTEM_ON,   //BM64 power on request(outside application must control RESET and MFB timing, then call this request)
    BT_REQ_SYSTEM_OFF,  //BM64 power off request(after call this, outside application must control RESET and MFB)
};

enum {
    BT_SINGLE_PKT = 0,
    BT_FRAGMENTED_START_PKT,  
    BT_FRAGMENTED_CONT_PKT, 
    BT_FRAGMENTED_END_PKT,
};

void BTAPP_TaskReq(uint8_t request);

// @ event define
enum {
    BT_EVENT_NONE = 0,

    BT_EVENT_CMD_SENT_ACK_OK,       //ACK OK
    BT_EVENT_CMD_SENT_ACK_ERROR,        //ACK error
    BT_EVENT_CMD_SENT_NO_ACK,           //no ACK
    
    BT_EVENT_NSPK_STATUS,
    BT_EVENT_LINE_IN_STATUS,
    BT_EVENT_A2DP_STATUS,
    BT_EVENT_CALL_STATUS_CHANGED,

    BT_EVENT_HFP_CONNECTED,
    BT_EVENT_HFP_DISCONNECTED,
    BT_EVENT_A2DP_CONNECTED,
    BT_EVENT_A2DP_DISCONNECTED,
    BT_EVENT_AVRCP_CONNECTED,
    BT_EVENT_AVRCP_DISCONNECTED,
    BT_EVENT_SPP_CONNECTED,
    BT_EVENT_IAP_CONNETED,
    BT_EVENT_SPP_IAP_DISCONNECTED,
    BT_EVENT_ACL_CONNECTED,
    BT_EVENT_ACL_DISCONNECTED,
    BT_EVENT_SCO_CONNECTED,
    BT_EVENT_SCO_DISCONNECTED,
    BT_EVENT_MAP_CONNECTED,
    BT_EVENT_MAP_DISCONNECTED,

    BT_EVENT_SYS_POWER_ON,
    BT_EVENT_SYS_POWER_OFF,
    BT_EVENT_SYS_STANDBY,
    BT_EVENT_SYS_PAIRING_START,
    BT_EVENT_SYS_PAIRING_OK,
    BT_EVENT_SYS_PAIRING_FAILED,

    BT_EVENT_LINKBACK_SUCCESS,
    BT_EVENT_LINKBACK_FAILED,

    BT_EVENT_BD_ADDR_RECEIVED,
    BT_EVENT_NAME_RECEIVED,
    BT_EVENT_PAIR_RECORD_RECEIVED,
    BT_EVENT_LINK_MODE_RECEIVED,

    BT_EVENT_PLAYBACK_STATUS_CHANGED,
    BT_EVENT_AVRCP_VOLUME_CTRL,
    BT_EVENT_AVRCP_ABS_VOLUME_CHANGED,
    BT_EVENT_HFP_VOLUME_CHANGED,
	BT_EVENT_TYPE_CODEC,

    
    NSPK_EVENT_SYNC_POWER_OFF,
    NSPK_EVENT_SYNC_VOL_CTRL,
    NSPK_EVENT_SYNC_INTERNAL_GAIN,
    NSPK_EVENT_SYNC_ABS_VOL,
    NSPK_EVENT_CHANNEL_SETTING,
    NSPK_EVENT_ADD_SPEAKER3,
    
    LE_STATUS_CHANGED,
    LE_ADV_CONTROL_REPORT,
    LE_CONNECTION_PARA_REPORT,
    LE_CONNECTION_PARA_UPDATE_RSP,
    GATT_ATTRIBUTE_DATA,
    
    PORT0_INPUT_CHANGED,
    PORT1_INPUT_CHANGED,
    PORT2_INPUT_CHANGED,
    PORT3_INPUT_CHANGED,

	NSPK_EVENT_VENDOR_CMD,
	NSPK_EVENT_VOL_MODE,
	NSPK_EVENT_EQ_MODE,	
	FEATURE_LIST_REPLY

};

#define DSP_EQ_SIZE    84
#define DSP_EQ_SEND_INTERVAL   100		   //  100ms
#define VENDOR_CMD_SIZE 	   9   
#define BTEQ_MAX_INDEX		   11		   // 84bytes + 1-seq each packet ==> 84/8 = 11 packets


enum {
    BT_STATUS_NONE,
    BT_STATUS_OFF,
    BT_STATUS_ON,
    BT_STATUS_READY
};
typedef enum {
    BT_SYSTEM_INIT,         //init
    BT_SYSTEM_POWER_OFF,    //event
    BT_SYSTEM_POWER_ON,     //event
    BT_SYSTEM_STANDBY,      //event
    BT_SYSTEM_CONNECTED,    //event
    BT_SYSTEM_PAIRING,      //event
} BT_SYS_STATUS;        // BM64 internal system status
#define BIT_SCO_LINK_STATUS 0x01//bit0: SCO link status
#define BIT_ACL_LINK_STATUS 0x02//bit1: ACL link status
#define BIT_HFP_LINK_STATUS 0x04//bit2: HFP link status
#define BIT_A2DP_LINK_STATUS 0x08//bit3: A2DP link status
#define BIT_AVRCP_LINK_STATUS 0x10//bit4: AVRCP link status
#define BIT_SPP_LINK_STATUS 0x20//bit5: SPP link status
#define BIT_IAP_LINK_STATUS 0x40//bit6: iAP link status
#define BIT_MAP_LINK_STATUS 0x80//bit7: MAP link status


//===============================================
//  VARIABLES CLAIM
//===============================================


//@
enum {
    BT_CALL_IDLE = 0,
    BT_VOICE_DIAL = 1,
    BT_CALL_INCOMMING = 2,
    BT_CALL_OUTGOING = 3,
    BT_CALLING = 4,
    BT_CALLING_WAITING = 5,
    BT_CALLING_HOLD = 6
} BT_CallStatus;
	
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
    BT_STATE_INIT_READ_DEVICE_ADDR_WAIT,  //8
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
    BT_STATE_POWER_OFF,//36
    BT_STATE_AUTORUN_NSPK_EVENT

} BTAPP_TaskState;

enum BT_NSPK_LINK_STATUS {
    BT_NSPK_NO_LINK = 0,
    BT_NSPK_MASTER_LINK_TO_SLAVE2 = 1,
    BT_NSPK_MASTER_LINK_TO_SLAVE3 = 2,
    BT_NSPK_MASTER_LINK_TO_BOTH = 3,
    BT_NSPK_SLAVE_LINK_TO_MASTER = 4,
    BT_BROADCAST_MASTER = 5,
    BT_BROADCAST_SLAVE = 6
};
enum BT_NSPK_EVENT {
    BT_CSB_EVENT_STANDBY = 0,
    BT_CSB_EVENT_BUSY = 1,
    BT_CSB_EVENT_CONNECTING = 2,
    BT_CSB_EVENT_CONNECTED = 3,
    BT_CSB_EVENT_LOSS = 4,
    BT_NSPK_EVENT_BACK_TO_MASTER = 5,
    BT_NSPK_EVENT_BACK_TO_SLAVE = 6,
    BT_CSB_EVENT_CHANGE_ROLE = 7,
    BT_CSB_EVENT_DISCONNECTED_BY_NFC = 8,
    BT_CSB_EVENT_CONTINUE_CONNECTING = 9
};


enum BT_NSPK_SYSTEM_STATUS {
    BT_CSB_STATUS_STANDBY = 0,
    BT_CSB_STATUS_CONNECTING,  // 1
    BT_CSB_STATUS_CONNECTED_AS_NSPK_MASTER,
    BT_CSB_STATUS_CONNECTED_AS_NSPK_SLAVE,
    BT_CSB_STATUS_NSPK_MASTER_CONNECTING,
    BT_CSB_STATUS_CONNECTED_AS_BROADCAST_MASTER, // 5
    BT_CSB_STATUS_CONNECTED_AS_BROADCAST_SLAVE,
    BT_CSB_STATUS_BROADCAST_MASTER_CONNECTING,   // 7
};
struct {
    enum BT_NSPK_LINK_STATUS nspk_link;
    enum BT_NSPK_EVENT snpk_event;
    enum BT_NSPK_SYSTEM_STATUS  nspk_status;
} BT_eCSBStatus;


extern uint8_t BTAPP_task_status;

uint8_t BTAPP_GetStatus(void);
void BT_SendEQBytes(void);


void BTAPP_EventHandler(uint8_t event, uint16_t para, uint8_t* para_full);

void BTAPP_EnterBTPairingMode( void );
/*void BTAPP_SetBTDiscoverable( void );*/
void BTAPP_ResetEEPROMtoDefault( void );
void BTAPP_PlayNextSong( void );
void BTAPP_PlayPreviousSong( void );
void BTAPP_PlayPause( void );
void BTAPP_StartFastForward( void );
void BTAPP_StartFastRewind( void );
void BTAPP_CancelForwardOrRewind( void );
void BTAPP_BroadcastBtnLongPress(void);
void BTAPP_BroadcastBtnDbClick(void);
void BTAPP_ExitBroadcastRegisterMode( void );
void BTAPP_GroupSpeakerSoundSync( void );
void BTAPP_NSPKBtnLongPress( void );
void BTAPP_NSPKBtnDbClick( void );
void BTAPP_NSPKBtnShortPress( void );
void BTAPP_CallEventShort( void );
void BTAPP_CallEventLong(void);
void BTAPP_VolUp(uint8_t cmdInfo);
void BTAPP_VolDown( uint8_t cmdInfo );
//void BTAPP_LineInEvent( uint8_t lineInIsPlugged);
bool getI2SAuxInJumper( void );
bool getDatabase3Jumper( void );
//SPP data exchange
void BT_SPPBuffClear( void );
bool BTAPP_SendDataOverSPP(uint8_t* addr, uint32_t size);
bool BTAPP_SendCharOverSPP(uint8_t byte);
bool BT_AddBytesToSPPBuff(uint8_t* data, uint8_t size);
bool BT_AddBytesToSPPBuffFromBeginning(uint8_t* data, uint8_t size, bool data_True_Command_False)  ;   
bool BT_AddBytesToSPPBuff_Fragmented_Pkt(uint8_t* data, uint8_t size, uint8_t pkt_type);
void SPPTask(void);                                 //test
uint8_t IsSPPTaskIdle(void);

bool BT_ReadOneByteFromSPPBuff( uint8_t* byte );
void BT_SaveLocalBDAddress(uint8_t* address);
void BT_SaveMasterBDAddress(uint8_t* address);
bool BT_CustomerGATT_AttributeData(uint8_t attributeIndex, uint8_t* attributeData, uint8_t attributeDataLength);
bool BT_isBLEConnected(void);
bool BT_SendMultiSpkEQBytes(uint8_t* data, uint8_t size);
void BT_SetMultiSpkEQRoute(uint8_t route);
bool BT_isA2DPPlaying(void);
bool BT_isA2DPPaused(void);
void BT_SaveDeviceName(uint8_t* name);
void BT_ToggleAudioPath(void);
void BT_A2DPAppStatus(void);
bool BT_isMaster(void);
bool BT_isStandalone(void);
void BT_SendVolInfo(void);
uint8_t BT_GetSysStatus(void);
uint8_t BT_GetLinkStatus(void);
void BTAPP_MFBOnMode(void);
void BT_SendMuteInfo(uint8_t mute);
bool BT_isCallIdle(void);


#endif
