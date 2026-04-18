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
#include "bt_command_decode.h"
#include "bt_command_send.h"
#include "bt_app.h"
#include "em_gpio.h"
#include "eusart.h"
#include "HA_DefinedValues.h"
#include "HA_MCU_cmd.h"
#include "LED_PWM.h"
#include "ble.h"
#include "Line_in.h"

#define BT_CMD_SIZE_MAX				200
#define Set_LED_Style(x,y,z)

//command decode state machine
typedef enum {
	RX_DECODE_CMD_SYNC_AA,
	RX_DECODE_CMD_SYNC_00,
	RX_DECODE_CMD_LENGTH,
	RX_DECODE_CMD_DATA,
	RX_DECODE_CMD_CHECKSUM
} RX_DECODE_MODE;
typedef enum
{
	APPS_GET_STATUS,
	APPS_SET_STATUS
}APPS_COMMAND;

//event ID
enum            
{
  ACK = 0x00,
  DEVICE_STATE = 0x01,
  CALL_STATUS = 0x02,
  CALL_ID = 0x03,
  SMS_INDICATION = 0x04,
  MISS_CALL_INDICATION = 0x05,
  PHONE_MAX_BATTERY_LEVEL = 0x06,
  PHONE_BATTERY_LEVEL = 0x07,
  PHONE_ROAMING_STATUS = 0x08,
  PHONE_MAX_SIGNAL_STRENGTH = 0x09,
  PHONE_SIGNAL_STRENGTH = 0x0A,
  PHONE_SERVICE_STATUS = 0x0B,
  BATTERY_LEVEL = 0x0C,
  CHARGER_STATUS = 0x0D,
  RESET_TO_DEFAULT = 0x0E,
  VOLUME_LEVEL = 0x0F,
  EQ_MODE = 0x10,
  MISS_CALL_HISTORY = 0x11,
  RECEIVED_CALL_HISTORY = 0x12,
  DIALED_CALL_HISTORY = 0x13,
  COMBINE_CALL_HISTORY = 0x14,
  PHONE_BOOK = 0x15,
  ACCESS_FINISH = 0x16,
  REMOTE_DEVICE_NAME = 0x17,
  UART_VERSION = 0x18,
  CALL_LIST_REPORT = 0x19,
  AVRCP_SPEC_RSP = 0x1A,
  BTM_UTILITY_REQ = 0x1B,
  VENDOR_AT_CMD_RSP = 0x1C,
  UNKNOW_AT_RESULT = 0x1D,
  REPORT_LINK_STATUS = 0x1E,
  REPORT_PAIRING_RECORD = 0x1F,
  REPORT_LOCAL_BD_ADDR = 0x20,
  REPORT_LOCAL_DEVICE_NAME = 0x21,
  REPORT_SPP_DATA = 0x22,
  REPORT_LINK_BACK_STATUS = 0x23,
  RINGTONE_FINISH_INDICATION = 0x24,
  USER_CONFIRM_SSP_REQ = 0x25,
  REPORT_AVRCP_VOL_CTRL = 0x26,
  REPORT_INPUT_SIGNAL_LEVEL = 0x27,
  REPORT_IAP_INFO = 0x28,
  REPORT_AVRCP_ABS_VOL_CTRL = 0x29,
  REPORT_VOICE_PROMPT_STATUS = 0x2A,
  REPORT_MAP_DATA = 0x2B,
  SECURITY_BONDLING_RES = 0x2C,
  REPORT_TYPE_CODEC = 0x2D,
  REPORT_TYPE_BTM_SETTING = 0x2E,
  REPORT_MCU_UPDATE_REPLY = 0x2F,
  REPORT_BTM_INITIAL_STATUS = 0x30,
  REPORT_LE_EVENT = 0x32,
  REPORT_nSPK_STATUS = 0x33,
  REPORT_nSPK_VENDOR_EVENT = 0x34,
  REPORT_Vendor_EEPROM_Data = 0x37,  //20081125
  REPORT_CUSTOMER_GATT_ATTRIBUTE_DATA = 0x39,
  REPORT_LINK_MODE = 0x3A,
  REPORT_FEATURE_LIST = 0x40
};

//Command decode: BTM_Utility_Req
enum
{
    BTM_CONTROL_AMP = 0x00,
    BTM_REPORT_LINE_IN_STATUS = 0x01,
    BTM_NOTIFY_HANDLE_BTM = 0x02,
    BTM_NOTIFY_EEPROM_UPDATE_FINISH = 0x03,
    BTM_REPORT_CODEC_STATUS = 0x04,
    NSPK_SYNC_POWER_OFF = 0x05,
    NSPK_SYNC_VOLUME_CTRL = 0x06,
    NSPK_SYNC_INTERNAL_GAIN = 0x07,
    NSPK_SYNC_A2DP_ABS_VOL = 0x08,
    NSPK_SYNC_CURRENT_CHANNEL_SETTING = 0x09,
    NSPK_NOTIFY_POWER_SYNCED = 0x0A,
    NSPK_NOTIFY_COMMAND_SUCCESS = 0x0B,
    NSPK_NOTIFY_COMMAND_FAIL = 0x0C,
    NSPK_NOTIFY_SLAVE_STATUS_CHANGED = 0x0D,
    NSPK_RESERVED = 0x0E,
    NSPK_NOTIFY_ADD_THIRD_SPEAKER = 0x0F
};
//command decode: BTM status
enum
{
  BT_OFF = 0x00,
  BT_DISCOVERABLE = 0x01,
  BT_ON = 0x02,
  BT_PAIRING_COMPLETE = 0x03,
  BT_PAIRING_FAIL = 0x04,
  BT_HFP_CONNECTED = 0x05,
  BT_A2DP_CONNECTED = 0x06,
  BT_HFP_DISCONNECTED = 0x07,
  BT_A2DP_DISCONNECTED = 0x08,
  BT_SCO_CONNECTED = 0x09,
  BT_SCO_DISCONNECTED = 0x0A,
  BT_ARVCP_CONNECTED = 0x0B,
  BT_AVRCP_DISCONNECTED = 0x0C,
  BT_SPP_CONNECTED = 0x0D,
  BT_SPP_DISCONNECTED = 0x0E,
  BT_STANDBY = 0x0F,
  BT_iAP_CONNECTED = 0x10,
  BT_ACL_DISCONNECTED = 0x11,
  BT_MAP_CONNECTED = 0x12,
  BT_MAP_FORBIDDEN = 0x13,
  BT_MAP_DISCONNECTED = 0x14,
  BT_ACL_CONNECTED = 0x15
};

#define INDIV_ACTION_CMD_LENGTH		8		// 4byte MAC address
//#define INDIV_ACTION_CMD_LENGTH		10		// 6byte MAC address



/*======================*/
/*  external variables  */
/*======================*/
uint8_t  BT_CmdDecodedFlag;
uint8_t  BT_CmdBuffer[BT_CMD_SIZE_MAX	];
uint8_t  BT_linkIndex = 0;

/*======================================*/
/*  internal variables          */
/*======================================*/
static RX_DECODE_MODE  BT_CmdDecodeState;
static uint8_t  BT_CmdDecodeCmdLength;
static uint8_t  BT_CmdDecodeChecksum;			
static uint8_t  BT_CmdDecodeDataCnt;                    //temporary variable in decoding
static unsigned short BT_CmdBufferPt;                    //

#ifdef DATABASE2_TEST       //test only
bool ParsePayloadAsCommand(uint8_t* command, uint8_t length);
#endif
void AppsCommandDecode(uint8_t* buffer);

extern int Blink_Count;

/*======================================*/
/*  function implemention       */
/*======================================*/
void BT_CommandDecodeInit(void)
{
    BT_CmdDecodedFlag = 0;
    BT_CmdDecodeState = RX_DECODE_CMD_SYNC_AA;
    while(eusartRxCount)
    {
        EUSART_Read();
    }
    BT_SPPBuffClear();
}

void BT_CommandDecodeMain(void)
{
	BT_CommandHandler();
	if(BT_CmdDecodedFlag)
	{
            BT_CommandDecode();
            BT_CmdDecodedFlag = 0;
	}
}

extern volatile uint8_t eusartRxCount;

void BT_CommandHandler(void){
    uint8_t current_byte;

    while (eusartRxCount) {
        current_byte = EUSART_Read();

        switch (BT_CmdDecodeState) {
            case RX_DECODE_CMD_SYNC_AA:
                if (current_byte == 0xaa) {
#if defined (DBG_PRINT)
					//printf("\n\rRX : "); // MB64 printf
#endif
                    BT_CmdDecodeState = RX_DECODE_CMD_SYNC_00;
                	}
                break;

            case RX_DECODE_CMD_SYNC_00:
                if (current_byte == 0x00)
                    BT_CmdDecodeState = RX_DECODE_CMD_LENGTH;
                else
                    BT_CmdDecodeState = RX_DECODE_CMD_SYNC_AA;
                break;

            case RX_DECODE_CMD_LENGTH:
                BT_CmdDecodedFlag = 0; //command receive flag clear
                BT_CmdBufferPt = 0; //buffer reset for command parameter
                BT_CmdDecodeCmdLength = current_byte;
                BT_CmdDecodeChecksum = current_byte; //checksum calculation start!
                BT_CmdDecodeDataCnt = current_byte; //save bytes number, use to check where is command end
                BT_CmdDecodeState = RX_DECODE_CMD_DATA; //next state
                break;

            case RX_DECODE_CMD_DATA:
                BT_CmdDecodeChecksum += current_byte;
                BT_CmdDecodeDataCnt--;
                BT_CmdBuffer[BT_CmdBufferPt++] = current_byte;
                if (BT_CmdDecodeDataCnt == 0) //no data remained?
                    BT_CmdDecodeState = RX_DECODE_CMD_CHECKSUM; //yes, next mode: checksum
                break;

            case RX_DECODE_CMD_CHECKSUM:
                if ((uint8_t) (BT_CmdDecodeChecksum + current_byte) == 0) {
                    BT_CmdDecodedFlag = 1;
                } else {
                }
                BT_CmdDecodeState = RX_DECODE_CMD_SYNC_AA;
                break;
            default:
                break;
        }
#if defined (DBG_PRINT)
		 //printf("%02x, ", current_byte);  // MB64 printf
#endif
        if (BT_CmdDecodedFlag) {
            break;
        }
    }
}

#ifdef DATABASE2_TEST       //test only
bool ParsePayloadAsCommand(uint8_t* command, uint8_t length)
{
    bool parse_result = true;
    bool parse_ended = false;
    RX_DECODE_MODE parse_state = RX_DECODE_CMD_SYNC_AA;
    uint8_t parse_checksum, parse_data_len;
    uint8_t current_byte;

    while (length) {
        current_byte = *command++;
        length --;

        switch (parse_state) {
            case RX_DECODE_CMD_SYNC_AA:
                if (current_byte == 0xaa)
                    parse_state = RX_DECODE_CMD_SYNC_00;
                else
                    parse_result = false;           //sync byte is not correct
                break;

            case RX_DECODE_CMD_SYNC_00:
                if (current_byte == 0x00)
                    parse_state = RX_DECODE_CMD_LENGTH;
                else
                    parse_result = false;           //second byte is not correct
                break;

            case RX_DECODE_CMD_LENGTH:
                parse_checksum = current_byte; //checksum calculation start!
                parse_data_len = current_byte; //save bytes number, use to check where is command end
                if( (parse_data_len+1) == length )
                    parse_state = RX_DECODE_CMD_DATA; //next state
                else
                    parse_result = false;           //length is not correct
                break;

            case RX_DECODE_CMD_DATA:
                parse_checksum += current_byte;
                parse_data_len--;
                if (parse_data_len == 0)                   //no data remained?
                    parse_state = RX_DECODE_CMD_CHECKSUM; //yes, next mode: checksum
                break;

            case RX_DECODE_CMD_CHECKSUM:
                if ((uint8_t)(parse_checksum + current_byte) == 0) {
                    parse_ended = true;
                } else {
                    parse_result = false;           //checksum is not correct
                }
                break;
            default:
                break;
        }

        if (parse_ended) {
            break;          //command parse is ended
        }
        if(parse_result == false) {
            break;          //command parse is terminated because error is found
        }
    }
    return parse_result;
}
#endif

void BT_CommandDecode( void )
{
    uint8_t spp_type, cmdInfo = CMD_INFO_IGNORE;
    uint16_t spp_total_length;
    uint16_t spp_payload_length;
    uint16_t para;

    // Send Ack to 5506 for the report event
    if (BT_CmdBuffer[0] == ACK)
    {
		cmdInfo = BT_UpdateAckStatusWhenReceived(BT_CmdBuffer[1],BT_CmdBuffer[2]);    
    }
	else
    {
        BT_SendAckToEvent(BT_CmdBuffer[0]);     //send ACK to this event
    }
    // Send all report event to SPP/BLE link except the SPP/BLE report event
/*    if (!((BT_CmdBuffer[0] == REPORT_SPP_DATA) ||
		  ((BT_CmdBuffer[0] == ACK) && 
		  (BT_CmdBuffer[1] == SEND_SPP_DATA) || (cmdInfo != CMD_INFO_APPS))
		  ))
*/
	if(BT_CmdBuffer[0] != REPORT_SPP_DATA)
    {
		if((BT_CmdBuffer[0] == ACK && cmdInfo != CMD_INFO_APPS) ||
			(BT_CmdBuffer[0] == ACK && BT_CmdBuffer[1] == SEND_SPP_DATA))
		;
		else
		{
	    	// All event passed to Audio Weidget 2.0
			if(BT_isBLEConnected())
				BT_SendSPPAllData(BT_CmdBuffer, (uint16_t)BT_CmdDecodeCmdLength, BT_linkIndex, (0-BT_CmdDecodeChecksum));
		}
    }
	
	if(BT_CmdBuffer[0] == REPORT_nSPK_VENDOR_EVENT &&
			BT_CmdBuffer[4] == APPS_SET_INDIV_MMI_ACTION)
	{
		if(BT_CmpBDAddress(&BT_CmdBuffer[5]))
		{
			BT_Send_ActionCommand(BT_CmdBuffer[11], 0, CMD_INFO_MCU);
		}
		return;		
	}

	//printf("\n\rBT_CmdBuffer = %02x\r\n",  BT_CmdBuffer[0]);

    switch(BT_CmdBuffer[0])
    {

		case REPORT_nSPK_VENDOR_EVENT:
			BTAPP_EventHandler(NSPK_EVENT_VENDOR_CMD, 0, &BT_CmdBuffer[1]);			
			break;
        case UART_VERSION:
            break;
		case BATTERY_LEVEL:
			//printf("\n\BATTERY_LEVEL = %02x\r\n",  BT_CmdBuffer[1]);
			if (BT_CmdBuffer[1] == 0 || BT_CmdBuffer[1] == 1) {
/*
				vs_set_volume(77, 77);
				BT_generate_specific_tone (0x8c); 		//VP_BATTERY_L
				SET_VOL_timer_count = 0;
*/
				blink_color =RED;
				Blink_Count = 3;
				LED_Blink_ON_timer_count = 0;
				
			}
			else if (BT_CmdBuffer[1] == 2 && BAT_Sound_Flag == 1) {
/*
				vs_set_volume(77, 77);
				BT_generate_specific_tone (0x8d); 		//VP_BATTERY_M
				SET_VOL_timer_count = 0;
*/

				blink_color =YELLOW;
				Blink_Count = 3;
				LED_Blink_ON_timer_count = 0;
				
			}
			else if (BAT_Sound_Flag == 1) { 
/*
				vs_set_volume(77, 77);
				BT_generate_specific_tone (0x8e); 		//VP_BATTERY_H
				SET_VOL_timer_count = 0;
*/

				blink_color =GREEN;
				Blink_Count = 3;
				LED_Blink_ON_timer_count = 0;
				
			}
			
/*
		BAT_Sound_Flag = false;  // test 20230629
*/


/*
			//vs_set_volume(Master_Volume, Master_Volume);
			BTAPP_TaskReq(BT_REQ_SYSTEM_OFF);
*/			
			
			BLE_SaveBatteryLevel(BT_CmdBuffer[2]);
            break;

		case REPORT_Vendor_EEPROM_Data:  // 20081125
			//BT_EEPROM_Data(BT_CmdBuffer[1]);
			//BT_EEPROM_Data(BT_CmdBuffer[2]);
            break;

        case DEVICE_STATE:
            switch(BT_CmdBuffer[1])
            {
                //power on, power off, standby etc.
                case BT_OFF:        //power off state
                    BTAPP_EventHandler(BT_EVENT_SYS_POWER_OFF, 0, &BT_CmdBuffer[1]);
                    break;
                case BT_ON:         //power on state
                    BTAPP_EventHandler(BT_EVENT_SYS_POWER_ON, 0, &BT_CmdBuffer[1]);
                    break;
                case BT_STANDBY:     //standby state
                    BTAPP_EventHandler(BT_EVENT_SYS_STANDBY, 0, &BT_CmdBuffer[1]);
                    break;

                //pairing etc
                case BT_DISCOVERABLE:       //pairing state
                    BTAPP_EventHandler(BT_EVENT_SYS_PAIRING_START, 0, &BT_CmdBuffer[1]);
                    break;
                case BT_PAIRING_COMPLETE:       //pairing ok
                    BTAPP_EventHandler(BT_EVENT_SYS_PAIRING_OK, 0, &BT_CmdBuffer[1]);
                    break;
                case BT_PAIRING_FAIL:           //pairing failed
                    BTAPP_EventHandler(BT_EVENT_SYS_PAIRING_FAILED, 0, &BT_CmdBuffer[1]);
                    break;

                    //HFP
                case BT_HFP_CONNECTED:
                    BT_linkIndex = BT_CmdBuffer[2]&0x0f;
                    BTAPP_EventHandler(BT_EVENT_HFP_CONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_HFP_DISCONNECTED:
                    BTAPP_EventHandler(BT_EVENT_HFP_DISCONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                    
                    //spp,iap etc
                case BT_SPP_CONNECTED:
                    BTAPP_EventHandler(BT_EVENT_SPP_CONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_iAP_CONNECTED:
                    BTAPP_EventHandler(BT_EVENT_IAP_CONNETED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_SPP_DISCONNECTED:
                    BTAPP_EventHandler(BT_EVENT_SPP_IAP_DISCONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;

                    //a2dp etc
                case BT_A2DP_CONNECTED:
                    BT_linkIndex = BT_CmdBuffer[2]&0x0f;
                    BTAPP_EventHandler(BT_EVENT_A2DP_CONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_A2DP_DISCONNECTED:
                    BTAPP_EventHandler(BT_EVENT_A2DP_DISCONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;

                    //avrcp etc
                case BT_ARVCP_CONNECTED:
                    BTAPP_EventHandler(BT_EVENT_AVRCP_CONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_AVRCP_DISCONNECTED:
                    BTAPP_EventHandler(BT_EVENT_AVRCP_DISCONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;

                    //ACL, SCO etc
                case BT_ACL_CONNECTED:
                    BTAPP_EventHandler(BT_EVENT_ACL_CONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_ACL_DISCONNECTED:
                    BTAPP_EventHandler(BT_EVENT_ACL_DISCONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_SCO_CONNECTED:
                    BTAPP_EventHandler(BT_EVENT_SCO_CONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_SCO_DISCONNECTED:
                    BTAPP_EventHandler(BT_EVENT_SCO_DISCONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;

                case BT_MAP_CONNECTED:
                    BTAPP_EventHandler(BT_EVENT_MAP_CONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BT_MAP_DISCONNECTED:
                    BTAPP_EventHandler(BT_EVENT_MAP_DISCONNECTED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;

                case BT_MAP_FORBIDDEN:
                    break;    
            }
			BLE_UpdateBTMState();
            break;

        case BTM_UTILITY_REQ:
            
            switch(BT_CmdBuffer[1])
            {
                case BTM_CONTROL_AMP:
                    break;
                case BTM_REPORT_LINE_IN_STATUS:
                    BTAPP_EventHandler(BT_EVENT_LINE_IN_STATUS,(uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case BTM_NOTIFY_HANDLE_BTM:
                    break;
                case BTM_NOTIFY_EEPROM_UPDATE_FINISH:
                    break;
                case BTM_REPORT_CODEC_STATUS:
                    BTAPP_EventHandler(BT_EVENT_A2DP_STATUS,(uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case NSPK_SYNC_POWER_OFF:
                    BTAPP_EventHandler(NSPK_EVENT_SYNC_POWER_OFF,0, &BT_CmdBuffer[1]);
                    break;
                case NSPK_SYNC_VOLUME_CTRL:
                    BTAPP_EventHandler(NSPK_EVENT_SYNC_VOL_CTRL,(uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case NSPK_SYNC_INTERNAL_GAIN:
                    BTAPP_EventHandler(NSPK_EVENT_SYNC_INTERNAL_GAIN,(uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case NSPK_SYNC_A2DP_ABS_VOL:
                    BTAPP_EventHandler(NSPK_EVENT_SYNC_ABS_VOL,(uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case NSPK_SYNC_CURRENT_CHANNEL_SETTING:
                    BTAPP_EventHandler(NSPK_EVENT_CHANNEL_SETTING,(uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
                case NSPK_NOTIFY_POWER_SYNCED:
                    break;
                case NSPK_NOTIFY_COMMAND_SUCCESS:
                    break;
                case NSPK_NOTIFY_COMMAND_FAIL:
                    break;
                case NSPK_NOTIFY_SLAVE_STATUS_CHANGED:
                    break;
                case NSPK_NOTIFY_ADD_THIRD_SPEAKER:
                    BTAPP_EventHandler(NSPK_EVENT_ADD_SPEAKER3,(uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
                    break;
            }
            break;
        
        case CALL_STATUS:
            
            BTAPP_EventHandler(BT_EVENT_CALL_STATUS_CHANGED, (uint16_t)BT_CmdBuffer[2], &BT_CmdBuffer[1]);
            break;
            
        case REPORT_nSPK_STATUS:
            
            para = BT_CmdBuffer[1];
            para <<= 8;
            para |= BT_CmdBuffer[2];
            BTAPP_EventHandler(BT_EVENT_NSPK_STATUS, para, &BT_CmdBuffer[1]);
            break;
            
        case REPORT_LINK_BACK_STATUS:
            
            if(BT_CmdBuffer[1] == 0 )       //ACL link-back
            {
                if(BT_CmdBuffer[2] == 0xFF)     //00 FF: ACL failed
                {
                    BTAPP_EventHandler(BT_EVENT_LINKBACK_FAILED, 0, &BT_CmdBuffer[1]);
                }
                else
                {
                    BTAPP_EventHandler(BT_EVENT_LINKBACK_SUCCESS, 0, &BT_CmdBuffer[1]);
                }
            }
            else if(BT_CmdBuffer[1] == 1 || BT_CmdBuffer[1] ==  2 || BT_CmdBuffer[1] == 3)  //01 00 or 02 00 or 03 00: HFP/A2DP/SPP success
            {
                if(BT_CmdBuffer[2] == 0x00)
                {
                    BTAPP_EventHandler(BT_EVENT_LINKBACK_SUCCESS, 0, &BT_CmdBuffer[1]);
                }
                else
                {
                    BTAPP_EventHandler(BT_EVENT_LINKBACK_FAILED, 0, &BT_CmdBuffer[1]);
                }
            }
            else
            {
                
            }
            break;
        
        case REPORT_LOCAL_BD_ADDR:
            BT_SaveLocalBDAddress(&BT_CmdBuffer[1]);
			BLE_SaveLocalAddr(&BT_CmdBuffer[1]);
            BTAPP_EventHandler(BT_EVENT_BD_ADDR_RECEIVED, 0, &BT_CmdBuffer[1]);
            break;
		case REPORT_LOCAL_DEVICE_NAME:
            BT_SaveDeviceName(&BT_CmdBuffer[1]);
            BTAPP_EventHandler(BT_EVENT_NAME_RECEIVED, 0, &BT_CmdBuffer[1]);
			break;
		case REPORT_FEATURE_LIST:
            BT_SaveFeatureList(BT_CmdBuffer[1]);
            BTAPP_EventHandler(FEATURE_LIST_REPLY, 0, &BT_CmdBuffer[1]);
			break;
        case REPORT_PAIRING_RECORD:
            BTAPP_EventHandler(BT_EVENT_PAIR_RECORD_RECEIVED, (uint16_t)(BT_CmdBuffer[1]), &BT_CmdBuffer[1]);
            break;

        case VOLUME_LEVEL:              //0x0f command: HFP Gain Level
            
            BTAPP_EventHandler(BT_EVENT_HFP_VOLUME_CHANGED, (uint16_t)(BT_CmdBuffer[2]), &BT_CmdBuffer[1]);
            break;
            
        case REPORT_AVRCP_VOL_CTRL:     //0x26 command
            
            BTAPP_EventHandler(BT_EVENT_AVRCP_VOLUME_CTRL, (uint16_t)(BT_CmdBuffer[2]), &BT_CmdBuffer[1]);
            break;
            
        case REPORT_AVRCP_ABS_VOL_CTRL: //0x29 command
            
            BTAPP_EventHandler(BT_EVENT_AVRCP_ABS_VOLUME_CHANGED, (uint16_t)(BT_CmdBuffer[2]), &BT_CmdBuffer[1]);
            break;

        case AVRCP_SPEC_RSP:
            
            //BT_CmdBuffer[0] = AVRCP_SPEC_RSP
            //BT_CmdBuffer[1] = database 0 or 1 link
            //BT_CmdBuffer[2] = 0x0F or 0x0D(INTRIM or CHANGED)
            //BT_CmdBuffer[3] = 0x48 fixed(Subunit_type + Subunit_ID)
            //BT_CmdBuffer[4] = 0x00 fixed(Opcode)
            //BT_CmdBuffer[5 ~ 7] = 00 19 58 fixed(company ID
            //BT_CmdBuffer[8] = 0x31 fixed(PDU ID)
            //BT_CmdBuffer[9] = 0x00 fixed(packet ID)
            //BT_CmdBuffer[10 - 11] = parameter length
            //BT_CmdBuffer[12] = EventID2 (EVENT_PLAYBACK_STATUS_CHANGED (0x01), EVENT_VOLUME_CHANGED (0x0d))
            //BT_CmdBuffer[13] = PLAYING STATUS
            if((BT_CmdBuffer[2] == 0x0D || BT_CmdBuffer[2] == 0x0F) &&
				BT_CmdBuffer[12] == 0x01)
			    //EVENT_PLAYBACK_STATUS_CHANGED (0x01)
            {
                BTAPP_EventHandler(BT_EVENT_PLAYBACK_STATUS_CHANGED, (uint16_t)(BT_CmdBuffer[13]), &BT_CmdBuffer[1]);
            }
            break;
                        
        case REPORT_SPP_DATA:
 
            spp_type = BT_CmdBuffer[2];

            spp_total_length = (uint16_t)BT_CmdBuffer[3];
            spp_total_length <<= 8;
            spp_total_length |= (uint16_t)BT_CmdBuffer[4];

            spp_payload_length = (uint16_t)BT_CmdBuffer[5];
            spp_payload_length <<= 8;
            spp_payload_length |= (uint16_t)BT_CmdBuffer[6];

#ifdef DATABASE2_TEST       //SPP carray another complete command
			if(BT_CmdBuffer[10] == MOBILE_APP_MCU_PROTOCOL)
			{	// Todo : need to define protocol for Mobile control MCU
//				BT_SetMultiSpkEQRoute(BT_CmdBuffer[11]);
				AppsCommandDecode(BT_CmdBuffer+11);				
				return;
			}
			if(BT_CmdBuffer[10] == MCU_SEND_EVENT_ACK)
			{
				// MCU has been acked the 5506 event before parsing to Mobile Apps.
				// Here MCU block any "event ack" from Mobile Apps.
				return;
			}
			if(BT_CmdBuffer[10] == 0x30)
			{
				// Group EQ
				if(BT_SendMultiSpkEQBytes(&BT_CmdBuffer[12], 84))
				{
					BT_SendAppsAck(BT_CmdBuffer[10], STS_OK);				
					return;
				}
			}			
/*			
			else if(BT_CmdBuffer[10] == EQ_MODE_SETTING)
			{
				// Send the eq mode to all slaves
				BT_Vendor_SendEqMode(BT_CmdBuffer[11]);
				// No "return" or "break" to send eqmode to Master.
			}
*/
			else if(BT_CmdBuffer[10] == 0x35)
			{
				// Rename device
				if(BT_CmdBuffer[13] >0 && BT_CmdBuffer[13] <=16)
				{
					BLE_SaveDeviceName(&BT_CmdBuffer[13]);
					BLE_forceUpdate();
				}
			}
			else if(BT_CmdBuffer[10] == MMI_CMD && BT_CmdBuffer[11] == 0x00)
			{
				switch(BT_CmdBuffer[12])
				{
					// Power on off				
					case 0x51:
						if(BTAPP_GetStatus() == BT_STATUS_OFF)
                {
						   BT_SendAppsAck(BT_CmdBuffer[10], STS_OK);
						   BTAPP_TaskReq(BT_REQ_SYSTEM_ON);
                }
                else
							BT_SendAppsAck(BT_CmdBuffer[10], ERROR_DISALLOWED);
					return;						
					case 0x53:
						if(BTAPP_GetStatus() == BT_STATUS_READY || BTAPP_GetStatus() == BT_STATUS_ON)
                {
						   BT_SendAppsAck(BT_CmdBuffer[10], STS_OK);
						   BTAPP_TaskReq(BT_REQ_SYSTEM_OFF);
                }
						else
							BT_SendAppsAck(BT_CmdBuffer[10], ERROR_DISALLOWED);
					return;
					case 0x52:
					case 0x54:
						BT_SendAppsAck(BT_CmdBuffer[10], STS_OK);
					return;
					// Volume Up Down
					case 0x30:
						BT_SendAppsAck(BT_CmdBuffer[10], STS_OK);
						BTAPP_VolUp(CMD_INFO_MCU);						
					return;
					case 0x31:
						BT_SendAppsAck(BT_CmdBuffer[10], STS_OK);
						BTAPP_VolDown(CMD_INFO_MCU);
					return;
				}
            }

			
            //BT_CmdBuffer[8] is specially designe for BLE as BLE max packet len is 20
            if (BT_CmdBuffer[8] >0 ) // byte is 1 -> fragmented start packet; byte is 2 ->  fragmented continue packet; byte is 3 -> framented end packet
                {
                BT_AddBytesToSPPBuff_Fragmented_Pkt(&BT_CmdBuffer[7], spp_payload_length, BT_CmdBuffer[8]);
                }
                else
                {
                if(ParsePayloadAsCommand(&BT_CmdBuffer[7], spp_payload_length)) {
                	BT_SendAppsCompleteCommand(&BT_CmdBuffer[7], spp_payload_length);
                }
            }
#else
            BT_SendAckToEvent(BT_CmdBuffer[0]);     //send ACK to this event
            BT_AddBytesToSPPBuff(&BT_CmdBuffer[7], spp_payload_length);
#endif
            break;
        case REPORT_TYPE_CODEC:
            BTAPP_EventHandler(BT_EVENT_TYPE_CODEC, 0, &BT_CmdBuffer[1]);
			break;
        case REPORT_CUSTOMER_GATT_ATTRIBUTE_DATA:
        
            BTAPP_EventHandler(GATT_ATTRIBUTE_DATA, (uint16_t)(BT_CmdBuffer[3]), &BT_CmdBuffer[1]);
            BT_CustomerGATT_AttributeData(BT_CmdBuffer[3], &BT_CmdBuffer[4], BT_CmdDecodeCmdLength-4);
            break;
            
        case REPORT_LINK_MODE:
            
            para = BT_CmdBuffer[2];     //second byte put to higher part
            para <<= 8;
            para |= BT_CmdBuffer[1];    //first byte put to lower part
            BTAPP_EventHandler(BT_EVENT_LINK_MODE_RECEIVED, para, &BT_CmdBuffer[1]);
            break;

        case REPORT_LE_EVENT:
            
            switch(BT_CmdBuffer[1])     //LE event sub command
            {
                case 0:
                    para = BT_CmdBuffer[2];
                    para <<= 8;
                    para |= BT_CmdBuffer[3];
                    BTAPP_EventHandler(LE_STATUS_CHANGED, para, &BT_CmdBuffer[1]);
                    break;
                case 1:
                    BTAPP_EventHandler(LE_ADV_CONTROL_REPORT, (uint16_t)(BT_CmdBuffer[2]), &BT_CmdBuffer[1]);
                    break;
                case 2:
                    BTAPP_EventHandler(LE_CONNECTION_PARA_REPORT, 0, &BT_CmdBuffer[1]);
                    break;
                case 3:
                    para = BT_CmdBuffer[2];
                    para <<= 8;
                    para |= BT_CmdBuffer[3];
                    BTAPP_EventHandler(LE_CONNECTION_PARA_UPDATE_RSP, para, &BT_CmdBuffer[1]);
                    break;
                default:
                    break;
            }
            break;
        
        case REPORT_INPUT_SIGNAL_LEVEL:
            
            BT_CmdBuffer[1] ^= 0xff;
            BT_CmdBuffer[2] ^= 0xff;
            BT_CmdBuffer[3] ^= 0xff;
            BT_CmdBuffer[4] ^= 0xff;
            if(BT_CmdBuffer[1])
            {
                para = BT_CmdBuffer[1];
                para <<= 8;
                para |= BT_CmdBuffer[5];
                BTAPP_EventHandler(PORT0_INPUT_CHANGED, para, &BT_CmdBuffer[1]);
            }
            if(BT_CmdBuffer[2])
            {
                para = BT_CmdBuffer[2];
                para <<= 8;
                para |= BT_CmdBuffer[6];
                BTAPP_EventHandler(PORT1_INPUT_CHANGED, para, &BT_CmdBuffer[1]);
            }
            if(BT_CmdBuffer[3])
            {
                para = BT_CmdBuffer[3];
                para <<= 8;
                para |= BT_CmdBuffer[7];
                BTAPP_EventHandler(PORT2_INPUT_CHANGED, para, &BT_CmdBuffer[1]);
            }
            if(BT_CmdBuffer[4])
            {
                para = BT_CmdBuffer[4];
                para <<= 8;
                para |= BT_CmdBuffer[8];
                BTAPP_EventHandler(PORT3_INPUT_CHANGED, para, &BT_CmdBuffer[1]);
            }
            break;
#ifdef PIC18_DEBUGGER			
			case 0xFE:
				BLE_SaveBatteryLevel(BT_GetOverRun(BT_CmdBuffer[0] ? 1 : 0));
			break;
#endif//PIC18_DEBUGGER
        default:
            break;
    }
}
void AppsCommandDecode(uint8_t* buffer)
{
	uint8_t payload_length;
	payload_length = (uint16_t)BT_CmdBuffer[8];
	payload_length <<= 8;
	payload_length |= (uint16_t)BT_CmdBuffer[9];
            
	switch(buffer[0])
	{
		case APPS_GET_STATUS:
			switch(buffer[1])
			{
				case APPS_GET_AUXIN_STATUS:
					BT_SendAppsAck(MOBILE_APP_MCU_PROTOCOL, STS_OK);
					AudioSelectAppsStatus();
				break;
				case APPS_GET_POWER_STATUS:
					BT_SendAppsAck(MOBILE_APP_MCU_PROTOCOL, STS_OK);
					BT_SendAppsPowerStatus(BTAPP_GetStatus() == BT_STATUS_OFF ? 0 : 2);
				break;
        default:
					BT_SendAppsAck(MOBILE_APP_MCU_PROTOCOL, ERROR_COMMAND_UNKNOW);
				break;

			}
		break;
		case APPS_SET_STATUS:
			switch(buffer[1])
			{
				case APPS_SET_AUDIO_SRC:
					BT_SendAppsAck(MOBILE_APP_MCU_PROTOCOL, STS_OK);
					BT_ToggleAudioPath();
				break;
				case APPS_SET_INDIV_MMI_ACTION:
					BT_SendAppsAck(MOBILE_APP_MCU_PROTOCOL, STS_OK);
					if(payload_length-2 == INDIV_ACTION_CMD_LENGTH && 
					   buffer[6] == MMI_CMD)
						BT_Vendor_SendCommand(&buffer[1] ,INDIV_ACTION_CMD_LENGTH);
				break;
				default:
					BT_SendAppsAck(MOBILE_APP_MCU_PROTOCOL, ERROR_COMMAND_UNKNOW);
				break;
			}
		break;
		default:
			BT_SendAppsAck(MOBILE_APP_MCU_PROTOCOL, ERROR_COMMAND_UNKNOW);
            break;
    }
}
void BT_CommandDecode1MS_event( void )
{
}

