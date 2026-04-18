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
#include <stdbool.h>
#include <stdint.h>
//#include <xc.h>
//#include "mcc/mcc.h"
#include "ble.h"
#include "bt_command_send.h"
#include "bt_command_decode.h"
//#include "sha1.h"
#include "bt_app.h"

//#define MSPK_ANDROID_APP	1

enum {
    BLE_ADV_TASK_IDLE,
    BLE_ADV_STOP,
    BLE_ADV_UPDATE_RESPONSE_DATA,
	BLE_ADV_ADV_INTERVAL,

    BLE_SET_ADV_TYPE,
    BLE_ADV_START,
    BLE_ADV_START_WAITING,
} BLE_advTaskState;
enum
{
	BLE_BTM_OFF=0,
	BLE_BTM_PAIR,
	BLE_BTM_STBY,
	BLE_BTM_CONN_HF,
	BLE_BTM_CONN_A2DP,
	BLE_BTM_CONN_SPP,
	BLE_BTM_CONN_ALL
};
BLE_ADV_TYPE BLE_advType;
bool BLE_advUpdateReq;
bool BLE_scoActive;
uint16_t BLE_timer1ms = 0xffff; // Init number will not decrement
uint8_t BT_deviceName[16+1];
uint8_t BT_LocalAddr[6];
uint8_t BT_MasterAddr[6];
uint8_t BT_groupLink;
uint8_t BT_groupstatus;
uint8_t BT_batteryLevel;
uint8_t BT_btmState;
uint8_t BT_feature_0;
/*------------------------------------------------------------*/
void BLE_SaveMasterAddr(uint8_t * address)
{
	uint8_t i;
	for(i=0;i<6;i++)
		BT_MasterAddr[i] = address[i];
}
void BLE_SaveLocalAddr(uint8_t * address)
{
	uint8_t i;
	for(i=0;i<6;i++)
		BT_LocalAddr[i] = address[i];
}
void BT_SaveFeatureList(uint8_t feature0)
{
	BT_feature_0 = feature0;

}
bool BT_CmpBDAddress(uint8_t* address)
{
	uint8_t i;
	for(i=0; i<4; i++)
		if(BT_LocalAddr[i] != address[i])
			break;
	if(i == 4)
		return true;
	return false;
}

void BLE_SaveDeviceName(uint8_t* name)
{
    uint8_t i;
	uint8_t len = *name;
	if(len >=17)
		len = 17;
	
    for(i=0; i<len; i++)
		BT_deviceName[i] = *name++;
}
/*------------------------------------------------------------*/
void BLE_advertiser_Init(void)
{
	BT_groupstatus = 0;
	BT_batteryLevel = 0xff;
	BT_feature_0= 0;
	BLE_scoActive = 0;

}
/*------------------------------------------------------------*/
void BLE_advUpdateLinkStatus(BT_GROUP_LINK group_link)
{
	if(group_link >LINK_CONNECTED && group_link <LINK_MORE_SLAVES)
		return;
	
    if(BT_groupLink != group_link)
    {
		BT_groupLink = group_link;
        BLE_advUpdateReq = true;
    }
}

/*------------------------------------------------------------*/
void BLE_advUpdateGroupStatus(BT_GROUP_STATUS group_status)
{
    if(BT_groupstatus != group_status)
    {
		BT_groupstatus = group_status;
        //if(BLE_advTaskState > BLE_ADV_UPDATE_RESPONSE_DATA || BLE_advTaskState == BLE_ADV_TASK_IDLE)
            BLE_advUpdateReq = true;
    }
}
/*------------------------------------------------------------*/
void BLE_EnableAdvertising(bool enable)
{
    uint8_t command[7];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = LE_SIGNALING;      //command ID
    command[4] = 0x01;      //subCommand 0x01: Enable advertising or disable advertising
    command[5] = enable;
    chksum = command[2] + command[3] + command[4] + command[5];
    chksum = ~chksum + 1;
    command[6] = chksum;
    BT_SendBytesAsCompleteCommand(&command[0], 7);
}

void BLE_SetAdvertisingType(void)
{
    uint8_t command[7];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = LE_SIGNALING;      //command ID
    command[4] = 0x04;      //subCommand 0x04: set advertising type
    command[5] = BLE_advType;
    chksum = command[2] + command[3] + command[4] + command[5];
    chksum = ~chksum + 1;
    command[6] = chksum;
    BT_SendBytesAsCompleteCommand(&command[0], 7);
}
void BLE_SetAdvInterval(void)
{
    uint8_t command[10];
    uint8_t chksum;

    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x04;      //length
    command[3] = LE_SIGNALING;      //command ID
    command[4] = 0x03;      //subCommand 0x04: set advertising type
    command[5] = 0x05;
    command[6] = 0xDC;

    chksum = command[2] + command[3] + command[4] + command[5] + command[6];
    chksum = ~chksum + 1;
    command[7] = chksum;
    BT_SendBytesAsCompleteCommand(&command[0], 8);

}
void BLE_advUpdateBTconnectable(BT_CONNECTABLE_STATUS connectable)
{

}

#ifdef MSPK_ANDROID_APP
void BLE_SetSlaveAdvData(void)
{
    uint8_t command[50];
    uint8_t chksum;
    uint8_t i;
	uint8_t* ptr = command;

    *ptr++ = 0xAA;      //header byte 0
    *ptr++ = 0x00;      //header byte 1
    *ptr++ = 3+3+13+(2+BT_deviceName[0]);      //length

    *ptr++ = LE_SIGNALING;      //command ID
    *ptr++ = 0x05;      //subCommand 0x06: set response data
    *ptr++ = 0x00;

	*ptr++ = 0x02;
	*ptr++ = 0x01;
	*ptr++ = 0x18;
	
    *ptr++ = 0x0C;
    *ptr++ = 0xff;		// Manufacturing data
    *ptr++ = 'M';      //command ID
    *ptr++ = 'C';      //subCommand 0x06: set response data
    *ptr++ = 'H';
    *ptr++ = 'P';   
    *ptr++ = BT_groupstatus;	
	for(i=0;i<6;i++)
		*ptr++ = BT_MasterAddr[i];

	*ptr++ = 1+BT_deviceName[0];	// header + XXXXX
	*ptr++ = 0x09;
	for(i=0; i< BT_deviceName[0]; i++)
		*ptr++ = BT_deviceName[1+i];
	
	chksum = 0;
	for(i=2; i<24+BT_deviceName[0]; i++)
		chksum += command[i];
	chksum = ~chksum + 1;

	*ptr = chksum;

    BT_SendBytesAsCompleteCommand(&command[0], 24 + BT_deviceName[0]+1);

}



void BLE_SetMasterAdvData(void)
{
    uint8_t command[40];
    uint8_t chksum;
    uint8_t i;
	uint8_t* ptr = command;

    *ptr++ = 0xAA;      //header byte 0
    *ptr++ = 0x00;      //header byte 1
    *ptr++ = 3+3+7+(2+BT_deviceName[0]);      //length

    *ptr++ = LE_SIGNALING;      //command ID
    *ptr++ = 0x05;      //subCommand 0x06: set response data
    *ptr++ = 0x00;

	*ptr++ = 0x02;
	*ptr++ = 0x01;
	*ptr++ = 0x18;

    *ptr++ = 0x06;
    *ptr++ = 0xff;		// Manufacturing data
    *ptr++ = 'M';      //command ID
    *ptr++ = 'C';      //subCommand 0x06: set response data
    *ptr++ = 'H';
    *ptr++ = 'P';   
    *ptr++ = BT_groupstatus;		

	*ptr++ = 1+BT_deviceName[0];	// header + "LE_" + XXXXX
	*ptr++ = 0x09;
	for(i=0; i< BT_deviceName[0]; i++)
		*ptr++ = BT_deviceName[1+i];
	
	chksum = 0;
	for(i=2; i<18+BT_deviceName[0]; i++)
		chksum += command[i];
	chksum = ~chksum + 1;

	*ptr = chksum;

    BT_SendBytesAsCompleteCommand(&command[0], 18 + BT_deviceName[0]+1);

}

void BLE_SetGroupAdvData(void)
{
	if(BT_groupstatus == SPEAKER_BROADCAST_SLAVE ||
	   BT_groupstatus == SPEAKER_nSPK_SLAVE)
	   BLE_SetSlaveAdvData();
	else
	   BLE_SetMasterAdvData();
}
#endif//MSPK_ANDROID_APP
void BLE_SaveBatteryLevel(uint8_t level)
{
	if(BT_batteryLevel == level)
		return;
	BT_batteryLevel = level;
	BLE_advUpdateReq = true;
}
void BLE_UpdateBTMState(void)
{
	BT_SYS_STATUS tmpSysStatus = BT_GetSysStatus();
	uint8_t tmpLinkStatus = BT_GetLinkStatus();
	uint8_t tmpBtmState=0, profiles=0;
	bool tmpScoActive=BLE_scoActive;

	if(tmpLinkStatus & BIT_HFP_LINK_STATUS)
	{
		tmpBtmState = BLE_BTM_CONN_HF;
		profiles++;
	}
	if(tmpLinkStatus & BIT_A2DP_LINK_STATUS)
	{
		tmpBtmState = BLE_BTM_CONN_A2DP;
		profiles++;
	}
	if(tmpLinkStatus & BIT_SPP_LINK_STATUS)
	{
		tmpBtmState = BLE_BTM_CONN_SPP;		
		profiles++;
	}
	if(tmpLinkStatus & BIT_SCO_LINK_STATUS)
		tmpScoActive = 1;
	else
		tmpScoActive = 0;

	if((tmpScoActive != BLE_scoActive) && BT_isBLEConnected())
	{	// If BLE is connected, whatever SCO changes, no immediate action taken place
		BLE_scoActive = tmpScoActive;
	}
	
	if(profiles == 0)
	{
		switch(tmpSysStatus)
		{
				case BT_SYSTEM_INIT:
				case BT_SYSTEM_POWER_OFF:
					tmpBtmState = BLE_BTM_OFF;
				break;
				case BT_SYSTEM_POWER_ON: 	
				case BT_SYSTEM_STANDBY:
				case BT_SYSTEM_CONNECTED:
					tmpBtmState = BLE_BTM_STBY;
				break;
				case BT_SYSTEM_PAIRING:
					tmpBtmState = BLE_BTM_PAIR;
				break;
                
		}
	}
	else if(profiles > 1)
	{
		tmpBtmState = BLE_BTM_CONN_ALL;
	}
	if(tmpBtmState == BT_btmState && tmpScoActive==BLE_scoActive)
		return;

	BT_btmState = tmpBtmState;
	BLE_scoActive = tmpScoActive;
	BLE_advUpdateReq = true;	
}
void BLE_SetAdvData(void)
{
	uint8_t command[50];
	uint8_t chksum;
	uint8_t i;
	uint8_t* ptr = command;

	*ptr++ = 0xAA;		//header byte 0
	*ptr++ = 0x00;		//header byte 1
//	*ptr++ = 3+3+(4+12);//Category 0 data format
	*ptr++ = 3+3+(4+17);//Category 1 data format


	// Command header
	*ptr++ = LE_SIGNALING;		//command ID
	*ptr++ = 0x05;		//subCommand 0x06: set response data
	*ptr++ = 0x00;

	// AD type : Flag
	*ptr++ = 0x02;
	*ptr++ = 0x01;
	*ptr++ = 0x1A;

	// AD type : Service Data
//	*ptr++ = 0x0f;	     // Categroy 0
	*ptr++ = 0x14;		 // Categroy 1
	*ptr++ = 0x16;	   //command ID
	*ptr++ = 0xDA;	   //ISSC UUID
	*ptr++ = 0xFE;	   //ISSC UUID
	
//	*ptr++ = (0 << 4) | (BT_batteryLevel & 0x0f);			//Category 0 data format
	*ptr++ = (1 << 4) | (BT_batteryLevel & 0x0f);	//Category 1 data format
	*ptr++ = BT_btmState;
	*ptr++ = BT_groupstatus;
	*ptr++ = BT_groupLink;
	for(i=0; i<6; i++)
		*ptr++ = BT_LocalAddr[i];
	if(BT_groupstatus == SPEAKER_SINGLE)	
	{
		for(i=0; i<6; i++)
			*ptr++ = 0;
	}
	else if((BT_groupstatus == SPEAKER_nSPK_MASTER) ||
			(BT_groupstatus == SPEAKER_BROADCAST_MASTER_ADDING)||
			(BT_groupstatus == SPEAKER_BROADCAST_MASTER))
	{
		for(i=0; i<6; i++)
			*ptr++ = BT_LocalAddr[i];
	}
	else if((BT_groupstatus == SPEAKER_nSPK_SLAVE) ||
			(BT_groupstatus == SPEAKER_BROADCAST_SLAVE))	
	{
		for(i=0; i<6; i++)
			*ptr++ = BT_MasterAddr[i];
	}		
	*ptr++ = BT_feature_0;

/*
	// AD type : Name
	*ptr++ = 1+BT_deviceName[0];	
	*ptr++ = 0x09;
	for(i=0; i< BT_deviceName[0]; i++)
		*ptr++ = BT_deviceName[1+i];
*/	
	chksum = 0;
//	for(i=2; i<27+BT_deviceName[0]; i++)
	for(i=2; i<30; i++)
		chksum += command[i];
	chksum = ~chksum + 1;

	*ptr = chksum;

	BT_SendBytesAsCompleteCommand(&command[0], 30+1);

}

void BLE_SetResponseData(void)
{
    uint8_t command[50];
    uint8_t chksum;
    uint8_t i,size;
	uint8_t* ptr = command;
    
    *ptr++ = 0xAA;      //header byte 0
    *ptr++ = 0x00;      //header byte 1
    *ptr++ = 3 + 2 + BT_deviceName[0];      //length

    *ptr++ = LE_SIGNALING;      //command ID
    *ptr++ = 0x06;      //subCommand 0x06: set response data
    *ptr++ = 00;

	// AD type : Name
	*ptr++ = 1+BT_deviceName[0];	
	*ptr++ = 0x09;
	for(i=0; i< BT_deviceName[0]; i++)
		*ptr++ = BT_deviceName[1+i];

	chksum = 0 ;
	for(i=2; i<8+BT_deviceName[0]; i++)
		chksum += command[i];
	chksum = ~chksum + 1;

	*ptr = chksum;

	BT_SendBytesAsCompleteCommand(&command[0], 8 + BT_deviceName[0] + 1);

}
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
void BLE_advertiserUpdateTask( void )
{
	uint8_t ackState;

	if(BLE_advUpdateReq == true)
    {
        BLE_advUpdateReq = false;
		if(BLE_scoActive)
		{
			BLE_EnableAdvertising(false);
			BLE_advTaskState = BLE_ADV_TASK_IDLE;
		}
		else		
        	BLE_advTaskState = BLE_ADV_STOP;
    }

	if(BLE_advTaskState >= BLE_ADV_STOP)
	{
		if(BT_IsCommandSendTaskIdle()==false)
			return;
		
	    switch(BLE_advTaskState)
	    {
	        case BLE_ADV_STOP:
	                BLE_EnableAdvertising(false);
	                BLE_advTaskState = BLE_ADV_UPDATE_RESPONSE_DATA;//BLE_ADV_STOP_WAITING;
	            break;
	        case BLE_ADV_UPDATE_RESPONSE_DATA:
			#ifdef MSPK_ANDROID_APP
				BLE_SetGroupAdvData();
			#else//MSPK_ANDROID_APP
				BLE_SetAdvData();
			#endif//MSPK_ANDROID_APP
					BLE_advTaskState = BLE_ADV_ADV_INTERVAL;//BLE_SET_ADV_TYPE;
	            break;
			case BLE_ADV_ADV_INTERVAL:
					BLE_SetResponseData();
				BLE_advTaskState = BLE_SET_ADV_TYPE;//BLE_ADV_UPDATE_DATA_WAITING;
				break;
	        case BLE_SET_ADV_TYPE:
	                BLE_SetAdvertisingType();
	                BLE_advTaskState = BLE_ADV_START;//BLE_SET_ADV_TYPE_WAITING;
	            break;
	        case BLE_ADV_START:
	                BLE_EnableAdvertising(true);
	                BLE_advTaskState = BLE_ADV_START_WAITING;
	            break;
	        case BLE_ADV_START_WAITING:
	                BLE_advTaskState = BLE_ADV_TASK_IDLE;
	            break;
	        default:
	            break;
	    }
	}
}
/*------------------------------------------------------------*/
void BLE_advTaskInit(void)
{
    BLE_advertiser_Init();
    BLE_advUpdateReq = false;
    BLE_advTaskState = BLE_ADV_TASK_IDLE;
	BT_btmState = BLE_BTM_OFF;
}
/*------------------------------------------------------------*/
void BLE_UpdateAdvType(BLE_ADV_TYPE type)
{
    //if(type != BLE_advType)
    //{
        BLE_advType = type;
       // if(BLE_advTaskState > BLE_ADV_UPDATE_RESPONSE_DATA || BLE_advTaskState == BLE_ADV_TASK_IDLE)
        BLE_advUpdateReq = true;
    //}
}
/*------------------------------------------------------------*/
void BLE_advUpdateRequest( void )
{
    BLE_advUpdateReq = true;
}
/*------------------------------------------------------------*/
bool BLE_advUpdateIsEnd( void )
{
    if(BLE_advUpdateReq == true)
        return false;
    if(BLE_advTaskState == BLE_ADV_TASK_IDLE)
        return true;
    else
        return false;
}
/*------------------------------------------------------------*/
//================================================
//1ms Timer
//================================================
void BLE_reAdvConnectable(void)
{
	BLE_timer1ms = 500;
}
void BLE_Timer1MS_event( void )
{
	if(BLE_timer1ms != 0xffff && BLE_timer1ms>0)
		--BLE_timer1ms;
}
bool BLE_isTaskRunning(void)
{
	return ((BLE_advTaskState != BLE_ADV_TASK_IDLE) || BLE_advUpdateReq);
}
void BLE_forceUpdate(void)
{
	BLE_advUpdateReq = true;
}

