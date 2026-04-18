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
#include "bt_command_send.h"
#include "bt_command_decode.h"
#include "em_gpio.h"
#include "bt_app.h"
#include "HA300_BSP.h"

#define ACK_TIME_OUT_MS                 1000
#define APP_INPUT_WAITING_TIME_OUT_MS   100
#define INTERVAL_AFTER_ACK_CMD          20
#define QUEQUED_CMD_MAX                 20

static struct {
    uint8_t SendingCmdNum;
    struct{
        uint16_t startBufPt;
        uint16_t endBufPt;
        uint16_t cmdSize;
        uint8_t cmdID;
		uint8_t cmdInfo;
        CMD_PROCESSING_STATUS cmdStatus;
    } SendingCmdArray[QUEQUED_CMD_MAX];
} BT_SendingCmd;

#define UR_TX_BUF_SIZE              400
static uint8_t          UR_TxBuf[UR_TX_BUF_SIZE];
static uint16_t         UR_TxBufHead;
static uint16_t         UR_TxBufTail;
static uint16_t         UR_TxBufTail2;
typedef enum {
	TXRX_BUF_EMPTY,
	TXRX_BUF_OK,
	TXRX_BUF_FULL
} TXRX_BUF_STATUS;
static TXRX_BUF_STATUS  UR_TxBufStatus;

typedef enum {
    BT_CMD_SEND_STATE_IDLE = 0,         //no data in the queue, MFB is low
    BT_CMD_SEND_MFB_HIGH_WAITING,       //set MFB to high, and wait for 2ms
    BT_CMD_SEND_DATA_SENDING,           //UART interface is working
    //BT_CMD_SEND_DATA_SENT_WAITING,      //wait for another 10ms after all data are sent out     ...//no longer need
    BT_CMD_SEND_ACK_WAITING,        //new...waiting for ack
    BT_CMD_SEND_ACK_ERROR,
    BT_CMD_SEND_ACK_OK,
} BT_CMD_SEND_STATE;
BT_CMD_SEND_STATE BT_CMD_SendState;
uint16_t BT_CommandSendTimer;//BT_CommandStartMFBWaitTimer;
uint16_t BT_commandOverRun=0;
uint16_t BT_bufferOverRun=0;

static bool copyCommandToBuffer(uint8_t* data, uint16_t size, uint8_t cmdInfo);
static bool StartRegisterNewCommand(uint16_t start_index, uint16_t cmd_size, uint8_t cmd_id, uint8_t cmd_info);
static bool EndRegisterNewCommand(uint16_t end_index);
static bool RemoveFirstCommand(void);
/*======================================*/
/*  function implemention  */
/*======================================*/
void BT_GiveUpThisCommand( void )
{
    RemoveFirstCommand();
}
/*------------------------------------------------------------*/

/*------------------------------------------------------------*/
bool copySendingCommandToBuffer(uint8_t* data, uint16_t size)
{
	copyCommandToBuffer(data, size, CMD_INFO_MCU);
}
static bool copyCommandToBuffer(uint8_t* data, uint16_t size, uint8_t cmdInfo)
{
    bool buf_result = true;
    uint8_t ur_tx_buf_status_save = UR_TxBufStatus;
    uint16_t ur_tx_buf_head_save = UR_TxBufHead;


	tx_data_printf(data, size);

    while(size--)
    {
		USART1_Tx(USART1, *data++);
    }

#if 0
	if(UR_TxBufHead > UR_TxBufTail)
	{
		if(UR_TxBufHead - UR_TxBufTail	+ size >= UR_TX_BUF_SIZE)
		{
			BT_bufferOverRun++;
			return false;		
		}
	}	
	else if(UR_TxBufHead < UR_TxBufTail)
	{
		if(UR_TxBufHead + size >=  UR_TxBufTail)
		{
			BT_bufferOverRun++;
			return false;
		}
	}

    if(UR_TxBufStatus !=  TXRX_BUF_FULL)
    {
        if(!StartRegisterNewCommand(UR_TxBufHead, size, data[3], cmdInfo))
        {
            return false;
        }
        
        while(size--)
        {
            UR_TxBuf[UR_TxBufHead++] = *data++;

            if(UR_TxBufHead >= UR_TX_BUF_SIZE)
                UR_TxBufHead = 0;

            if(UR_TxBufHead ==  UR_TxBufTail)
            {
                if(size)
                {
                    buf_result = false;
                    UR_TxBufStatus = ur_tx_buf_status_save;		//restore in this case
                    UR_TxBufHead = ur_tx_buf_head_save;                 //restore in this case
                }
                else
                {
                    UR_TxBufStatus =  TXRX_BUF_FULL;            //test this code
                }
                break;
            }
            else
            {
                UR_TxBufStatus = TXRX_BUF_OK;
            }
        }

        if(buf_result)
        {
            EndRegisterNewCommand(UR_TxBufHead);
            //StartToSendCommand();
        }
    }
    else
    {
        buf_result = false;
    }
#endif

    return buf_result;
}

static bool checkCopySendingBuffer(uint8_t cmdID, uint16_t size, uint8_t cmdInfo)
{
	if(UR_TxBufHead > UR_TxBufTail)
	{
		if(UR_TxBufHead - UR_TxBufTail	+ size >= UR_TX_BUF_SIZE)
		{
			BT_bufferOverRun++;
			return false;		
		}
	}	
	else if(UR_TxBufHead < UR_TxBufTail)
	{
		if(UR_TxBufHead + size >=  UR_TxBufTail)
		{
			BT_bufferOverRun++;
			return false;
		}
	}
	if(!StartRegisterNewCommand(UR_TxBufHead, size, cmdID, cmdInfo))
	{
		return false;
	}
	return true;
}



extern void tx_data_printf(uint8_t* data, uint16_t size);

/*------------------------------------------------------------*/
static bool continueCopySendingCommandToBuffer(uint8_t* data, uint16_t size)
{
    bool buf_result = true;
    uint8_t ur_tx_buf_status_save = UR_TxBufStatus;
    uint16_t ur_tx_buf_head_save = UR_TxBufHead;

	tx_data_printf(data, size);

    while(size--)
    {
		USART1_Tx(USART1, *data++);
    }
    return buf_result;

#if 0
    if(UR_TxBufStatus !=  TXRX_BUF_FULL)
    {
        while(size--)
        {
            UR_TxBuf[UR_TxBufHead++] = *data++;

            if(UR_TxBufHead >= UR_TX_BUF_SIZE)
                UR_TxBufHead = 0;

            if(UR_TxBufHead ==  UR_TxBufTail)
            {
                if(size)
                {
                    buf_result = false;
                    UR_TxBufStatus = ur_tx_buf_status_save;		//restore in this case
                    UR_TxBufHead = ur_tx_buf_head_save;                 //restore in this case
                }
                else
                {
                    UR_TxBufStatus =  TXRX_BUF_FULL;            //test this code
                }
                break;
            }
            else
            {
                UR_TxBufStatus = TXRX_BUF_OK;
            }
        }
    }
    else
    {
        buf_result = false;
    }
    return buf_result;
#endif	
}

/*------------------------------------------------------------*/
static uint8_t calculateChecksum(uint8_t* startByte, uint8_t* endByte)
{
    uint8_t checksum = 0;
    while(startByte <= endByte)
    {
        checksum += *startByte;
        startByte++;
    }
    checksum = ~checksum + 1;
    return checksum;
}

static uint8_t calculateChecksum2(uint8_t checksum, uint8_t* startByte, uint16_t length)
{
    while(length)
    {
        checksum += *startByte++;
        length--;
    }
    return checksum;
}

/*------------------------------------------------------------*/
#if 0
uint8_t BT_IsAllowedToSendCommand( void )       //no longer need
{
    uint16_t i, size = sizeof(CommandAckStatus);
    uint8_t* p = &CommandAckStatus.MMI_ACTION_status;
    for(i=0; i<size; i++)
    {
        if(*p == COMMAND_IS_SENT)
            return false;
        p++;
    }
    return true;
}
#endif
uint8_t BT_IsCommandSendTaskIdle( void )
{
    if(BT_CMD_SendState == BT_CMD_SEND_STATE_IDLE)
        return true;
    else
        return false;
}
/*------------------------------------------------------------*/

void BT_SendAppsCompleteCommand(uint8_t* command, uint8_t command_length)
{
    copyCommandToBuffer(command, command_length, CMD_INFO_APPS);
}
void BT_SendBytesAsCompleteCommand(uint8_t* command, uint8_t command_length)
{
    copySendingCommandToBuffer(command, command_length);
   // BT_UpdateAckStatusWhenSent(command[3]);
}

/*------------------------------------------------------------*/
void BT_MMI_ActionCommand(uint8_t MMI_ActionCode, uint8_t link_index)
{
	BT_Send_ActionCommand(MMI_ActionCode, link_index, CMD_INFO_MCU);
}
void BT_Send_ActionCommand(uint8_t MMI_ActionCode, uint8_t link_index, uint8_t cmd_info)
{
    uint8_t command[8];
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = MMI_CMD;      //command ID
    command[4] = link_index;      //link_index, set to 0
    command[5] = MMI_ActionCode;
    command[6] = calculateChecksum(&command[2], &command[5]);
	copyCommandToBuffer(&command[0], 7, cmd_info);
    //CommandAckStatus.MMI_ACTION_status = COMMAND_IS_SENT;
}
void BT_Send_EQCmd(uint8_t* eqData,uint8_t size)
{
	if(size >= 90)
	eqData[89] = calculateChecksum(&eqData[2], &eqData[88]);
	copySendingCommandToBuffer(eqData, 90);
}

void BT_Send_EQMode(uint8_t eqMode)
{
    uint8_t command[7];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x03;                      //length
    command[3] = EQ_MODE_SETTING;        	//command ID
    command[4] = eqMode;                 	//
	command[5] = 0;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
}

void BT_Vendor_SendEqMode(uint8_t eqMode)
{
	uint8_t command[10];
	command[0] = NSPK_EVENT_EQ_MODE;
	command[1] = 0;
	command[2] = eqMode;
	BT_Vendor_SendCommand(command, 3);
}

void BT_Vendor_SendVol(uint8_t event, uint8_t updn)
{
	uint8_t command[10];
	command[0] = event;
	command[1] = 0;
	command[2] = updn;
	BT_Vendor_SendCommand(command, 3);
}
void BT_Vendor_SendVolInfo(uint8_t volMode, uint8_t vol)
{
	uint8_t command[10];
	command[0] = NSPK_EVENT_VOL_MODE;
	command[1] = volMode;
	command[2] = vol;
	BT_Vendor_SendCommand(command, 3);
}


void BT_Vendor_SendCommand(uint8_t* data, uint8_t size)
{

	uint8_t command[20], i;
	command[0] = 0xAA;	 		//header byte 0
	command[1] = 0x00;	 		//header byte 1
	command[2] = 4 + size;	 	//length
	command[3] = 0x2A;			//command ID
	command[4] = 0x03;	   		//broadcast to all slaves
	command[5] = 0x00;			// vendor data length
	command[6] = size;			// vendor data length
	for(i=0; i <size; i++)
		*((command+7)+i) = *(data+i);

	command[7+size] = calculateChecksum(&command[2], &command[7+size-1]);
	copySendingCommandToBuffer(&command[0], 7+size+1);

}


void BT_Vendor_EQCmd(uint8_t* eqData, uint8_t sequence, uint8_t size)
{
	uint8_t command[17], i;
	command[0] = 0xAA;	 		//header byte 0
	command[1] = 0x00;	 		//header byte 1
	command[2] = 4 + size;	 	//length
	command[3] = 0x2A;			//command ID
	command[4] = 0x03;	   		//broadcast to all slaves
	command[5] = 0x00;			// vendor data length
	command[6] = size;			// vendor data length
	command[7] = sequence | 0x80;		// sequence number with 0x80
	for(i=0; i <size-1; i++)
		command[i+8] = eqData[(sequence*(VENDOR_CMD_SIZE-1))+i];

	command[7+(size-1)+1] = calculateChecksum(&command[2], &command[7+(size-1)]);
	copyCommandToBuffer(&command[0], 7+(size-1)+2, CMD_INFO_MCU);
}

/*------------------------------------------------------------*/
void BT_MusicControlCommand(uint8_t CtrlCode)
{
    uint8_t command[8];
    command[0] = 0xAA;      //header byte 0
    command[1] = 0x00;      //header byte 1
    command[2] = 0x03;      //length
    command[3] = MUSIC_CONTROL;      //command ID
    command[4] = 0x00;      //link_index, set to 0
    command[5] = CtrlCode;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    //CommandAckStatus.MUSIC_CTRL_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_SendAckToEvent(uint8_t ack_event)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = MCU_SEND_EVENT_ACK;        //command ID
    command[4] = ack_event;                 //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
}

/*------------------------------------------------------------*/
void BT_SendDiscoverableCommand(uint8_t discoverable)
{
    uint8_t command[6];
    command[0] = 0xAA;
    command[1] = 0x00;
    command[2] = 0x02;
    command[3] = DISCOVERABLE;
    command[4] = discoverable;      //0: disable, 1: enable
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    //CommandAckStatus.DISCOVERABLE_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_ReadBTMLinkModeCommand( void )
{
    uint8_t command[6];
    command[0] = 0xAA;
    command[1] = 0x00;
    command[2] = 0x02;
    command[3] = READ_LINK_MODE;
    command[4] = 0;         //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
   // CommandAckStatus.READ_LINK_MODE_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_GroupZoneWrite(uint8_t Group_main, uint8_t Group_sub) //20081125
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x06;                      //length
    //command[3] = Read_Vendor_EEPROM_Data;    //command ID
    command[3] = 0x35;    //command ID
    command[4] = 02;                      //offset
    command[5] = 00;                      //Length
    command[6] = 02;    //command ID
    command[7] = Group_main;                      //offset
    command[8] = Group_sub;                      //Length
    command[9] = calculateChecksum(&command[2], &command[8]);
    copySendingCommandToBuffer(&command[0], 10);

}

/*------------------------------------------------------------*/
void BT_Read_BTM_Batt_CHG_Status(void) 
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = READ_BTM_BATTERY_CHARGETR_STATUS;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
}


 /*------------------------------------------------------------*/
void BT_ReadDeviceAddressCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = READ_LOCAL_BD_ADDR;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
   // CommandAckStatus.READ_BD_ADDRESS_status = COMMAND_IS_SENT;
}
void BT_ReadNSpkStatus(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = READ_NSPK_LINK_STATUS;    //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);

}
void BT_ReadDeviceNameCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = READ_LOCAL_DEVICE_NAME;    //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
  //  CommandAckStatus.READ_Device_Name_status = COMMAND_IS_SENT;

}

void BT_ReadFeatureListCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = READ_FEATURE_LIST;    //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
  //  CommandAckStatus.READ_Device_Name_status = COMMAND_IS_SENT;

}


void BT_DSP_noiseReductionCtrlCommand(uint8_t Cmd_Type)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DSP_NR_CTRL;    //command ID
    command[4] = Cmd_Type;                      //ENABLE_SPK_NR 
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
  //  CommandAckStatus.READ_Device_Name_status = COMMAND_IS_SENT;

}



/*------------------------------------------------------------*/
void BT_GetPairRecordCommand(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = READ_PAIRING_RECORD;         //command ID
    command[4] = 0x00;                      //dummy byte
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    //CommandAckStatus.READ_PAIR_RECORD_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_LinkBackToLastDevice(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = PROFILE_LINK_BACK;         //command ID
    command[4] = 0x00;                      //0x00: last device, 0x01: last HFP device, 0x02: last A2DP device
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
   // CommandAckStatus.LINK_BACK_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_LinkBackMultipoint(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = PROFILE_LINK_BACK;         //command ID
    command[4] = 0x06;                      //multipoint devices
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
//    CommandAckStatus.LINK_BACK_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_LinkBackToDeviceByBTAddress(uint8_t* address)
{
    uint8_t command[14];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 10;                      //length
    command[3] = PROFILE_LINK_BACK;         //command ID
    command[4] = 0x05;              //0x05: link back to device with specified address
    command[5] = 0x00;
    command[6] = 0x07;
    command[7] = *address++;        //byte 0
    command[8] = *address++;        //byte 1
    command[9] = *address++;        //byte 2
    command[10] = *address++;        //byte 3
    command[11] = *address++;        //byte 4
    command[12] = *address++;        //byte 5
    command[13] = calculateChecksum(&command[2], &command[12]);
    copySendingCommandToBuffer(&command[0], 14);
   // CommandAckStatus.LINK_BACK_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_DisconnectAllProfile(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DISCONNECT;                //command ID
    command[4] = 0x0f;                      //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
   // CommandAckStatus.DISCONNECT_PROFILE_status = COMMAND_IS_SENT;
}
/*------------------------------------------------------------*/
void BT_DisconnectSPPProfile(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DISCONNECT;                //command ID
    command[4] = 0x08;                      //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    //CommandAckStatus.DISCONNECT_PROFILE_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_DisconnectHFPProfile(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DISCONNECT;                //command ID
    command[4] = 0x02;                      //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    //CommandAckStatus.DISCONNECT_PROFILE_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_DisconnectA2DPProfile(void)
{
    uint8_t command[6];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x02;                      //length
    command[3] = DISCONNECT;                //command ID
    command[4] = 0x04;                      //event to ack
    command[5] = calculateChecksum(&command[2], &command[4]);
    copySendingCommandToBuffer(&command[0], 6);
    //CommandAckStatus.DISCONNECT_PROFILE_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_SetOverallGainCommand(uint8_t set_type, uint8_t gain1, uint8_t gain2, uint8_t gain3)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[3] = SET_OVERALL_GAIN;                //command ID
    command[4] = 0x00;                      //link index
    command[5] = 0x00;                      //mask bits
    command[6] = set_type;                      //type
    if(set_type == 1 || set_type == 2)
    {
        command[2] = 0x04;                    //length
        command[7] = calculateChecksum(&command[2], &command[6]);
        copySendingCommandToBuffer(&command[0], 8);
    }
    else if(set_type == 3)
    {
        command[2] = 0x07;                    //length
        command[7] = gain1&0x0f;
        command[8] = gain2&0x0f;
        command[9] = gain3&0x0f;
        command[10] = calculateChecksum(&command[2], &command[9]);
        copySendingCommandToBuffer(&command[0], 11);
    }
    else
    {
        command[2] = 0x07;                    //lengthcommand[2] = 0x07;                    //length
        command[7] = gain1&0x7f;
        command[8] = gain2&0x7f;
        command[9] = gain3&0x7f;
        command[10] = calculateChecksum(&command[2], &command[9]);
        copySendingCommandToBuffer(&command[0], 11);
    }
  //  CommandAckStatus.SET_OVERALL_GAIN_status = COMMAND_IS_SENT;
}
/*------------------------------------------------------------*/
void BT_SetOverallGain(uint8_t gain1, uint8_t gain2, uint8_t gain3)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07; //lengthcommand[2] = 0x07;                    //length
    command[3] = SET_OVERALL_GAIN;                //command ID
    command[4] = 0;                      //link index
    command[5] = 0x00;                      //mask bits
    command[6] = 0x05;                      //type
    command[7] = gain1 & 0x7f;
    command[8] = gain2 & 0x7f;
    command[9] = gain3 & 0x7f;
    command[10] = calculateChecksum(&command[2], &command[9]);
    copySendingCommandToBuffer(&command[0], 11);
  //  CommandAckStatus.SET_OVERALL_GAIN_status = COMMAND_IS_SENT;
}
/*------------------------------------------------------------*/
void BT_updateA2DPGain(uint8_t gain, uint8_t cmdInfo)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07; //lengthcommand[2] = 0x07;                    //length
    command[3] = SET_OVERALL_GAIN;                //command ID
    command[4] = 0;                      //link index
    command[5] = 0x01;                      //mask bits
    command[6] = 0x04;                      //type
    command[7] = gain & 0x7f;
    command[8] = 0;
    command[9] = 0;
    command[10] = calculateChecksum(&command[2], &command[9]);
	copyCommandToBuffer(&command[0], 11, cmdInfo);
//    copySendingCommandToBuffer(&command[0], 11);
    
 //   CommandAckStatus.SET_OVERALL_GAIN_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_updateHFPGain(uint8_t gain, uint8_t cmdInfo)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07;                    //length
    command[3] = SET_OVERALL_GAIN;                //command ID
    command[4] = 0;                      //link index
    command[5] = 0x02;                      //mask bits
    command[6] = 0x03;                      //type
    command[7] = 0;
    command[8] = gain & 0x0f;
    command[9] = 0;
    command[10] = calculateChecksum(&command[2], &command[9]);
	copyCommandToBuffer(&command[0], 11, cmdInfo);
//	copySendingCommandToBuffer(&command[0], 11);
    
   // CommandAckStatus.SET_OVERALL_GAIN_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_updateLineInGain(uint8_t gain, uint8_t cmdInfo)
{
    uint8_t command[11];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x07;                    //length
    command[3] = SET_OVERALL_GAIN;                //command ID
    command[4] = 0;                      //link index
    command[5] = 0x04;                      //mask bits
    command[6] = 0x03;                      //type
    command[7] = 0;
    command[8] = 0;
    command[9] = gain & 0x0f;
    command[10] = calculateChecksum(&command[2], &command[9]);
	copyCommandToBuffer(&command[0], 11, cmdInfo);
//    copySendingCommandToBuffer(&command[0], 11);
    //CommandAckStatus.SET_OVERALL_GAIN_status = COMMAND_IS_SENT;
}



void BT_Send_Personal_MSPK_GROUP_Control(uint8_t eqMode)
{
    uint8_t command[12];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 0x03;                      //length
    command[3] = 0x3A;        	//command ID  Personal_MSPK_GROUP_Control
    command[4] = 0x01;                 	// 0x00 :  Disable this feature, 0x01 :  Enable this feature
    command[5] = 0x00;
    command[6] = 0x11;
    command[7] = 0x22;
    command[8] = 0x33;
    command[9] = 0x44;
    command[10] = 0x55;
    command[11] = calculateChecksum(&command[2], &command[10]);
    copySendingCommandToBuffer(&command[0], 11);
}


/*------------------------------------------------------------*/
void BT_SendAppsAck(uint8_t cmd_id, uint8_t status)
{
	uint8_t command [10];
	command[0] = 0xAA;
	command[1] = 0x00;
	command[2] = 0x03;
	command[3] = 0x00;
	command[4] = cmd_id;
	command[5] = status;
	command[6] = calculateChecksum(&command[2], &command[5]);
	BT_SendSPPData(command, 7, 0);
}
void BT_SendAppsPowerStatus(uint8_t status)
{
	uint8_t command [10];
	command[0] = 0xAA;
	command[1] = 0x00;
	command[2] = 0x02;
	command[3] = 0x01;
	command[4] = status;
	command[5] = calculateChecksum(&command[2], &command[4]);
	BT_SendSPPData(command, 6, 0);
}
void BT_SendAppBTMStatus(uint8_t status)
{
	uint8_t command [10];
	command[0] = 0xAA;
	command[1] = 0x00;
	command[2] = 0x02;
	command[3] = 0x01;
	command[4] = status;
	command[5] = calculateChecksum(&command[2], &command[4]);
	BT_SendSPPData(command, 6, 0);
	
}
void BT_SendSPPData(uint8_t* addr, uint16_t size, uint8_t link_index)
{
    uint8_t command[11];
    uint8_t checksum = 0;
    uint16_t cmd_length = size + 7;
	// Send data if BLE connect
	// Check if the buffer is full
	// If buffer is OK, it will add an item on  BT_SendingCmd

#if 0  // test jaehoon
    if(BT_isBLEConnected()==0 || checkCopySendingBuffer(SEND_SPP_DATA, 10+size+1, CMD_INFO_MCU) == false)
		return;
#endif
    command[0] = 0xAA;                      //header byte 0
    command[1] = (uint8_t)(cmd_length>>8);                 //header byte 1
    command[2] = (uint8_t)(cmd_length&0xff);                      //length
    command[3] = SEND_SPP_DATA;             //command ID
    command[4] = link_index;                      //link_index, set to 0
    command[5] = 0x00;          //single packet format
    //total_length: 2byte
    command[6] = (uint8_t)(size>>8);
    command[7]= (uint8_t)(size&0xff);
    //payload_length: 2byte
    command[8] = (uint8_t)(size>>8);
    command[9] = (uint8_t)(size&0xff);


    continueCopySendingCommandToBuffer(&command[0], 10);
    checksum = calculateChecksum2(checksum, &command[1], 9);
    continueCopySendingCommandToBuffer(addr, size);
    	checksum = calculateChecksum2(checksum, addr, size);
    	checksum = ~checksum + 1;
    continueCopySendingCommandToBuffer(&checksum, 1);
	EndRegisterNewCommand(UR_TxBufHead);
  //  CommandAckStatus.SPP_DATA_status = COMMAND_IS_SENT;
}


/*------------------------------------------------------------*/
// Send BT SPP data with command header
void BT_SendSPPAllData(uint8_t* addr, uint16_t size, uint8_t link_index, uint8_t all_data_checksum)
{
    uint8_t command[13];
    uint8_t checksum = 0;
    uint16_t total_data_size = size + 4;
    uint16_t cmd_length = total_data_size + 7;

	if(BT_isBLEConnected()==0 || checkCopySendingBuffer(SEND_SPP_DATA, 13+size+1+1, CMD_INFO_MCU) == false)
		return;
	
    command[0] = 0xAA;                      //header byte 0
    command[1] = (uint8_t)(cmd_length>>8);                 //header byte 1
    command[2] = (uint8_t)(cmd_length&0xff);                      //length
    command[3] = SEND_SPP_DATA;             //command ID
    command[4] = link_index;                      //link_index, set to 0
    command[5] = 0x00;          //single packet format
    //total_length: 2byte
    command[6] = (uint8_t)(total_data_size>>8);
    command[7]= (uint8_t)(total_data_size&0xff);
    //payload_length: 2byte
    command[8] = (uint8_t)(total_data_size>>8);
    command[9] = (uint8_t)(total_data_size&0xff);
    command[10] = 0xAA;
    command[11] = 0x00;
    command[12] = size;
    continueCopySendingCommandToBuffer(&command[0], 13);
    checksum = calculateChecksum2(checksum, &command[1], 12);
    continueCopySendingCommandToBuffer(addr, size);
    checksum = calculateChecksum2(checksum, addr, size);
    continueCopySendingCommandToBuffer(&all_data_checksum, 1);
    command[0] = all_data_checksum;
    checksum = calculateChecksum2(checksum, &command[0], 1);
    checksum = ~checksum + 1;
    continueCopySendingCommandToBuffer(&checksum, 1);
	EndRegisterNewCommand(UR_TxBufHead);
   // CommandAckStatus.SPP_DATA_status = COMMAND_IS_SENT;
}

void BT_LoopBackSPPData(uint8_t* addr, uint16_t total_command_size)
{
    uint8_t command[11];
    uint8_t checksum = 0;
    uint16_t cmd_length = total_command_size+1;

	if(BT_isBLEConnected()==0 || checkCopySendingBuffer(SEND_SPP_DATA, 4+total_command_size+1, CMD_INFO_MCU) == false)
		return;

	
    command[0] = 0xAA;                                  //header byte 0
    command[1] = (uint8_t)(cmd_length>>8);                 //header byte 1(length high byte)
    command[2] = (uint8_t)(cmd_length&0xff);                      //length(length low byte)
    command[3] = SEND_SPP_DATA;             //command ID
    continueCopySendingCommandToBuffer(&command[0], 4);
    checksum = calculateChecksum2(checksum, &command[1], 3);
    //total_command_size -= 1;
    continueCopySendingCommandToBuffer(addr, total_command_size);
    checksum = calculateChecksum2(checksum, addr, total_command_size);
    checksum = ~checksum + 1;
    continueCopySendingCommandToBuffer(&checksum, 1);
	EndRegisterNewCommand(UR_TxBufHead);
   // CommandAckStatus.SPP_DATA_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_SetupBTMGPIO( void )
{
    uint8_t command[20];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 13;                        //length
    command[3] = GPIO_CTRL;                //command ID
    command[4] = 0xFF;                      //IO_Ctrl_Mask_P0,
    command[5] = 0xDF;                      //IO_Ctrl_Mask_P1,
    command[6] = 0xFF;                      //IO_Ctrl_Mask_P2,
    command[7] = 0xBF;                      //IO_Ctrl_Mask_P3,
    command[8] = 0x00;                      //IO_Setting_P0,
    command[9] = 0x00;                      //IO_Setting_P1,
    command[10] = 0x00;                     //IO_Setting_P2,
    command[11] = 0x00;                     //IO_Setting_P3,
    command[12] = 0x00;                     //Output_Value_P0,
    command[13] = 0x00;                     //Output_Value_P1,
    command[14] = 0x00;                     //Output_Value_P2,
    command[15] = 0x00;                     //Output_Value_P3,
    command[16] = calculateChecksum(&command[2], &command[15]);
    copySendingCommandToBuffer(&command[0], 17);
  //  CommandAckStatus.SET_GPIO_CTRL_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_generate_specific_tone (uint8_t ToneParameter )  // jaehoon
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 3;                        //length
    command[3] = BTM_UTILITY_FUNCTION;                //command ID
    command[4] = 2;                         // generate the specific tone 
    command[5] = ToneParameter;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
}

/*------------------------------------------------------------*/
void BT_EnterLineInMode(uint8_t disable0_enable1, uint8_t analog0_I2S1)
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 3;                        //length
    command[3] = BTM_UTILITY_FUNCTION;                //command ID
    command[4] = 1;                         //utility_function_type=0x01
    if(analog0_I2S1)
    {
        if(disable0_enable1)
            command[5] = 3;
        else
            command[5] = 2;
    }
    else
    {
        if(disable0_enable1)
            command[5] = 1;
        else
            command[5] = 0;
    }
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    //CommandAckStatus.BTM_UTILITY_REQ_status = COMMAND_IS_SENT;
}

/*------------------------------------------------------------*/
void BT_SetRXBufferSize( void )
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 3;                        //length
    command[3] = BT_MCU_UART_RX_BUFF_SIZE;                //command ID
    command[4] = 0;
    command[5] = 200;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    //CommandAckStatus.SET_RX_BUFFER_SIZE_status = COMMAND_IS_SENT;
}
void BT_ProfileLinkBack(uint8_t linked_profile, uint8_t link_index)
{
    uint8_t command[10];
    command[0] = 0xAA;                      //header byte 0
    command[1] = 0x00;                      //header byte 1
    command[2] = 3;                        //length
    command[3] = ADDITIONAL_PROFILE_LINK_SETUP;                //command ID
    command[4] = link_index;
    command[5] = linked_profile;
    command[6] = calculateChecksum(&command[2], &command[5]);
    copySendingCommandToBuffer(&command[0], 7);
    //CommandAckStatus.PROFILE_LINK_BACK_status = COMMAND_IS_SENT;
}
/*------------------------------------------------------------*/
void BT_CommandSendInit( void )
{
    UR_TxBufHead = 0;
    UR_TxBufTail = 0;
    UR_TxBufStatus = TXRX_BUF_EMPTY;
    BT_SendingCmd.SendingCmdNum = 0;
}

/*------------------------------------------------------------*/
void BT_CommandSendTask( void )
{
    switch(BT_CMD_SendState)
    {
        case BT_CMD_SEND_STATE_IDLE:
            if(BT_SendingCmd.SendingCmdNum)
            {
                //BM6X_MFB_SetHigh();
                BT_CommandSendTimer = 3;      //wait 2 - 3ms
                BT_CMD_SendState = BT_CMD_SEND_MFB_HIGH_WAITING;
            }
            else
//            	SET_BT_POWERKEY(0);

            break;
            
        case BT_CMD_SEND_MFB_HIGH_WAITING:
            if(!BT_CommandSendTimer)
            {
                UART_TransferFirstByte();
                BT_SendingCmd.SendingCmdArray[0].cmdStatus = IN_SENDING;
                BT_CMD_SendState = BT_CMD_SEND_DATA_SENDING;
            }
            break;

        case BT_CMD_SEND_DATA_SENDING:
            //do nothing, UART interrupt will handle
            break;

        case BT_CMD_SEND_ACK_WAITING:       //new
            if(!BT_CommandSendTimer)
            {
                BTAPP_EventHandler(BT_EVENT_CMD_SENT_NO_ACK, (uint16_t)(BT_SendingCmd.SendingCmdArray[0].cmdID),  0);        //send event to user application layer with command id, and event type
                BT_SendingCmd.SendingCmdArray[0].cmdStatus = ERROR_NO_ACK;
                BT_CommandSendTimer = APP_INPUT_WAITING_TIME_OUT_MS;
                BT_CMD_SendState = BT_CMD_SEND_ACK_ERROR;
            }
            break;
        
        case BT_CMD_SEND_ACK_ERROR:
            if (!BT_CommandSendTimer) {
                if (BT_SendingCmd.SendingCmdNum) {
                    UART_TransferFirstByte();
                    BT_SendingCmd.SendingCmdArray[0].cmdStatus = IN_SENDING;
                    BT_CMD_SendState = BT_CMD_SEND_DATA_SENDING; //re-sending last command or sending next command
                } else {
                    //BM6X_MFB_SetLow();
                BT_CMD_SendState = BT_CMD_SEND_STATE_IDLE;
            }
            }
            break;

        case BT_CMD_SEND_ACK_OK:
            if (!BT_CommandSendTimer) {
                RemoveFirstCommand();
                if (BT_SendingCmd.SendingCmdNum) {
                    UART_TransferFirstByte();
                    BT_SendingCmd.SendingCmdArray[0].cmdStatus = IN_SENDING;
                    BT_CMD_SendState = BT_CMD_SEND_DATA_SENDING;
                } else {
                    //BM6X_MFB_SetLow();
                    BT_CMD_SendState = BT_CMD_SEND_STATE_IDLE;
                }
            }
            break;

        default:
            break;
    }
}

/*------------------------------------------------------------*/
uint8_t BT_UpdateAckStatusWhenReceived(uint8_t command_id, uint8_t ack_status)
{
    uint16_t temp;
    if( BT_CMD_SendState == BT_CMD_SEND_ACK_WAITING )
{
        BTAPP_EventHandler(BT_EVENT_CMD_SENT_ACK_OK, (uint16_t)command_id, 0);        //send event to user application layer with command id, and event type
        if(command_id == BT_SendingCmd.SendingCmdArray[0].cmdID)
            BT_SendingCmd.SendingCmdArray[0].cmdStatus = ack_status;
		// Here we can know what is the ACK status 5506 reply to the previous sent command.
        BT_CommandSendTimer = APP_INPUT_WAITING_TIME_OUT_MS;
        BT_CMD_SendState = BT_CMD_SEND_ACK_OK;
		return BT_SendingCmd.SendingCmdArray[0].cmdInfo;
    }
	return CMD_INFO_IGNORE;
}

/*------------------------------------------------------------*/
void BT_CommandSend1MS_event(void)
    {
    if( BT_CommandSendTimer/*BT_CommandStartMFBWaitTimer*/)
            {
        -- BT_CommandSendTimer/*BT_CommandStartMFBWaitTimer*/;
    }
}

/*------------------------------------------------------------*/
void UART_TransferFirstByte( void )
{
    uint8_t data;
    UR_TxBufTail2 = BT_SendingCmd.SendingCmdArray[0].startBufPt;
    data = UR_TxBuf[UR_TxBufTail2++];
    if(UR_TxBufTail2 >= UR_TX_BUF_SIZE)
            UR_TxBufTail2 = 0;
        EUSART_Write(data);
    }

/*------------------------------------------------------------*/
void UART_TransferNextByte( void )
{
    uint8_t data;
    if(UR_TxBufTail2 == BT_SendingCmd.SendingCmdArray[0].endBufPt)
    {
        if(BT_CMD_SendState == BT_CMD_SEND_DATA_SENDING)
        {
            if(BT_SendingCmd.SendingCmdArray[0].cmdID != MCU_SEND_EVENT_ACK)
            {
                BT_CommandSendTimer =  ACK_TIME_OUT_MS;
                BT_CMD_SendState = BT_CMD_SEND_ACK_WAITING;
            }
            else        //just sent is ACK_TO_EVENT command
{
                BT_CommandSendTimer = INTERVAL_AFTER_ACK_CMD;
                BT_SendingCmd.SendingCmdArray[0].cmdStatus = STS_OK;
                BT_CMD_SendState = BT_CMD_SEND_ACK_OK;
            }
    }
}
    else
{
        data = UR_TxBuf[UR_TxBufTail2++];
        if(UR_TxBufTail2 >= UR_TX_BUF_SIZE)
            UR_TxBufTail2 = 0;
        EUSART_Write(data);
    }
}

static bool StartRegisterNewCommand(uint16_t start_index, uint16_t cmd_size, uint8_t cmd_id, uint8_t cmdInfo)
{
    if(!cmd_size)
        return false;
    
    if(BT_SendingCmd.SendingCmdNum < QUEQUED_CMD_MAX)
{
        BT_SendingCmd.SendingCmdArray[BT_SendingCmd.SendingCmdNum].startBufPt = start_index;
        BT_SendingCmd.SendingCmdArray[BT_SendingCmd.SendingCmdNum].cmdSize = cmd_size;
        BT_SendingCmd.SendingCmdArray[BT_SendingCmd.SendingCmdNum].cmdID = cmd_id;
        BT_SendingCmd.SendingCmdArray[BT_SendingCmd.SendingCmdNum].cmdInfo = cmdInfo;

        if(cmd_id!=0x1f)
            //Nop();  //jhlee 20171111
        return true;
    }
    else
    {
	    BT_commandOverRun++;
        return false;
    }
}

static bool EndRegisterNewCommand(uint16_t end_index)
{
    if(BT_SendingCmd.SendingCmdNum < QUEQUED_CMD_MAX)
    {
        BT_SendingCmd.SendingCmdArray[BT_SendingCmd.SendingCmdNum].endBufPt = end_index;
        BT_SendingCmd.SendingCmdArray[BT_SendingCmd.SendingCmdNum].cmdStatus = IN_QUEUE;
        BT_SendingCmd.SendingCmdNum++;
    }
    return true;
}

static bool RemoveFirstCommand(void)//index starting from 0
{
    uint8_t i;
    if(!BT_SendingCmd.SendingCmdNum)
        return false;            //parameter error
    
    if(BT_SendingCmd.SendingCmdNum == 1)
    {
        BT_SendingCmd.SendingCmdNum--;
        UR_TxBufTail = UR_TxBufTail2;
        if (UR_TxBufHead == UR_TxBufTail)
            UR_TxBufStatus = TXRX_BUF_EMPTY;
        else
            UR_TxBufStatus = TXRX_BUF_OK;
        return true;
    }
    
    for(i = 0; i < BT_SendingCmd.SendingCmdNum - 1; i++)
    {
        //copy next command info to previous one
        BT_SendingCmd.SendingCmdArray[i].cmdSize = BT_SendingCmd.SendingCmdArray[i + 1].cmdSize;
        BT_SendingCmd.SendingCmdArray[i].cmdStatus = BT_SendingCmd.SendingCmdArray[i + 1].cmdStatus;
        BT_SendingCmd.SendingCmdArray[i].endBufPt = BT_SendingCmd.SendingCmdArray[i + 1].endBufPt;
        BT_SendingCmd.SendingCmdArray[i].startBufPt = BT_SendingCmd.SendingCmdArray[i + 1].startBufPt;
        BT_SendingCmd.SendingCmdArray[i].cmdID = BT_SendingCmd.SendingCmdArray[i + 1].cmdID;
        BT_SendingCmd.SendingCmdArray[i].cmdInfo = BT_SendingCmd.SendingCmdArray[i + 1].cmdInfo;

    }
    BT_SendingCmd.SendingCmdNum--;
    UR_TxBufTail = UR_TxBufTail2;
    if (UR_TxBufHead == UR_TxBufTail)
        UR_TxBufStatus = TXRX_BUF_EMPTY;
    else
        UR_TxBufStatus = TXRX_BUF_OK;
    return true;
}

/*------------------------------------------------------------*/
uint8_t BT_GetAckStatusSendSPPData( void )
{
    return BT_SendingCmd.SendingCmdArray[0].cmdStatus;
}

uint16_t BT_GetOverRun(uint8_t choice)
{
	switch(choice)
	{
		case 0:
		return BT_bufferOverRun;
		case 1:
		return BT_commandOverRun;
	}
}
 
