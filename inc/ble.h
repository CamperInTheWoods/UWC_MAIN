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
#ifndef BT_BLE_H
#define BT_BLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SPEAKER_SINGLE = 0,
    SPEAKER_nSPK_MASTER = 1,
    SPEAKER_nSPK_SLAVE = 4,
    SPEAKER_BROADCAST_MASTER_ADDING = 5,
    SPEAKER_BROADCAST_SLAVE = 6,
	SPEAKER_BROADCAST_MASTER = 7
} BT_GROUP_STATUS;

typedef enum {
    LINK_STBY = 0,
    LINK_BUSY = 1,
    LINK_CONNECTING = 2,
    LINK_CONNECTED = 3,
	LINK_MORE_SLAVES = 9,
} BT_GROUP_LINK;


typedef enum {
    MASTER_ROLE = 0,
            PLAYER_ROLE = 1
} BT_GROUP_ROLE;

typedef enum {
    STEREO = 0,
            LEFT = 1,
            RIGHT = 2,
            MONAURAL = 3
} BT_GROUP_OUTPUT_CHANNEL;

typedef enum {
    BT_CONNECTABLE = 0,
            BT_NON_CONNECTABLE = 1
}BT_CONNECTABLE_STATUS;

typedef enum {
    CONNECTABLE_UNDIRECT_ADV = 0,           //connectable
    CONNECTABLE_RESERVED = 1,
    SCANNABLE_UNDIRECT_ADV = 2,
    NON_CONNECTABLE_UNDIRECT_ADV = 3,       //non-connectable
} BLE_ADV_TYPE;

void BLE_advertiser_Init(void);
void BLE_advUpdateLocalUniqueID(uint8_t* BT_address);
void BLE_advUpdateMasterUniqueID(uint8_t* BT_address);
void BLE_advUpdateNumOfPlayer(uint8_t num);
void BLE_advUpdateLinkStatus(BT_GROUP_LINK group_link);
void BLE_advUpdateGroupStatus(BT_GROUP_STATUS group_status);
void BLE_advUpdateRoleInGroup(BT_GROUP_ROLE role);
void BLE_advUpdateOutputChannel(BT_GROUP_OUTPUT_CHANNEL channel);
void BLE_advUpdateBTconnectable(BT_CONNECTABLE_STATUS connectable);

void BLE_advertiserUpdateTask( void );
void BLE_advTaskInit(void);
void BLE_UpdateAdvType(BLE_ADV_TYPE type);
void BLE_advUpdateRequest( void );          //basically don't use it
bool BLE_advUpdateIsEnd( void );

void BLE_Timer1MS_event( void );
void BLE_reAdvConnectable(void);
void BLE_SetAdvData(void);
void BLE_SaveDeviceName(uint8_t* name);
void BLE_SaveLocalAddr(uint8_t * address);
void BLE_SaveMasterAddr(uint8_t * address);
void BLE_forceUpdate(void);
bool BLE_isTaskRunning(void);
void BLE_UpdateBTMState(void);
void BLE_SaveBatteryLevel(uint8_t level);
bool BT_CmpBDAddress(uint8_t* address);
void BT_SaveFeatureList(uint8_t feature0);

#endif
