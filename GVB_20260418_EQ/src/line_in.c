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
//#include <xc.h>
//#include "mcc/mcc.h"
#include "line_in.h"
#include "bt_app.h"
#include "bt_command_send.h"
//#include "yda174dsp_handler.h"


#define BTM_PORT_READ_INTERNAL_MS       500
#define PORT_DETECT_INTERVAL_MS        50
#define ANG_POSITIVE_VALIDATE_CNT       3
#define ANG_LOSS_VALIDATE_TIME_CNT      3
#define AUDIO_TOGGLE_INTERVAL_MS        200

uint16_t ang_audio_detect_cnt = 0;
uint8_t ang_audio_detect_input;
uint8_t audio_selected = AUDIO_A2DP;
uint8_t auxin_plugged = 0;
uint8_t audioUnMute=0;

//bool    lineInCmdSendReq = false;
enum {
	APPS_AUX_UNPLUG = 0x80,
	APPS_AUX_PLUG_AUX,
	APPS_AUX_PLUG_A2DP		
};

enum {
    BT_LINEIN_IDLE,         
    BT_LINEIN_DETECTED,    
	BT_LINEIN_MUTE,        
    BT_LINEIN_WAIT_A2DP,    
    BT_LINEIN_UNMUTE,
    BT_LINEIN_TO_APPS,
}BT_lineInStatus;        // BM64 internal system status
uint8_t    lineInCmdSendReq = 0;

uint16_t AnalogDetectTimer;
uint16_t AudioToggleTimer;

//uint16_t BTM_PortReadTimer;

void AnalogAudioDetectInit(void)
{
    AnalogDetectTimer = 10;
    audio_selected = 0;
    ang_audio_detect_input = 0;     //depend on circuit...
    AudioToggleTimer = 0;
    //BTM_PortReadTimer = 1000;
}

void AnalogAudioDetectTask(void)
{
    if(AnalogDetectTimer==0)
    {
        AnalogAudioDetect();
        AnalogDetectTimer = PORT_DETECT_INTERVAL_MS;
    }
    /*if(BTM_PortReadTimer==0)
    {
        if(BTAPP_GetStatus()==BT_STATUS_READY)
        {
            BTM_PortReadTimer = BTM_PORT_READ_INTERNAL_MS;
        }
    }*/
    if (BTAPP_GetStatus() == BT_STATUS_ON || BTAPP_GetStatus() == BT_STATUS_READY) {
        if (lineInCmdSendReq > BT_LINEIN_IDLE) 
		{
            if (BT_IsCommandSendTaskIdle()) 
            {                
            	switch(lineInCmdSendReq)
            	{
					case BT_LINEIN_DETECTED:												
						audio_selected = auxin_plugged ? AUDIO_AUXIN : AUDIO_A2DP;		

						if(!BT_isCallIdle() || (BT_GetLinkStatus() & BIT_SCO_LINK_STATUS ))
						{
							if (getI2SAuxInJumper()) //HIGH: I2S
								BT_EnterLineInMode(audio_selected, 1);
							else
								BT_EnterLineInMode(audio_selected, 0);						
							lineInCmdSendReq = BT_LINEIN_IDLE;
							break;
						}

						lineInCmdSendReq = BT_LINEIN_MUTE; 		
						AudioToggleTimer = AUDIO_TOGGLE_INTERVAL_MS;						
						
		                if(BT_isA2DPPlaying()&& audio_selected==AUDIO_AUXIN)
		                {
		                	// Mute before Pausing A2DP music
		                	if(BT_isMaster())
								BT_SendMuteInfo(1);
							//yda174_ForceMute(1);		
		                }
		                else if(BT_isA2DPPaused()&& audio_selected==AUDIO_A2DP)
		                {
		                	// Mute before Playing A2DP music
		                	if(BT_isMaster())
								BT_SendMuteInfo(1);		                	
							//yda174_ForceMute(1);		
		                }

					break;
					case BT_LINEIN_MUTE:
						if(AudioToggleTimer > 0)
							break;						
						if(BT_isA2DPPlaying()&& audio_selected==AUDIO_AUXIN)
						{
							// Pause A2DP music
							BT_MusicControlCommand(6);
						}
						else if(BT_isA2DPPaused()&& audio_selected==AUDIO_A2DP)
						{
							// Play A2DP music
							BT_MusicControlCommand(5);
						}				
						lineInCmdSendReq = BT_LINEIN_WAIT_A2DP; 										
						break;
					case BT_LINEIN_WAIT_A2DP:
	                    lineInCmdSendReq = BT_LINEIN_UNMUTE;
	                    if (getI2SAuxInJumper()) //HIGH: I2S
	                        BT_EnterLineInMode(audio_selected, 1);
	                    else
	                        BT_EnterLineInMode(audio_selected, 0);
						if(!BT_isCallIdle() || (BT_GetLinkStatus() & BIT_SCO_LINK_STATUS ))
							audioUnMute = 0;
						else
							audioUnMute = 1;
						break;

					case BT_LINEIN_UNMUTE:
						if(audioUnMute > 0)
							break;						
						BT_SendVolInfo();
						AudioToggleTimer = AUDIO_TOGGLE_INTERVAL_MS;								
						lineInCmdSendReq = BT_LINEIN_TO_APPS;
						break;
					case BT_LINEIN_TO_APPS:
						if(AudioToggleTimer > 0)
							break;
						//yda174_ForceMute(0);
						if(BT_isMaster())
							BT_SendMuteInfo(0);
						AudioSelectAppsStatus();
						lineInCmdSendReq = BT_LINEIN_IDLE;
					break;
            }
          }
       }
    }
}

void AnalogAudioDetect_Timer1MS_event()
{
    if(AnalogDetectTimer)
        --AnalogDetectTimer;
	if(AudioToggleTimer)
		--AudioToggleTimer;
    //if(BTM_PortReadTimer)
    //    --BTM_PortReadTimer;
}

extern uint8_t ang_input;   // 0 : Line In connect, 1 : Line In Disconnect

void AnalogAudioDetect(void)
{
    //uint8_t current_ang_input = LINE_IN_DETECT_GetValue();
    uint8_t current_ang_input = ang_input;  // 0 : Line In connect, 1 : Line In Disconnect

    if(current_ang_input != ang_audio_detect_input)
    {
        ang_audio_detect_cnt = 0;
        ang_audio_detect_input = current_ang_input;
    }
    else
    {
        if(ang_audio_detect_cnt <= 60000)
            ang_audio_detect_cnt ++;
    }

	if(BT_isStandalone() || BT_isMaster())
	{
	    if(auxin_plugged == 0) //current is BT selected
	    {
	        if(!ang_audio_detect_input && (ang_audio_detect_cnt > ANG_POSITIVE_VALIDATE_CNT))
	        {
	            auxin_plugged = 1;
				lineInCmdSendReq = BT_LINEIN_DETECTED;    //BTAPP_LineInEvent(1);
	        }
	    }
	    else        //current is AUX selected
	    {
	        if(ang_audio_detect_input && (ang_audio_detect_cnt > ANG_LOSS_VALIDATE_TIME_CNT))
	        {
	            auxin_plugged = 0;
	            lineInCmdSendReq = BT_LINEIN_DETECTED;    //BTAPP_LineInEvent(0);
	        }
	    }
	}
}

int AnalogIsPlugIn(void)
{
	if (auxin_plugged == 1) return 1;
	else return 0;
}
unsigned char AudioSelected(void)
{
	return audio_selected;
}
unsigned char AudioSelectToggle(void)
{
	if(!BT_isCallIdle() || (BT_GetLinkStatus() & BIT_SCO_LINK_STATUS ))
	{
		if(auxin_plugged)
		{
			if(audio_selected == AUDIO_A2DP)
				audio_selected = AUDIO_AUXIN;
			else
				audio_selected = AUDIO_A2DP;
			if (getI2SAuxInJumper()) //HIGH: I2S
				BT_EnterLineInMode(audio_selected, 1);
			else
				BT_EnterLineInMode(audio_selected, 0);						
			lineInCmdSendReq = BT_LINEIN_IDLE;
			return true;
		}
	}

	if(audio_selected == AUDIO_A2DP)
	{
    	if(BT_isMaster())
			BT_SendMuteInfo(1);	
		//yda174_ForceMute(1);		
		audio_selected = AUDIO_AUXIN;		
		lineInCmdSendReq = BT_LINEIN_MUTE;
		
	}
	else if (audio_selected == AUDIO_AUXIN)
	{
		if(BT_isMaster())
			BT_SendMuteInfo(1);
		//yda174_ForceMute(1);		
		audio_selected = AUDIO_A2DP;
		lineInCmdSendReq = BT_LINEIN_MUTE;	
	}
	else
		return true;  // by jhlee 20171111
	
	AudioToggleTimer = AUDIO_TOGGLE_INTERVAL_MS;

}
void AudioSelectAppsStatus(void)
{
	if(auxin_plugged)
	{
		BT_SendAppBTMStatus(audio_selected == AUDIO_A2DP ? APPS_AUX_PLUG_A2DP : APPS_AUX_PLUG_AUX);						
	}
	else
	{
		BT_SendAppBTMStatus(APPS_AUX_UNPLUG);												
	}
}
unsigned char isAudioSelectUnmute(void)
{
	unsigned char tmp;
	tmp = audioUnMute;
	audioUnMute = 0;
	return tmp;	
}
