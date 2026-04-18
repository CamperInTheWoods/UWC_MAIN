/*
 * HA_DSP_cmd.h
 *
 *  Author: Rapiarno
 */ 
#ifndef HA_DSP_CMD_H
#define HA_DSP_CMD_H

void vs_set_volume(uint8_t leftchannel, uint8_t rightchannel);
void recovery_Mic_input(int Line_Mic );
void LoadUserCode_Patch();
void LoadUserCode_EQ7(void);
void Write_User_EQ_DataToDB();
void EarSpeaker(int level);
void EQ_DB_display(void);
int	EQ_DB_Working(uint16_t EQ_Select_index);
int	EQ_VoiceDB_Working(uint16_t EQ_Select_index);

#endif
