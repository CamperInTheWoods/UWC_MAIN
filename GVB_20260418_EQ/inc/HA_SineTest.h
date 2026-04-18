/*
 * HA_SineTest.h
 *
 *  Author: Rapiarno
 */ 

#ifndef HA_SINE_TEST_H
#define HA_SINE_TEST_H

static unsigned int SINE_AUDATA[7] = {0x0080, 0x0140, 0x0320, 0x07d0, 0x1388, 0x30d4, 0x79e0};
static unsigned int SINE_AICTRL0 = 0x8000;

extern signed char EQ_value[2][7];
extern int profile_index, leftright_select, frequency_index, frequency_old_index, lastState_volume_value;
extern unsigned char volume_value;


void SineT_Req_Val();
void SineT_Start();
void SineT_Volume();
void SineT_Stop_Sound();
void SineT_Save();
void SineT_Play();

#endif


