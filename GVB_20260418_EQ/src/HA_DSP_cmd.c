/*
 * HA_DSP_cmd.c
 *
 * Created: 2015-10-01 
 *  Author: Rapiarno
 */ 

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "HA_vs1063a.h"
#include "HA_DefinedValues.h"
#include "HA_TimerCount.h"
#include "HA_const_data.h"
#include "efm32hg_msc.h"
#include "HA_MCU_cmd.h"
#include "HA_vs1063a.h"

#include "HA_DSP_cmd.h"

///////////////////////////////////////////////////////////////////////////////////////////
// set VS10xx volume attenuation    0x00 lound - 0xfe silent
extern signed char EQ_value[2][7];
extern bool currentState_LineMic;


void vs_set_volume(uint8_t leftchannel, uint8_t rightchannel)
{
#ifndef SV2_RX
	SCI_Vol = leftchannel << 8 | rightchannel;
	////                                 printf("SCI Volume value : 0x%04X\r\n", SCI_Vol);
	WriteSci(SCI_VOL, SCI_Vol);
#endif
}


void recovery_Mic_input(int Line_Mic) {
	int i;

#if 0
	WriteSci(SCI_CLOCKF, 0x8BE8);
	WriteSci(SCI_AICTRL0, 0xBB80);	// C : Setting the sample rate to 48kHz
	WriteSci(SCI_AICTRL1, 16*1024);	// D : Encoding gain (1024 = 1x) or 0 for automatic gain control   // 0xA400
	WriteSci(SCI_AICTRL2, 0x0);	// E : Maximum autogain amplification (1024 = 1-x)  0x2400 = 9x   0 = 64x(Max)

	SCI_Status = ReadSci(SCI_STATUS);
	WriteSci(SCI_STATUS, SCI_Status | 0x01); // SS_REFERENCE_SEL 1 : 1.65V

	if( Line_Mic == 1) {// 1 : Mic, 0: Line
		WriteSci(SCI_AICTRL3, 0x0014);	// 0x0012 : right channel, 0x0013 : left & right channel
		WriteSci(SCI_MODE, SM_SDINEW | SM_RESET  | SM_ENCODE);  // initial SCI Mode setting for Mic
	}
	else {
		WriteSci(SCI_AICTRL3, 0x0010);	// 0x0012 : right channel, 0x0013 : left & right channel
		WriteSci(SCI_MODE, SM_SDINEW | SM_RESET  | SM_ENCODE | SM_LINE1);  // initial SCI Mode setting for Line
	}

		
	WriteSci(SCI_AIADDR, 0);
	LoadUserCode_EQ7();
	WriteSci(SCI_AIADDR, 0x34);
	vs_Wait();

	if (NC_Index == 1) {  // 1 = NC for voice, 0 = Normal mode
		EQ_VoiceDB_Working(EQ_Select_index);  // write profile EQ to DSP
	}
	else {
		EQ_DB_Working(EQ_Select_index);  // write profile EQ to DSP
	}

	if( Line_Mic == 1) {// 1 : Mic, 0: Line   // Works on Mic Mode only
		//-3 dB at 300 Hz with sample rate 8 kHz:
		WriteSci(SCI_WRAMADDR, 0x5a96);
		WriteSci(SCI_WRAM, 0x728b);
		WriteSci(SCI_WRAM, 0x8d75);
		WriteSci(SCI_WRAM, 0x9aea);
	}
#else
	WriteSci(SCI_CLOCKF, 0x8BE8);
	WriteSci(SCI_AICTRL0, 0xBB80);	// C : Setting the sample rate to 48kHz
	WriteSci(SCI_AICTRL1, 0);	// D : Encoding gain (1024 = 1x) or 0 for automatic gain control   // 0xA400
	WriteSci(SCI_AICTRL2, 64*1024);	// E : Maximum autogain amplification (1024 = 1-x)  0x2400 = 9x   0 = 64x(Max)

	SCI_Status = ReadSci(SCI_STATUS);
	WriteSci(SCI_STATUS, SCI_Status | 0x01); // SS_REFERENCE_SEL 1 : 1.65V


	if( Line_Mic == 1) {// 1 : Mic, 0: Line
		WriteSci(SCI_AICTRL3, 0x0014);	// 0x0012 : right channel, 0x0013 : left & right channel
		WriteSci(SCI_MODE, SM_SDINEW | SM_RESET  | SM_ENCODE);  // initial SCI Mode setting for Mic
	}
	else {
		WriteSci(SCI_AICTRL3, 0x0010);	// 0x0012 : right channel, 0x0013 : left & right channel
		WriteSci(SCI_MODE, SM_SDINEW | SM_RESET  | SM_ENCODE | SM_LINE1);  // initial SCI Mode setting for Line
	}
		
	WriteSci(SCI_AIADDR, 0);
	LoadUserCode_EQ7();
	WriteSci(SCI_AIADDR, 0x34);
	vs_Wait();

	EQ_Select_index = 0;
	EQ_DB_Working(EQ_Select_index);  // write profile EQ to DSP
#endif
}


void LoadUserCode_EQ7(void) {

	unsigned int i = 0;
	unsigned int addr, n, val;
	unsigned int ex_par;


	ex_par = sizeof(plugin_EQ7)/sizeof(plugin_EQ7[0]);
	////                                 printf( "plugin_EQ7 size : = %04d\r\n", ex_par);
	
	while (i<sizeof(plugin_EQ7)/sizeof(plugin_EQ7[0]))
	{
		//pgm_read_word_near
		addr = plugin_EQ7[i++];

		n = plugin_EQ7[i++];
		if (n & 0x8000U)
		{ /* RLE run, replicate n samples */
			n &= 0x7FFF;
			val = plugin_EQ7[i++];	//
			while (n--)							//
			{									//
				WriteSci(addr, val);	// address[n] = plugin[i];
				//__delay_ms(1);
				//s//     printf(str1, "EQ7 addr = %04X, val = %04X, iR=%d\r\n", addr, val, i); usart_serial_write_packet(USART_SERIAL, str1, strlen(str1));
			}									//
			////                                 printf(".");
		} else
		{           /* Copy run, copy n samples */
			while (n--)
			{
				val = plugin_EQ7[i++];
				WriteSci(addr, val);	// address[n] = plugin[i];
				//__delay_ms(1);
				//s//     printf(str1, "EQ7 addr = %04X, val = %04X, iR=%d\r\n", addr, val, i); usart_serial_write_packet(USART_SERIAL, str1, strlen(str1));

			}
			////                                 printf("-");
		}

	}

	int val2 =0;

	val2=i;

}



// EarSpeaker Spatial Processing is a headphone externalizer algorithm. datasheet p75
void EarSpeaker(int level) {
	WriteSci(SCI_WRAMADDR, 0x1e1e);
	switch(level)
	{
	case 1:
		WriteSci(SCI_WRAM, 0);
		break;
	case 2:
		WriteSci(SCI_WRAM, 12000);
		break;
	case 3:
		WriteSci(SCI_WRAM, 38000);
		break;
	case 4:
		WriteSci(SCI_WRAM, 50000);
		break;
	default:
		break;
	}


}


void EQ_DB_display(void) {

}

int	EQ_DB_Working(uint16_t EQ_Select_index) {
	signed int param1, param2, param3, param4, param5, param6, param7 ;
 	signed char eq0, eq1, eq2, eq3, eq4, eq5, eq6;

	eq0 =  EQ7_case[EQ_Select_index][0];
	eq1 =  EQ7_case[EQ_Select_index][1];
	eq2 =  EQ7_case[EQ_Select_index][2];
	eq3 =  EQ7_case[EQ_Select_index][3];
	eq4 =  EQ7_case[EQ_Select_index][4];
	eq5 =  EQ7_case[EQ_Select_index][5];
	eq6 =  EQ7_case[EQ_Select_index][6];

	param1 =  (signed int) (  eq0 * 256);
	param2 =  (signed int) (  eq1 * 256);
	param3 =  (signed int) (  eq2 * 256);
	param4 =  (signed int) (  eq3 * 256);
	param5 =  (signed int) (  eq4 * 256);
	param6 =  (signed int) (  eq5 * 256);
	param7 =  (signed int) (  eq6 * 256);
	
	do {
		WriteSci(SCI_WRAMADDR, 0x180B);
	} while (ReadSci(SCI_WRAM) & 8);

	WriteSci(SCI_WRAMADDR, 0x1800);
	WriteSci(SCI_WRAM, param1);
	WriteSci(SCI_WRAM, param2);
	WriteSci(SCI_WRAM, param3);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, param4);
	WriteSci(SCI_WRAM, param5);
	WriteSci(SCI_WRAM, param6);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, param7);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 8); // Activate filter designer
}

int	EQ_VoiceDB_Working(uint16_t EQ_Select_index) {
#if 1	
	//signed long param1, param2, param3, param4, param5, param6, param7 ;
	signed int param1, param2, param3, param4, param5, param6, param7 ;
 	signed char eq0, eq1, eq2, eq3, eq4, eq5, eq6;

	eq0 =  EQ7_Voice_case[EQ_Select_index][0];
	eq1 =  EQ7_Voice_case[EQ_Select_index][1];
	eq2 =  EQ7_Voice_case[EQ_Select_index][2];
	eq3 =  EQ7_Voice_case[EQ_Select_index][3];
	eq4 =  EQ7_Voice_case[EQ_Select_index][4];
	eq5 =  EQ7_Voice_case[EQ_Select_index][5];
	eq6 =  EQ7_Voice_case[EQ_Select_index][6];

	param1 =  (signed int) (  eq0 * 256);
	param2 =  (signed int) (  eq1 * 256);
	param3 =  (signed int) (  eq2 * 256);
	param4 =  (signed int) (  eq3 * 256);
	param5 =  (signed int) (  eq4 * 256);
	param6 =  (signed int) (  eq5 * 256);
	param7 =  (signed int) (  eq6 * 256);
	
	do {
		WriteSci(SCI_WRAMADDR, 0x180B);
	} while (ReadSci(SCI_WRAM) & 8);

	// Set new equalizer values
	WriteSci(SCI_WRAMADDR, 0x1800);
	WriteSci(SCI_WRAM, param1);
	WriteSci(SCI_WRAM, param2);
	WriteSci(SCI_WRAM, param3);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, param4);
	WriteSci(SCI_WRAM, param5);
	WriteSci(SCI_WRAM, param6);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, param7);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 8); // Activate filter designer
#endif
}



