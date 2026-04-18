/*
 * HA_SineTest.c
 *
 *  Author: Rapiarno
 */ 

#include "HA_vs1063a.h"
#include "HA_DefinedValues.h"
#include "efm32hg_msc.h"
#include "HA_SineTest.h"
#include "HA_MCU_cmd.h"

int flag_sign=0;
signed char EQ_value[2][7] = {
//{0, 10, 10, 10, 10, 0, 0},
{0, 0, 0, 0, 0, 0, 0},
{0, 0, 0, 0, 0, 0, 0}
};  					

int profile_index, leftright_select, frequency_index, frequency_old_index, lastState_volume_value;
unsigned char volume_value;


void SineT_Req_Val() {
#if 0
	int i;

	//printf("[HA100]send_val 0 5 10 15 20 25 30\n");

#if 0
	printf("[GENIE]send_val ");
	for(i=0; i<7;i++) {
		//EQ_value[0][i]  =   EQ7_case[EQ_Select_index][i];
		//EQ_value[0][i]  =  (signed char) nvm_eeprom_read_byte(EEPROM_1st_profile_EQ_base_nAddr + i);
		//volume_value =  (unsigned char) (254 -EQ_value[0][i] ) /2 ;
		printf(" %d", EQ_value[0][i]);
	}

	printf("\n");
	//printf("[HA100]send_val 0 5 10 15 20 25 30\n");
#else
	//printf("[GENIE]send_val  0 5 10 15 20 25 30\n");
	printf("[GENIE]send_val %d %d %d %d %d %d %d\n", EQ_value[0][0], EQ_value[0][1], \
	EQ_value[0][2], EQ_value[0][3], EQ_value[0][4],EQ_value[0][5],EQ_value[0][6]);

#endif

	frequency_old_index = -1;
	leftright_select =  0;	// left only
	WriteSci(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);	// 0x0824
#endif
}




void SineT_Start() {
	int i;
	unsigned char leftright_value;


	return;
}


void SineT_Volume() {
#if 0
	frequency_index =  strtoul(argv[1], NULL, 10);
	volume_value =  strtoul(argv[2], NULL, 10);


	EQ_value[leftright_select][frequency_index] =  (signed char)volume_value;
	printf("Changed EQ_value[%d][%d]= %i\r\n", leftright_select, frequency_index,  (signed char)volume_value);

	if((signed char)volume_value < 0) volume_value = 0;

	volume_value =  volume_value + 20;		// 20dB up for sine test
	
	volume_value = (254 - (volume_value*2));

	frequency_old_index = frequency_index;

//	if(frequency_old_index != frequency_index) {

#if 0
		vs_set_volume(0, 0);
		//WriteSci(SCI_AICTRL0, 0x046d);
		//WriteSci(SCI_AICTRL0, SINE_AICTRL0);
		//__delay_ms(10);
		//printf("Changed Test Frequency index[%d] : AUDATA = 0x%04X\r\n", frequency_index, SINE_AUDATA[frequency_index]);
		//WriteSci(SCI_AUDATA, 0x5a64);
		WriteSci(SCI_AUDATA, SINE_AUDATA[frequency_index]);
		SCI_Audata= ReadSci(SCI_AUDATA);
		printf("SCI_Audata : %04X\r\n", SCI_Audata);

		__delay_ms(10);
		//WriteSci(SCI_AIADDR, 0x4020);  // Start SINE signal
		//__delay_ms(200);
#else
		WriteSci(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);	// 0x0824
		vs_set_volume(volume_value, 0xfe);
		//WriteSci(SCI_VOL, 0);
		#if 0
		switch ( frequency_index) 
		{
			case 0:
				WriteSci(SCI_AICTRL0, 0xe76);
				WriteSci(SCI_AUDATA, 0x46d);
				break;		
			case 1:
				WriteSci(SCI_AICTRL0, 0x1f4);
				WriteSci(SCI_AUDATA, 0x5208);
				break;		
			case 2:
				WriteSci(SCI_AICTRL0, 0x5a64);
				WriteSci(SCI_AUDATA, 0x46d);
				break;		
			case 3:
				WriteSci(SCI_AICTRL0, 0xaf0);
				WriteSci(SCI_AUDATA, 0x5dc0);
				break;		
			case 4:
				WriteSci(SCI_AICTRL0, 0x3138);
				WriteSci(SCI_AUDATA, 0x32c8);
				break;		
			case 5:
				WriteSci(SCI_AICTRL0, 0x6aa4);
				WriteSci(SCI_AUDATA, 0x3a98);
				break;		
			case 6:
				WriteSci(SCI_AICTRL0, 0x7b0c);
				WriteSci(SCI_AUDATA, 0x7d00);
				break;		
			case 7:
				WriteSci(SCI_AICTRL0, 0x5555);
				WriteSci(SCI_AUDATA, 0xbb80);
				break;		
			default :
				vs_set_volume(0xfe, 0xfe);
				break;
		}
		#else
		switch ( frequency_index) 
		{
			case 0:
				WriteSci(SCI_AICTRL0, 524 );
				WriteSci(SCI_AUDATA, 8000);
				break;		
			case 1:
				WriteSci(SCI_AICTRL0, 1311 );
				WriteSci(SCI_AUDATA, 8000);
				break;		
			case 2:
				WriteSci(SCI_AICTRL0, 3277 );
				WriteSci(SCI_AUDATA, 8000);
				break;		
			case 3:
				WriteSci(SCI_AICTRL0, 8192 );
				WriteSci(SCI_AUDATA, 8000);
				break;		
			case 4:
				WriteSci(SCI_AICTRL0, 20480 );
				WriteSci(SCI_AUDATA, 8000);
				break;		
			case 5:
				WriteSci(SCI_AICTRL0, 8533 );
				WriteSci(SCI_AUDATA, 48000);
				break;		
			case 6:
				WriteSci(SCI_AICTRL0, 21299 );
				WriteSci(SCI_AUDATA, 48000);
				break;		
			case 7:
				WriteSci(SCI_AICTRL0, 0x5555);
				WriteSci(SCI_AUDATA, 0xbb80);
				break;		
			default :
				vs_set_volume(0xfe, 0xfe);
				break;
		}
		
		#endif
		WriteSci(SCI_AIADDR, 0x4020);  // Start New SINE test

#endif

//	}
	//Sine_tone_generate = HIGH;
#endif
}


void SineT_Stop_Sound() {
	//Sine_tone_generate = LOW;
	vs_set_volume(0xff, 0xff);
	
}



void SineT_Save() {	
#if 0
	int i;

	for(i=0; i<7;i++) {
		//nvm_eeprom_write_byte(EEPROM_7EQ_base_nAddr + i, (unsigned char)EQ_value[leftright_select][i] );
		//nvm_eeprom_write_byte(EEPROM_1st_profile_EQ_base_nAddr + i, (unsigned char)EQ_value[leftright_select][i] );
		printf("EQ_value[leftright_select][%d] = unsigned %d\r\n", i, (unsigned char)EQ_value[leftright_select][i] );
		printf("EQ_value[leftright_select][%d] = signed %d\r\n", i, (signed char)EQ_value[leftright_select][i] );
	}
	//EQ_Select_index = 0;
	//nvm_eeprom_write_byte(EEPROM_SELECTED_EQ_VALUE_nAddr, EQ_Select_index);
}


void SineT_Play() {
	int Profile_EEPROM_7EQ_base_nAddr, i;
	signed int param1, param2, param3, param4, param5, param6, param7 ;
	unsigned char HA_estimated_db;
	signed char play_no[7];

	for(i=0; i<7;i++) {

		HA_estimated_db = EQ_value[leftright_select][i];
		printf("HA_estimated_db= %d\r\n", HA_estimated_db );
		//if (HA_estimated_db < 144 ) {EQ_value[leftright_select][i] = (144 - HA_estimated_db)/2;}  // Bad ear
		if (HA_estimated_db < 234 ) {EQ_value[leftright_select][i] = (234 - HA_estimated_db)/2;}  // Bad ear  10dB 기준으로 설정 
		else {EQ_value[leftright_select][i] = 0;} // Good ear

		play_no[i] = (EQ_value[leftright_select][i] * (-1)) ;
		printf("play_no [%d] = %d\r\n", i, play_no[i]);

		//nvm_eeprom_write_byte(EEPROM_7EQ_base_nAddr + i, play_no[i] );
	}

	param1 =  (signed int) (play_no[0]*256);
	param2 =  (signed int) (play_no[1]*256);
	param3 =  (signed int) (play_no[2]*256);
	param4 =  (signed int) (play_no[3]*256);
	param5 =  (signed int) (play_no[4]*256);
	param6 =  (signed int) (play_no[5]*256);
	param7 =  (signed int) (play_no[6]*256);
 
	do {
		WriteSci(SCI_WRAMADDR, 0x180B);
	} while (ReadSci(SCI_WRAM) & 8);

	// Set new equalizer values
	WriteSci(SCI_WRAMADDR, 0x1800);

	printf("param1 : = 0x%04X\r\n", param1);
	WriteSci(SCI_WRAM, param1);

	printf("param2 : = 0x%04X\r\n", param2);
	WriteSci(SCI_WRAM, param2);

	printf("param3 : = 0x%04X\r\n", param3);
	WriteSci(SCI_WRAM, param3);

	WriteSci(SCI_WRAM, 0);

	printf("param4 : = 0x%04X\r\n", param4);
	WriteSci(SCI_WRAM, param4);

	printf("param5 : = 0x%04X\r\n", param5);
	WriteSci(SCI_WRAM, param5);

	printf("param6 : = 0x%04X\r\n", param6);
	WriteSci(SCI_WRAM, param6);

	WriteSci(SCI_WRAM, 0);

	printf("param7 : = 0x%04X\r\n", param7);
	WriteSci(SCI_WRAM, param7);

	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 8); // Activate filter designer

	Sine_tone_generate = LOW;

	//EQ_Select_index = 0;
	//nvm_eeprom_write_byte(EEPROM_SELECTED_EQ_VALUE_nAddr, EQ_Select_index);

	printf("SineT_Play\r\n");
#endif
}


