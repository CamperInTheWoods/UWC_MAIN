/*
 * HA_MCU_cmd.c
 *
 * Created: 2015-10-01 
 *  Author: Rapiarno
 */ 


#include "HA_DefinedValues.h"
#include "HA_vs1063a.h"
#include "key.h"
#include "HA_SineTest.h"
#include "InitDevice.h"
#include "udelay.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "HA_MCU_cmd.h"
#include "bt_app.h"
#include "ble.h"
#include "bt_command_send.h"
#include "bt_command_decode.h"

#include "HA300_BSP.h"
#include "LED_PWM.h"

#include "HA_TimerCount.h"
#include "HA_DSP_cmd.h"
#include "eeprom_emulation.h"

#include "em_adc.h"
#include "xvf3800_eq.h"

/* oled_show_eq_status()는 main.c에 정의됨 */
extern void oled_show_eq_status(bool eq_on);



extern EE_Variable_TypeDef DB_Index, Group_ID_Sub, MicBT_Index; // Define the non-volatile variables.
extern EE_Variable_TypeDef DB_Vol, DB_NC; // Define the non-volatile variables.
extern EE_Variable_TypeDef DB_Vol_Left, DB_Vol_Right; // Define the non-volatile variables.

extern enum BT_CallStatus;

extern union{
    uint16_t value;
    struct {
        uint16_t SpeakerAddCommandReq        : 1;       //0xE2(NSPK)
        uint16_t BroadcastModeCommandReq     : 1;       //0xE2(broadcast)
        uint16_t NSpeakerTriggerMasterReq    : 1;       //0xE0
        uint16_t NSpeakerTriggerSlaveReq     : 1;       //0xE1
        uint16_t BroadcastTriggerMasterReq   : 1;       //0xE0
        uint16_t BroadcastTriggerSlaveReq    : 1;       //0xE1
        uint16_t PairReq                    : 1;        //ANY_MODE_ENTERING_PAIRING
        uint16_t ResetEEPROMReq             : 1;        //RESET_EEPROM_SETTING
        uint16_t NextSongReq                : 1;        //NEXT_SONG
        uint16_t PreviousSongReq            : 1;        //PREVIOUS_SONG
        uint16_t IncSpkGainReq              : 1;        //INC_SPK_GAIN
        uint16_t DecSpkGainReq              : 1;        //DEC_SPK_GAIN
        uint16_t AcceptCallReq              : 1;        //ACCEPT_CALL
        uint16_t ForceEndCallReq            : 1;        //FORCE_END_CALL
        uint16_t LastNumberRedialReq        : 1;        //LAST_NUMBER_REDIAL
        uint16_t RejectCallReq              : 1;        //REJECT_CALL
    };
} NextMMIActionReq;
//extern union NextMMIActionReq;


extern uint8_t  BT_linkIndex;
extern long tick_DSP_INDEX_timer_count;

//unsigned int test_mem[128];
unsigned int test_mem[1];
//unsigned int dsp_adc_decim_data_left, dsp_adc_decim_data_right, adc_cnt=0, dsp_adc[10], DSP_Act_Cnt=20;
unsigned int DSP_Act_Num=0, DSP_Act_State=1;
unsigned long dsp_adc_sum=0;

uint16_t EQ_Select_index, EQ_DB_index;		// EEPROM
uint8_t Group_Zone_Main, Group_Zone_Sub;

uint16_t NC_DB_index;		// Noise reduction

bool currentState_LineMic=1;

bool BT_working_flag = false;

 int connection_count = 0;

long SET_VOL_timer_count = 1;

long SOUND_LEVEL_timer_count = 1;
long SOUND_LEVEL_CHECK_timer_count = 1;

long LED_timer_count = 1;
long RED_LED_timer_count = 1;

long AB1565_Power_On_timer_count = 1;

bool NC_Index = 0;  // 1 = NC for voice, 0 = Normal mode

long Time_1SEC_timer_count = 1;

long Time_A2DP_timer_count = 1;


long LED_Blink_ON_timer_count = 1;
long LED_Blink_OFF_timer_count = 1;

long DSP_INDEX_timer_count = 1;
long AUDIOJACK_timer_count = 1;


long NSPK_Voice_timer_count = 1;


long BT_ON_timer_count = 1;
long BT_NSPK_ON_timer_count = 1;

//uint8_t DB_LED_Color[10] = { RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, VIOLET, WHITE, INDIGO, MAGENTA };
uint8_t DB_LED_Color[10] = { RED, ORANGE, YELLOW, GREEN, BLUE, INDIGO, VIOLET, WHITE,  CYAN, MAGENTA };  

unsigned char MAX_Volume=0,MIN_Volume=80, MAX_VOL_Count = 16, MIN_VOL_Count=1 ;    // for released version
unsigned char VOL_Count, VOL_number;
unsigned char GAP_Volume;


uint8_t MAX_Left_Volume=0, MIN_Left_Volume=80, MAX_Left_VOL_Count=16, MIN_Left_VOL_Count=1;
uint8_t VOL_Left_Count, VOL_Left_number;

uint8_t MAX_Right_Volume=0, MIN_Right_Volume=80, MAX_Right_VOL_Count=16, MIN_Right_VOL_Count=1;
uint8_t VOL_Right_Count, VOL_Right_number;

int a2dpstat;
bool a2dpstat_LED = 0;

// for HA100 Termianl
char CmdBuf[COMMAND_BUF_SIZE] = "";
char argv[10][20];
int Cmd_flag, argc;
int nRxCnt=0;

int micgain = 12;
//unsigned char nRxCnt=0;

// for Bluetooth
//char CmdBuf_BT[COMMAND_BUF_SIZE] = "";
//char argv_BT[10][20];
//int Cmd_flag_BT, argc_BT;
//int nRxCnt_BT=0;
//unsigned char nRxCnt_BT=0;

int earp=0;

char incomingByte;      // a variable to read incoming serial data into
char incomingByte_BT;      // a variable to read incoming serial data into
bool Sine_tone_state = LOW;

bool first_MSPK_Button = true;
char incoming_cmd_flag = 0;      // a variable to read incoming serial data into


signed int Pparam1=0, Pparam2=0, Pparam3=0, Pparam4=0, Pparam5=0, Pparam6=0, Pparam7=0 ;


uint8_t ang_input = 1;   // 0 : Line In connect, 1 : Line In Disconnect

uint8_t Master_Volume;
uint8_t Master_Left_Volume, Master_Right_Volume;
uint8_t Gain_Data;

uint32_t GN_Working_State;

int BT_cmd_No = 0;

typedef enum {
    UI_IDLE = 0,
    UI_DB_INDEX = 1,
    UI_BT_WORKING = 2
} UI_STATUS;
UI_STATUS UI_Before_Status;

uint8_t Rainbow_RGB[7][3] = { {0, 100, 100}, {0, 64, 100}, {0, 0, 100}, {100, 50, 100}, {100, 50, 0}, {100, 100, 0}, {20, 100, 80} };


char rcmd[30][20]={
	"req_val",				// 0
	"start",				// 1
	"volume",				// 2
	"stop_sound",			// 3
	"save",				// 4
	"exit",				// 5
	"play",				// 6
	"rdm",				// 7
	"wrm",				// 8
	"tm",				// 9
	"sine",				// 10
	"nsine",				// 11
	"vol",				// 12
	"eq",					// 13
	"+SPPDATA=12,volume",				// 14
	"+SPPDATA=13,volume",				// 15
	"+SPPDATA=14,volume",		// 16
	"+GATTDATA=24,eq",					// 17
	"+DEVSTAT",			// 18
	"+A2DPSTAT[0]",			// 19
	"CIS_connected",			// 20 extra char for GLE by eh
	"GLE_C",			// 21 extra char for GLE by eh
	"CIS_disconnected",			// 22 extra char for GLE by eh
	" ",					//
};


//received_cmd_value = PROT_C
#define REQ_VAL			0
#define START			1
#define VOLUME			2
#define STOP_SOUND		3
#define SAVE			4
#define EXIT			5
#define PLAY			6
//------------------------------
#define RDM				7
#define WRM				8
#define TM				9
#define T_SINE			10
#define T_NSINE			11
#define VOL				12
#define EQ		   	    13	// test volume
#define VOLUME1		14	// get eeprom value
#define VOLUME2			15
#define VOLUME3		16
#define NSPK			17
#define DEVSTAT			18
#define A2DPSTAT			19
#define CIS_CONNECTED			20
#define GLE_COMMAND			21
#define CIS_DISCONNECTED			22
#define NO_CMD			23

void gpioSetup(void);
void GPIO_Unified_IRQ(void) ;

void print_nspk_status(void);

void Write_DSP_Memory() {
#if 0
	int i;
	
	WriteSci(SCI_WRAMADDR, 0x1800);
	for (i=0; i<128; i++) {
		test_mem[i] = 0;
		WriteSci(SCI_WRAM, (unsigned int)i);
	}
#endif
}

void 	Read_DSP_Memory() {
#if 0
	int i;


	WriteSci(SCI_WRAMADDR, 0x1800);
	for (i=0; i<128; i++) {
		test_mem[i] = ReadSci(SCI_WRAM);
		usbprintf("test_mem[%d] = %d\r\n", i, test_mem[i]);
		delayMs(10);
	}
#endif

}


void 	Read_T_Memory() {
#if 0
	int i;
	
	for (i=0; i<128; i++) {
		usbprintf("test_mem[%d] = %d\r\n", i, test_mem[i]);
	}
#endif
}


////////////////////////////////////////////////////////////////////////////////////////
unsigned int volume_f()
{
	unsigned char left, right;

	if(argc == 1) {
		left = right = 0;  // MAX vol
	}
	else if(argc == 2) {
		left = strtoul(argv[1], NULL, 10);
		right = left;
	}
	else if(argc == 3){

		left = strtoul(argv[1], NULL, 10);
		right = strtoul(argv[2], NULL, 10);
	}

	if(left == 255) left = 254;
	if(right == 255) right = 254;
	
	vs_set_volume(left, right);

	usbprintf("Vol : left  = %d, right = %d\n\r", left, right);

	return 1;

}// end of function

///////////////////////////////////////////////////////////////////////////////////////////
unsigned int get_eeprom_f()
{
	unsigned char param_addr;
	unsigned char param_value;
	int i, count;

	if(argc == 2) {
		param_addr = strtoul(argv[1], NULL, 16);
		param_value = nvm_eeprom_read_byte(param_addr);
		//usbprintf("EEPROM 0x%02X = 0x%02X\n\r", param_addr, param_value);
	}
	else if (argc == 3) {
		param_addr = strtoul(argv[1], NULL, 16);
		count = strtoul(argv[2], NULL, 10);
		
		//usbprintf("EEPROM 0x%02X = ", param_addr+i);
		for(i=0; i< count; i++) {
			param_value = nvm_eeprom_read_byte(param_addr+i);
			//usbprintf("0x%02X, ", param_value);
		}
		//usbprintf("\n\r");
	}
	return 1;

}// end of function


/*
  model_name Function
*/
void model_name() {

	//printf("[GENIE]model_name HA-110\n");
	//printf("\n");
}

/*
  Equalizer Function
*/
void equalizer_7band() {
//	printf("equalizer_7band\n\r");
	LED_BLUE_Blink_Count(2);


	do {
		WriteSci(SCI_WRAMADDR, 0x180B);
	} while (ReadSci(SCI_WRAM) & 8);

	// Set new equalizer values
	WriteSci(SCI_WRAMADDR, 0x1800);

//	printf("param1 : = 0x%04X\r\n", Pparam1);
	WriteSci(SCI_WRAM, Pparam1);

//	printf("param2 : = 0x%04X\r\n", Pparam2);
	WriteSci(SCI_WRAM, Pparam2);

//	printf("param3 : = 0x%04X\r\n", Pparam3);
	WriteSci(SCI_WRAM, Pparam3);

	WriteSci(SCI_WRAM, 0);

//	printf("param4 : = 0x%04X\r\n", Pparam4);
	WriteSci(SCI_WRAM, Pparam4);

//	printf("param5 : = 0x%04X\r\n", Pparam5);
	WriteSci(SCI_WRAM, Pparam5);

//	printf("param6 : = 0x%04X\r\n", Pparam6);
	WriteSci(SCI_WRAM, Pparam6);

	WriteSci(SCI_WRAM, 0);

//	printf("param7 : = 0x%04X\r\n", Pparam7);
	WriteSci(SCI_WRAM, Pparam7);

	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 0);
	WriteSci(SCI_WRAM, 8); // Activate filter designer
	
}


/*
  SetFilter Function
*/
void SetFilter(unsigned int memoryLocation, unsigned int freqHz, double dB, double q, signed int left, signed int right) {
#if 0
	// Check that a previous filter update is not going on,  and wait if necessary.
	do {
		WriteSci(SCI_WRAMADDR, 0x1800 + memoryLocation*4 + 3);
	} while (ReadSci(SCI_WRAM) & 1);

	// Do our filter update
	WriteSci(SCI_WRAMADDR, 0x1800 + memoryLocation*4);
	WriteSci(SCI_WRAM, freqHz);
	WriteSci(SCI_WRAM, (signed int)(dB*256.0));
	WriteSci(SCI_WRAM, (unsigned int)( q*256.0));
	WriteSci(SCI_WRAM, (left?4:0)|(right?2:0)|1); // Left, right, update
#endif
}


///////////////////////////////////////////////////////////////////////////////////////////
int setfilter_f() {

#if 0
	int flag;
	unsigned int memoryLocation_v;
	unsigned int freqHz_v;
	signed long dB_v;
	signed long q_v;
	signed int left_v;
	signed int right_v;

	memoryLocation_v = strtoul(argv[1], NULL, 10);
	freqHz_v = strtoul(argv[2], NULL, 10);
	dB_v = strtoul(argv[3], NULL, 10);
	q_v = strtoul(argv[4], NULL, 10);
	left_v = strtoul(argv[5], NULL, 10);
	right_v = strtoul(argv[6], NULL, 10);

	usbprintf("memoryLocation_v = %i\n\r", memoryLocation_v);
	usbprintf("freqHz_v = %i\n\r", freqHz_v);
	usbprintf("dB_v = %i\n\r", dB_v);
	usbprintf("q_v = %i\n\r", q_v);
	usbprintf("left_v = %i\n\r", left_v);
	usbprintf("right_v = %i\n\r", right_v);
	SetFilter(memoryLocation_v, freqHz_v, dB_v, q_v, left_v, right_v);
#endif	
	return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////
// unsigned char sine_test_on[] = {0x53, 0xef, 0x6e, 126, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };	// 5168Hz.

unsigned int t_sine_f()
{
	#if 0
	//int j=0;

	unsigned int param;

	param = strtoul(argv[1], NULL, 10);

	if ( param == 1 ) {
		WriteSci(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);	// 0x0824
		//SCI_Mode = ReadSci(SCI_MODE);
		//SCI_Mode |= SM_TESTS;
		//WriteSci(SCI_MODE, SCI_Mode);
		WriteSci(SCI_VOL, 0);
		WriteSdi(sine_test_on, 16);
		usbprintf("sine ON\r\n");
		//SCI_Mode &= ~SM_TESTS;
		//WriteSci(SCI_MODE, SCI_Mode);
	}
	else {
		SCI_Mode = ReadSci(SCI_MODE);
		//SCI_Mode |= SM_TESTS;
		//WriteSci(SCI_MODE, SCI_Mode);
		WriteSci(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);	// 0x0824
		WriteSdi(sine_test_off, 16);
		usbprintf("sine OFF\r\n");
		//SCI_Mode &= ~SM_TESTS;
		//WriteSci(SCI_MODE, SCI_Mode);
	}
	return 1;
	#endif

}// end of function



unsigned int t_newsine_f()
{
	#if 0
	//int j=0;

	unsigned int param;

	param = strtoul(argv[1], NULL, 10);

	WriteSci(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);	// 0x0824
	WriteSci(SCI_VOL, 0);
	switch ( param) 
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
	WriteSci(SCI_AIADDR, 0x4020);  // Start New SINE test

	return 1;
	#endif

}// end of function





/*
int	MODE_button_handle() {
	int event =MODE_EV_NONE;


	int button_now_pressed = GPIO_PinInGet(MODE_BUTTON_PORT, MODE_BUTTON_PIN) ; //0 :ioport_get_pin_level(MODE_BUTTON); // 0 : not presss,  1:pressed

	if (!button_now_pressed) { 
		  MODE_long_k_event = 0;
	}

	if (!button_now_pressed  && MODE_button_was_pressed) { 
			usbprintf("cnt %d\r\n",MODE_button_pressed_counter);
		if ( (MODE_SHORTPRESS_LEN < MODE_button_pressed_counter) && ( MODE_button_pressed_counter < MODE_LONGPRESS_LEN)) {
			//usbprintf("xxx %d, %d\r\n",!button_now_pressed, MODE_button_was_pressed);
		  	event = MODE_EV_SHORTPRESS;
		}
		else {
		  event = MODE_EV_NONE;
		}
	}
	else if (button_now_pressed  && MODE_button_was_pressed) { 
			//usbprintf("cnt xxx %d\r\n",MODE_button_was_pressed);
		if (MODE_button_pressed_counter >= MODE_LONGPRESS_LEN && MODE_long_k_event == 0) {
			//usbprintf("yyy %d, %d\r\n",button_now_pressed, MODE_button_was_pressed);
		  	event = MODE_EV_LONGPRESS;
		  	MODE_long_k_event = 1;
		}
		else {
		  event = MODE_EV_NONE;
		}
	}
	else {
		event = MODE_EV_NONE;
	}


	if (button_now_pressed && event != MODE_EV_LONGPRESS) {
		++MODE_button_pressed_counter;
	}
	else {
		MODE_button_pressed_counter = 0;
	}

	MODE_button_was_pressed = button_now_pressed;
	return event;

}


int	VOL_UP_button_handle() {
	int event = VOLUP_EV_NONE;

	int button_now_pressed =  GPIO_PinInGet(UP_BUTTON_PORT, UP_BUTTON_PIN) ; // 0 : not presss,  1:pressed


	if (!button_now_pressed) { 
		  VOLUP_long_k_event = 0;
	}

	if (!button_now_pressed  && VOLUP_button_was_pressed) { 
		if ( (VOLUP_SHORTPRESS_LEN < VOLUP_button_pressed_counter) && ( VOLUP_button_pressed_counter < VOLUP_LONGPRESS_LEN)) {
		//usbprintf("ZZZ %d, %d\r\n",!button_now_pressed, POWER_button_was_pressed);
		  event = VOLUP_EV_SHORTPRESS;
		}
		else {
		  event = VOLUP_EV_NONE;
		}
	}
	else if (button_now_pressed  && VOLUP_button_was_pressed) { 
		if (VOLUP_button_pressed_counter >= VOLUP_LONGPRESS_LEN && VOLUP_long_k_event == 0) {
		//usbprintf("AAA %d, %d\r\n",button_now_pressed, POWER_button_was_pressed);
		  event = VOLUP_EV_LONGPRESS;
		  VOLUP_long_k_event = 1;
		}
		else {
		  event = VOLUP_EV_NONE;
		}
	}
	else {
		event = VOLUP_EV_NONE;
	}


	if (button_now_pressed && event != VOLUP_EV_LONGPRESS) {
		++VOLUP_button_pressed_counter;
	}
	else {
		VOLUP_button_pressed_counter = 0;
	}

	VOLUP_button_was_pressed = button_now_pressed;

	return event;

}


int	VOL_DOWN_button_handle() {
	int event = VOLDOWN_EV_NONE;

	int button_now_pressed = GPIO_PinInGet(DOWN_BUTTON_PORT, DOWN_BUTTON_PIN) ; // 0 : not presss,  1:pressed

	if (!button_now_pressed) { 
		  VOLDOWN_long_k_event = 0;
	}

	if (!button_now_pressed  && VOLDOWN_button_was_pressed) { 
		//usbprintf("%d, %d\r\n",!button_now_pressed, VOLDOWN_button_pressed_counter);
		if ( (VOLDOWN_SHORTPRESS_LEN < VOLDOWN_button_pressed_counter) && ( VOLDOWN_button_pressed_counter < VOLDOWN_LONGPRESS_LEN)) {
		  event = VOLDOWN_EV_SHORTPRESS;
		}
		else {
		  event = VOLDOWN_EV_NONE;
		}
	}
	else if (button_now_pressed  && VOLDOWN_button_was_pressed) { 
		//usbprintf("%d, %d\r\n",button_now_pressed, VOLDOWN_button_pressed_counter);
		if (VOLDOWN_button_pressed_counter >= VOLDOWN_LONGPRESS_LEN && VOLDOWN_long_k_event == 0) {
		  event = VOLUP_EV_LONGPRESS;
		  VOLDOWN_long_k_event = 1;
		}
		else {
		  event = VOLDOWN_EV_NONE;
		}
	}
	else {
		event = VOLDOWN_EV_NONE;
	}


	if (button_now_pressed && event != VOLDOWN_EV_LONGPRESS) {
		++VOLDOWN_button_pressed_counter;
	}
	else {
		VOLDOWN_button_pressed_counter = 0;
	}

	VOLDOWN_button_was_pressed = button_now_pressed;
	return event;

}
*/


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


void data_printf(uint8_t data){

//	 printf("%02x, ", data); // MB64 printf
}


void tx_data_printf(uint8_t* data, uint16_t size){

#if defined (DBG_PRINT)
	 printf("\n\rTX : "); // MB64 printf
	 while(size--) {
		 printf("%02x ",*data++);
	 }
	 printf("\n\r");
#endif	 
}

void BM64_UART_TX_ready(){
#if 0
	GPIO_PinModeSet(gpioPortD, 6,     gpioModePushPull, 0);   //MFB, P2_0 HIGH for Flash Application Mode
	delayMs(1);
	GPIO_PinModeSet(gpioPortD, 6,     gpioModePushPull, 1);   //MFB, P2_0 HIGH for Flash Application Mode
	delayMs(1);
	GPIO_PinModeSet(gpioPortD, 6,     gpioModePushPull, 0);   //MFB, P2_0 HIGH for Flash Application Mode
	delayMs(2);
	GPIO_PinModeSet(gpioPortD, 6,     gpioModePushPull, 1);   //MFB, P2_0 HIGH for Flash Application Mode
	delayMs(2);
#endif	
}


void print_nspk() {

	int tmp_nspk_link;
	int tmp_nspk_event;
	int tmp_nspk_status;

	int tmp_btapp_task_status;

	tmp_nspk_link = BT_nspk_link;
	tmp_nspk_event = BT_nspk_event;
	tmp_nspk_status = BT_nspk_status;

	tmp_btapp_task_status = BTAPP_task_status;

	
	print_nspk_status();
	if (tmp_nspk_link != BT_nspk_link) {
		//printf("\n\rBT_eCSBStatus.BT_nspk_link = %d\r\n",  BT_nspk_link);
	}
	if (tmp_nspk_event != BT_nspk_event) {
		//printf("\n\rBT_eCSBStatus.BT_nspk_event = %d\r\n",  BT_nspk_event);
	}
	if (tmp_nspk_status != BT_nspk_status) {
		//printf("\n\rBT_eCSBStatus.nspk_status = %d\r\n",  BT_nspk_status);
	}

	if (tmp_btapp_task_status != BTAPP_task_status) {
		//printf("\n\rBTAPP_task_status = %d\r\n",  BTAPP_task_status);
	}
}

void print_nspk_value() {

	print_nspk_status();

	//printf("BT_eCSBStatus.nspk_link = %d\r\n",  BT_nspk_link);
	//printf("BT_eCSBStatus.nspk_event = %d\r\n",  BT_nspk_event);
	//printf("BT_eCSBStatus.nspk_status = %d\r\n",  BT_nspk_status);
	//printf("Print_BT_CMD_SendState = %d\r\n",  Print_BT_CMD_SendState);
}

/*-----------------------------------------------------------------------------*/
//this is test function, don't need to migrate the code.
void Button_Event_task()
{
    switch (GN_Working_State)
    {
        case GN_STATUS_IDLE:
			Button_event_loop();
            break;
        case GN_STATUS_GROUP_ZONE:
			Button_event_Group_Zone_ID();
            break;
        case GN_STATUS_MSPK:
			Button_event_MSPK();
            break;
        case GN_STATUS_DB_INDEX:
			Button_event_DB_Index();
            break;
        case GN_STATUS_BT_OFF:
            break;
        case GN_STATUS_BT_ON: 
			Button_event_BT_ON();
            break;
        case GN_STATUS_L_VOLUME: 
			Button_event_L_Volume();
        case GN_STATUS_R_VOLUME: 
			Button_event_R_Volume();
            break;
        case GN_STATUS_TEST_MODE:
			Button_event_Test_Mode();
            break;
    }
}

void Button_event_Test_Mode() {

	int i;

	if (BUTTON_event != NONE_push_event) {

//		printf("[GT3] UI state = Button_event_Test_Mode\r\n");
		
		if(BUTTON_event == UP_push_event){
			SetColor(BLUE);	//	SET_LED_BLUE(0);
		}
		else if (BUTTON_event == DOWN_push_event){
			SetColor(GREEN);	//	SET_LED_GREEN(0);
		}
		else if (BUTTON_event == MODE_push_event){
			SetColor(RED);	//	SET_LED_RED(0);
		}

		//GN_Working_State = GN_STATUS_IDLE;
		LED_timer_count = 0;
		tick_LED_timer_count = 0;
//		Key_Button_state = 0;
		
		BUTTON_event = NONE_push_event;
	}
}

int EarSpeakerleve=0;

void Button_event_BT_ON() {
 }


void Button_event_L_Volume() {
	int i;

	if (BUTTON_event != NONE_push_event) {

		if(BUTTON_event) { REG_LED_Off(); } // Clear All LEDs.
		switch(BUTTON_event)
		{
		case MODE_push_event:
			break;
		case MODE_middle_event:
		case MODE_long_event:
			break;
		case UP_push_event:
 			if(VOL_Left_number <MAX_Left_VOL_Count ) {
				VOL_Left_number++;
	 			Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;
				vs_set_volume(Master_Left_Volume, Master_Right_Volume);
			}
			if (VOL_Left_number == 16) {
//				SET_LED_BLUE(0);
				SetColor(BLUE);
			}
			else {
				SetColor(WHITE);
			}
			EE_Write(&DB_Vol_Left, VOL_Left_number);
			break;
		case UP_middle_event:
		case UP_long_event:
			break;
		case DOWN_push_event:
			if(VOL_Left_number >MIN_Left_VOL_Count ) {
				VOL_Left_number--;
	 			Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;

				vs_set_volume(Master_Left_Volume, Master_Right_Volume);
			}
			if (VOL_Left_number == 1) {
				SetColor(RED);
			}
			else {
				SetColor(WHITE);
			}
			EE_Write(&DB_Vol_Left, VOL_Left_number);
			break;
		case DOWN_middle_event:
		case DOWN_long_event:
			break;
		defualt:
			break;
		}
		BUTTON_event = NONE_push_event;
	}

}


void Button_event_R_Volume() {
	int i;

	if (BUTTON_event != NONE_push_event) {

		if(BUTTON_event) { REG_LED_Off(); } // Clear All LEDs.
		switch(BUTTON_event)
		{
		case MODE_push_event:
			break;
		case MODE_middle_event:
		case MODE_long_event:
			break;
		case UP_push_event:
 			if(VOL_Right_number <MAX_Right_VOL_Count ) {
				VOL_Right_number++;
	 			Master_Right_Volume = MIN_Right_Volume - VOL_Right_number * GAP_Volume;
				vs_set_volume(Master_Left_Volume, Master_Right_Volume);
			}
			if (VOL_Right_number == 16 ) {
//				SET_LED_BLUE(0);
				SetColor(BLUE);
			}
			else {
				SetColor(WHITE);
			}
			EE_Write(&DB_Vol_Right, VOL_Right_number);
			break;
		case UP_middle_event:
		case UP_long_event:
			break;
		case DOWN_push_event:
			if(VOL_Right_number >MIN_Right_VOL_Count ) {
				VOL_Right_number--;
	 			Master_Right_Volume = MIN_Right_Volume - VOL_Right_number * GAP_Volume;

				vs_set_volume(Master_Left_Volume, Master_Right_Volume);
			}
			if ( VOL_Right_number == 1 ) {
				SetColor(RED);
			}
			else {
				SetColor(WHITE);
			}
			EE_Write(&DB_Vol_Right, VOL_Right_number);
			break;
		case DOWN_middle_event:
		case DOWN_long_event:
			break;
		defualt:
			break;
		}
		BUTTON_event = NONE_push_event;
	}

}




void Button_event_Group_Zone_ID() {
	int i;

	// Group_Zone_Sub = Group_Zone_readValue + 36 ; // Sub Start 36
	int  tmep = (int) Group_Zone_Sub-0x36;

	if (BUTTON_event != NONE_push_event) {

//		printf("[GT3] UI state = Button_event_Group_Zone_ID\r\n");

		if(BUTTON_event) { REG_LED_Off(); } // Clear All LEDs.
		switch(BUTTON_event)
		{
		case MODE_push_event:
			for(i=0; i<3;i++) {
				SetColor(DB_LED_Color[tmep/7]);
				delayMs(100);
				REG_LED_Off();
				delayMs(100);
			}
			BT_GroupZoneWrite(Group_Zone_Main, Group_Zone_Sub);
			EE_Write(&Group_ID_Sub, (uint16_t)tmep);

			GN_Working_State = GN_STATUS_IDLE;
			break;
		case MODE_middle_event:
		case MODE_long_event:
			break;
		case UP_push_event:
			if(tmep < (70-1)) {
				tmep++; 
				Group_Zone_Sub = Group_Zone_Sub +1; 
			}
			//printf("Group_Zone_Sub = %d, %d\r\n",  tmep, Group_Zone_Sub);
				i = tmep/7;
				Blink_Count = i+1;
				blink_color = tmep%7 + 1;
				LED_Blink_ON_timer_count = 0;
			tick_DSP_INDEX_timer_count = 0;

			BT_GroupZoneWrite(Group_Zone_Main, Group_Zone_Sub);
			EE_Write(&Group_ID_Sub, (uint16_t)tmep);

			break;
		case UP_middle_event:
		case UP_long_event:
			break;
		case DOWN_push_event:
			if(tmep > 0)  {
				tmep--; 
				Group_Zone_Sub = Group_Zone_Sub -1; 
			}
			//printf("Group_Zone_Sub = %d, %d\r\n",  tmep, Group_Zone_Sub);
				i = tmep/7;
				Blink_Count = i+1;
				blink_color = tmep%7 + 1;
				LED_Blink_ON_timer_count = 0;
			tick_DSP_INDEX_timer_count = 0;
			BT_GroupZoneWrite(Group_Zone_Main, Group_Zone_Sub);
			EE_Write(&Group_ID_Sub, (uint16_t)tmep);

			break;
		case DOWN_middle_event:
		case DOWN_long_event:
			break;
		defualt:
			break;
		}
		BUTTON_event = NONE_push_event;

//		Group_Zone_Sub = tmep +1;


	}

}

void Button_event_DB_Index() {

	int i;

	if (BUTTON_event != NONE_push_event) {
		if(BUTTON_event == MODE_push_event){
			tick_DSP_INDEX_timer_count = 0;
			DSP_INDEX_timer_count = 1;

			EE_Write(&DB_Index, EQ_Select_index);
			for(i=0; i<EQ_Select_index;i++) {
				SET_LED_GREEN(0);
				delayMs(200);
				SET_LED_GREEN(1);
				delayMs(200);
			}
			
			if (UI_Before_Status == UI_IDLE) {
				GN_Working_State = GN_STATUS_IDLE;
			} else if (UI_Before_Status == UI_BT_WORKING) {
				GN_Working_State = GN_STATUS_BT_ON;
			}
		}
		else if(BUTTON_event == UP_push_event){
				if(EQ_Select_index == (EQ_MAX_NUMBER-1)) EQ_Select_index = 0;
				else EQ_Select_index++;
				//printf("EQ_Select_index = %d\r\n",  EQ_Select_index);
				EQ_DB_Working(EQ_Select_index);  // write profile EQ to DSP
				for(i=0; i<EQ_Select_index;i++) {
					SET_LED_GREEN(0);
					delayMs(200);
					SET_LED_GREEN(1);
					delayMs(200);
				}
		}
		else if(BUTTON_event == DOWN_push_event){
				if(EQ_Select_index == 0) EQ_Select_index = 6;
				else EQ_Select_index--;
				//printf("EQ_Select_index = %d\r\n",  EQ_Select_index);
				EQ_DB_Working(EQ_Select_index);  // write profile EQ to DSP
				for(i=0; i<EQ_Select_index;i++) {
					SET_LED_GREEN(0);
					delayMs(200);
					SET_LED_GREEN(1);
					delayMs(200);
				}
		}
		else if (BUTTON_event == DOWN_middle_event ||BUTTON_event == DOWN_long_event ){
		}

		BUTTON_event = NONE_push_event;
	}
}



void Button_event_MSPK() {

	int i;

	if (BUTTON_event != NONE_push_event) {

//		printf("[GT3] UI state = Button_event_MSPK\r\n");

		
		if(BUTTON_event == UP_push_event){
			LED_GREEN_Blink_Count(2);
            if (BT_IsCommandSendTaskIdle()) {
				//printf("NSPK_ADD_THIRD_SPK\n\r");
	            BT_MMI_ActionCommand ( 0xF6, BT_linkIndex);     //NSPK_ADD_THIRD_SPK
            }
		}
		else if (BUTTON_event == DOWN_push_event){
			LED_BLUE_Blink_Count(2);
			//printf("BTAPP_GroupSpeakerSoundSync\n\r");
			BM64_UART_TX_ready();		 	
	        BTAPP_GroupSpeakerSoundSync();
		}
		else if (BUTTON_event == MODE_push_event){
			SetColor(RED);	//	SET_LED_RED(0);
		}

		GN_Working_State = GN_STATUS_IDLE;
		LED_timer_count = 0;
		tick_LED_timer_count = 0;
		//KEY_Button_timer1ms = 0;
//		Key_Button_state = 0;
		
		BUTTON_event = NONE_push_event;
	}
}

void LED_RED_Blink_Count(int j){
	int i;
	for(i=0; i<j;i++) {
	  SetColor(RED);	//	SET_LED_RED(0);
	  delayMs(200);
	  SetColor(BLACK);	//	SET_LED_RED(1);
	  delayMs(200);
	}
}

void LED_GREEN_Blink_Count(int j){
	int i;
	for(i=0; i<j;i++) {
	  SetColor(GREEN);	//	SET_LED_GREEN(0);
	  delayMs(200);
	  SetColor(BLACK);	//	SET_LED_GREEN(1);
	  delayMs(200);
	}
}

void LED_BLUE_Blink_Count(int j){
	int i;
	for(i=0; i<j;i++) {
	  SetColor(BLUE);	//	SET_LED_BLUE(0);
	  delayMs(200);
	  SetColor(BLACK);	//	SET_LED_BLUE(1);
	  delayMs(200);
	}
}

void REG_LED_Off(){
	SetColor(BLACK);	//	SET_LED_RED(1);
						//	SET_LED_GREEN(1);
						//	SET_LED_BLUE(1);
}

void Button_event_loop()
{
	int i;
   uint32_t sample;


	if (BUTTON_event != NONE_push_event)
	{
		if(BUTTON_event) { REG_LED_Off(); } // Clear All LEDs.

		switch(BUTTON_event)
		{
			case MODE_push_event:
				if(mic_mute_state == true) {  //Up_Key_Button_initial_state = 1;
					mic_mute_state = false;
					SET_PAM8019_MUTE(0);   // High: outputs Hi-Z, Low: outputs enabled
				}
				else {
					mic_mute_state = true;
					SET_PAM8019_MUTE(1);   // High: outputs Hi-Z, Low: outputs enabled
				}
				break;
			case MODE_middle_event:
			{
				/* MODE 2~8초: XVF3800 PP EQ 켜기/끄기 토글 */
				bool eq_on = xvf3800_eq_toggle();
				oled_show_eq_status(eq_on);
				if (eq_on) {
					SetColor(GREEN);  /* EQ ON  → 녹색 */
				} else {
					SetColor(BLUE);   /* EQ OFF → 파란색 */
				}
				LED_timer_count = 0;
				tick_LED_timer_count = 0;
				break;
			}
			case MODE_long_event:
				/* MODE 8초+: VS1063a EQ 프리셋 선택 (기존 middle 동작) */
				UI_Before_Status = UI_IDLE;
				GN_Working_State = GN_STATUS_DB_INDEX;
				SET_LED_RED(1);
				SET_LED_GREEN(1);
				SET_LED_BLUE(1);

				for(i=0; i<EQ_Select_index;i++) {
					SET_LED_GREEN(0);
					delayMs(200);
					SET_LED_GREEN(1);
					delayMs(200);
				}

				SET_LED_BLUE(0);
				break;

			case UP_push_event:
	 			if(VOL_Left_number <MAX_Left_VOL_Count && VOL_Right_number <MAX_Right_VOL_Count  ) {
					VOL_Left_number++;
		 			Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;

					VOL_Right_number++;
		 			Master_Right_Volume = MIN_Right_Volume - VOL_Right_number * GAP_Volume;

					//vs_set_volume(Master_Left_Volume, Master_Right_Volume);
					SET_LED_RED(1);
					SET_LED_GREEN(1);
					SET_LED_BLUE(1);
					if (VOL_Right_number > 10 ) {
						VOL_Right_number=10;
					}
					PAM8019_Volume(VOL_Right_number*50+500);
					SET_LED_GREEN(0);
					//PAM8019_Volume(800);
					//while(1);

				}
				LED_timer_count = 0;
				tick_LED_timer_count = 0;
#if 0
				if (VOL_Left_number == 16 || VOL_Right_number == 16 ) {
					SetColor(BLUE);
				}
				else {
					SetColor(WHITE);
				}
				EE_Write(&DB_Vol_Left, VOL_Left_number);
				EE_Write(&DB_Vol_Right, VOL_Right_number);

				LED_timer_count = 0;
				tick_LED_timer_count = 0;
#endif				
				break;

			case UP_middle_event:
			case UP_long_event:
				break;
			case DOWN_push_event:
				if(VOL_Left_number >MIN_Left_VOL_Count && VOL_Right_number >MIN_Right_VOL_Count ) {
					VOL_Left_number--;
		 			Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;

					VOL_Right_number--;
		 			Master_Right_Volume = MIN_Right_Volume - VOL_Right_number * GAP_Volume;

					//vs_set_volume(Master_Left_Volume, Master_Right_Volume);
					SET_LED_RED(1);
					SET_LED_GREEN(1);
					SET_LED_BLUE(1);
					if (VOL_Right_number <1 ) {
						VOL_Right_number=1;
					}
					PAM8019_Volume(VOL_Right_number*50+500);
					SET_LED_BLUE(0);
					//PAM8019_Volume(200);
					//while(1);
				}
				LED_timer_count = 0;
				tick_LED_timer_count = 0;
#if 0

				if (VOL_Left_number == 1 || VOL_Right_number == 1 ) {
					SetColor(RED);
				}
				else {
					SetColor(WHITE);
				}


				EE_Write(&DB_Vol_Left, VOL_Left_number);
				EE_Write(&DB_Vol_Right, VOL_Right_number);

				LED_timer_count = 0;
				tick_LED_timer_count = 0;
#endif
				break;
			case DOWN_middle_event:
			case DOWN_long_event:
				break;

			defualt:
				break;
		}
		BUTTON_event = NONE_push_event;
	}
}


int command_compare() {

	char	*ptr;
	int idx = 0;

	ptr = strtok(CmdBuf, " ");

	//printf("CmdBuf = %s\r\n", CmdBuf);
	
	while(ptr) {
		strcpy(argv[idx++], ptr);
		ptr = strtok(NULL," ");
	}

	argc = idx;
	for(idx=0;idx<NO_CMD;idx++)
	{
		if(strcmp(argv[0],rcmd[idx]) == 0) {
			return idx;
		}
	}
	return NO_CMD;

}



//////////////////////////////////////////////////////////////////////////////////
int HA_command_running( void ) {
	int rtn_value = -1;
	int RF_value;

	switch( Cmd_flag ) {
#if 0
		case REQ_VAL:
			printf("REQ_VAL\r\n");
			SineT_Req_Val();
			rtn_value = 1;
			break;
		case START:
			usbprintf("START\r\n");
			//SineT_Start();
			Sine_tone_generate = HIGH;
			rtn_value = 1;
			break;
		case VOLUME:
			Sine_tone_generate = HIGH;
			printf("VOLUME\r\n");
			SineT_Volume();
			rtn_value = 1;
			break;
		case STOP_SOUND:
			printf("STOP_SOUND\r\n");
			SineT_Stop_Sound();
			rtn_value = 1;
			break;
		case SAVE:
			//printf("SAVE\r\n");
			SineT_Save();
			printf("[GENIE]save OK\n");
			//printf("\n");
			rtn_value = 1;
			break;
		case EXIT:
			printf("EXIT\r\n");
			Sine_tone_generate = LOW;
			//recovery_Mic_input();
			rtn_value = 1;
			break;
		case PLAY:
			printf("PLAY\r\n");
			//SineT_Play();
			rtn_value = 1;
			break;
		//////////////////////////////////////////////////

		case RDM:
			Read_DSP_Memory();
			//USART_Tx(USART1, 'b');
			rtn_value = 1;
			break;
		case WRM:
			Write_DSP_Memory();
			rtn_value = 1;
			break;
		case TM:
			Read_T_Memory();
			rtn_value = 1;
			break;
#endif



		case GLE_COMMAND:
			rtn_value = 1;
			break;

		case CIS_CONNECTED:
		        rtn_value = 1; // ���ɾ� ���� ����
			    break;

		case CIS_DISCONNECTED:
		        rtn_value = 1; // ���ɾ� ���� ����

			    break;


		case T_NSINE:
			//t_newsine_f();
			rtn_value = 1;
			break;
		case T_SINE:
			//t_sine_f();
			rtn_value = 1;
			break;
		case VOL:
			//volume_f();
			rtn_value = 1;
			break;
		case EQ:	// 
			//printf("EQ\r\n");
			equalizer_7band();
			rtn_value = 1;
			break;
		
		case VOLUME1:	// 
		case VOLUME2:	// 
		case VOLUME3:	// 
			RF_value = (strtol(argv[1], NULL, 10));
			if (RF_value == 0) {
				Pparam1 = (signed int) ((strtol(argv[2], NULL, 10))*256);
//				printf("EQ param1=%i\r\n", Pparam1);
			}
			else if (RF_value == 1) {
				Pparam2 = (signed int) ((strtol(argv[2], NULL, 10))*256);
//				printf("EQ param2=%i\r\n", Pparam2);
			}
			else if (RF_value == 2) {
				Pparam3 = (signed int) ((strtol(argv[2], NULL, 10))*256);
//				printf("EQ param3=%i\r\n", Pparam3);
			}
			else if (RF_value == 3) {
				Pparam4 = (signed int) ((strtol(argv[2], NULL, 10))*256);
//				printf("EQ param4=%i\r\n", Pparam4);
			}
			else if (RF_value == 4) {
				Pparam5 = (signed int) ((strtol(argv[2], NULL, 10))*256);
//				printf("EQ param5=%i\r\n", Pparam5);
			}
			else if (RF_value == 5) {
				Pparam6 = (signed int) ((strtol(argv[2], NULL, 10))*256);
//				printf("EQ param6=%i\r\n", Pparam6);
			}
			else if (RF_value == 6) {
				Pparam7 = (signed int) ((strtol(argv[2], NULL, 10))*256);
//				printf("EQ param7=%i\r\n", Pparam7);
			}
			else {
			}
			
//			printf("EQ\r\n");
			equalizer_7band();
			rtn_value = 1;
			break;

/*		
		case GET_EEPROM:
			printf("EQ\r\n");
			equalizer_7band();
			rtn_value = 1;
			break;
		case FILTER:
			printf("EQ\r\n");
			equalizer_7band();
			rtn_value = 1;
			break;

		case MODEL_NAME:
			printf("EQ\r\n");
			equalizer_7band();
			rtn_value = 1;
			break;
*/			
		case DEVSTAT:
			//printf("[CMD] DEVSTAT = %s\r\n",argv[1] );
			rtn_value = 1;
			break;
		case A2DPSTAT:

			a2dpstat =  atoi(argv[1]);
			if (  a2dpstat == 4) {
				blink_color = BLUE;
				Blink_Count = 3;
				LED_Blink_ON_timer_count = 0;
			}
			//printf("[CMD] A2DPSTAT = %s\r\n",argv[1] );

			rtn_value = 1;
			break;
		case NSPK:
			//printf("EQ\r\n");
			equalizer_7band();
			rtn_value = 1;
			break;

		default :
			//usbprintf("BT Test\r\n");
			//USART_Tx(USART1, 'a');
			rtn_value = -1;
			break;
	}
	return rtn_value;
}




void SineTone_event_loop() {
	// Sine test Tone Generate


	if (Sine_tone_interval == 1) {	// 1000 msec timer


	#if 0

				if (flag_cnt == 0) {
					if (timer_vol < 250) timer_vol++;
					else flag_cnt = 1;
				}
				else {
					if (timer_vol > 1) timer_vol--; 
					else flag_cnt = 0;
				}
					
				usbprintf("timer_vol = %d\r\n", timer_vol);
				vs_set_volume(timer_vol, timer_vol);
	#endif

		//Sine_tone_generate = HIGH;
		if(Sine_tone_generate == HIGH) {
			//usbprintf("Signed Tone Event ON : Sine_tone_state = %d\r\n", Sine_tone_state);
		 	Sine_tone_state = LOW;	//test
			if (Sine_tone_state == LOW) Sine_tone_state= HIGH;
			else Sine_tone_state= LOW;


			if(Sine_tone_state == HIGH) {
				//susbprintf(str1, "Changed EQ_value[%d][%d]= %d", leftright_select, frequency_index, volume_value); Serial.println(str1);

				volume_value = (unsigned char) EQ_value[leftright_select][frequency_index];
				//usbprintf("Signed Tone On, Vol = %02x \r\n", volume_value);
				if(leftright_select == 0) {
					vs_set_volume(volume_value, 0xfe);
				}
				else {
					vs_set_volume(0xfe, volume_value);
				}
			}
			else {
				usbprintf("Tone Off\r\n");
				WriteSci(SCI_VOL, 0xFEFE);
			}

		}
		Sine_tone_interval = 0;
	}
}

void	battery_check() {
}

void 	Connector_check() {
/*
 *
	currentState_LineMic = ioport_get_pin_level(MIC_LINE); // 1:MIC, 0:Line Input
	currentState_USB_Power = ioport_get_pin_level(USB_POWER); 
	//usbprintf("Check Connector\r\n");

	if (currentState_LineMic != lastState_LineMic){
		if( currentState_LineMic == LOW) {	// Connected : Line In
			// Need to check new board lsh_test
			// ioport_set_pin_level(ANALOG_SW_IN1, HIGH);
			// ioport_set_pin_level(ANALOG_SW_IN2, LOW);
		}
		else {	// Not Connected

            // Need to check new board lsh_test

			// ioport_set_pin_level(ANALOG_SW_IN1, LOW);
			// ioport_set_pin_level(ANALOG_SW_IN2, HIGH);

		}
		__delay_ms(50);//crude form of button debouncing
		//usbprintf("Mic/Line Value : %d\r\n", currentState_LineMic);
		recovery_Mic_input();
	}

	if (currentState_USB_Power != lastState_USB_Power) {
		
//					// pwm_start(COLOR_R, 100);
//					// pwm_start(COLOR_G, 100);
//					// pwm_start(COLOR_B,  100);

		if( currentState_USB_Power == LOW) {	// Connedted USB Power Data Cable
			//currentState_BAT_STAT1 =  ioport_get_pin_level(BATTERY_STAT1); 
			//currentState_BAT_STAT2 =  ioport_get_pin_level(BATTERY_STAT2); 
			if(currentState_BAT_STAT1 == LOW && currentState_BAT_STAT2 == HIGH) {	
				//// pwm_start(COLOR_R,  50);
			}

			 GPIO_PinOutClear(port, pin);

			 ioport_set_pin_level(ANALOG_SW_IN1, LOW);
			 ioport_set_pin_level(ANALOG_SW_IN2, HIGH);

			//// pwm_start(COLOR_R,  0);
			//// pwm_start(COLOR_G, 100);
			//// pwm_start(COLOR_B,  100);

			// LOW Power Mode
			WriteSci(SCI_CLOCKF, 0x0000);
			WriteSci(SCI_VOL, 0xFFFF);
			WriteSci(SCI_AUDATA, 0x0010);
			// LOW Power Mode ; END

		}
		else {// Not Connected
			 ioport_set_pin_level(ANALOG_SW_IN1, LOW);
			 ioport_set_pin_level(ANALOG_SW_IN2, HIGH);
//			 ioport_set_pin_level(ANALOG_SW_IN1, LOW);
//			 ioport_set_pin_level(ANALOG_SW_IN2, LOW);
			//// pwm_start(COLOR_R,  100);

			//// pwm_start(COLOR_R,  100);
			//// pwm_start(COLOR_G, 0);
			//// pwm_start(COLOR_B,  100);
			recovery_Mic_input();

		}
		__delay_ms(50);//crude form of button debouncing
	}
	else {
		if( currentState_USB_Power == LOW) {	// Connedted USB Power Data Cable
			if( currentState_BAT_STAT1 == HIGH && currentState_BAT_STAT2 == LOW) {	
				//// pwm_start(COLOR_R,  100);
			}
		}
	}


	lastState_LineMic = currentState_LineMic;
	lastState_USB_Power = currentState_USB_Power;
*/
}

int w_time=0,counts=10;
long vvv=0;
int mmm;

void Timer_event_loop() {
/*
	if (ADC_Battery_timer == 1) {	// 2sec timer
		//adc_enable(&MY_ADC);
		adc_start_conversion(&MY_ADC, MY_ADC_CH);
		adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);
		ch0_result = adc_get_result(&MY_ADC, MY_ADC_CH);

		mmm=Get_mVolt_ADValue(ch0_result);
		//usbprintf("%4d mV\t", mmm);
		//usbprintf("%4d\n\r", ch0_result);

		ADC_Battery_timer = 0;
	}

	if (event_timer_alert_bat_red == 1) {

		if (ALERT_RED_LED_BATTERY == HIGH) {	
			// pwm_start(COLOR_R, 0);
			// pwm_start(COLOR_G, 100);
			// pwm_start(COLOR_B,  100);

			ALERT_RED_LED_BATTERY = LOW; 
		}
		else {	
			// pwm_start(COLOR_R, 100);
			// pwm_start(COLOR_G, 100);
			// pwm_start(COLOR_B,  100);
			ALERT_RED_LED_BATTERY = HIGH; 
		}

#if 0
		w_time--;
		if(w_time==0)
		{
			vvv /=counts;
			mmm=Get_mVolt_ADValue(vvv);
			usbprintf("%4d mV\t", mmm);
			usbprintf("%4d\n\r", vvv);
			w_time=counts;
			vvv=0;
		}		
		vvv += ch0_result;	
#endif

		event_timer_alert_bat_red = 0;
	}


	if (event_timer == 1) {	// 200msec timer
		Connector_check();
		event_timer = 0;
	}

#if 1
	if (LED_Off_Timer == 1) {	// 200msec timer

		//SCI_Aictrol1 = ReadSci(SCI_AICTRL1);
		//SCI_Aictrol3 = ReadSci(SCI_AICTRL3);
		//usbprintf("SCI_AICTRL1= 0x%04X,  SCI_AICTRL3= 0x%04X\r\n", SCI_Aictrol1, SCI_Aictrol3);
		//if( SCI_Aictrol1 == 1) WriteSci(SCI_AICTRL1, 0);

	
		if(RED_LED_STATE ==HIGH) {
			RED_LED_STATE = LOW;
			// pwm_start(COLOR_R,  100);
			//pwm_stop(COLOR_R);
		}
		if(GREEN_LED_STATE == HIGH) {
			GREEN_LED_STATE = LOW;
			// pwm_start(COLOR_G,  100);
			//pwm_stop(COLOR_G);
		}
		if(BLUE_LED_STATE == HIGH) {
			BLUE_LED_STATE = LOW;
			// pwm_start(COLOR_B,  100);
			//pwm_stop(COLOR_B);
		}

		LED_Off_Timer = 0;
	}
#endif

	
#if 1
	if (DSP_ADC_DECIM_Timer == 1) {

		WriteSci(SCI_WRAMADDR, ADC_CONTROL);
		WriteSci(SCI_WRAM, ADC_ENABLE | ADC_DECIM_FACTOR_48K);

		WriteSci(SCI_WRAMADDR, ADC_DATA_LEFT);
		dsp_adc_decim_data_left = ReadSci(SCI_WRAM);


		if(adc_cnt == 9) adc_cnt = 0;
		dsp_adc[adc_cnt] = 0xffff - dsp_adc_decim_data_left;
		//dsp_adc_decim_data_left = 0xffff - dsp_adc_decim_data_left;
		//usbprintf("%04x,\t", dsp_adc[adc_cnt]);

		
		adc_cnt++;

		dsp_adc_sum = 0;
		int isum;
		for( isum=0; isum <10; isum++) {
			dsp_adc_sum = dsp_adc_sum + dsp_adc[isum];
			//usbprintf("0x%04X,\t", dsp_adc[isum]);
		}
		//usbprintf("\n\r");
		unsigned int check = dsp_adc_sum>>16&0xFFFF;
		
		if (check > 2) {
			usbprintf("0x%08X ", dsp_adc_sum>>16&0xFFFF);
			usbprintf("0x%08X\n\r", dsp_adc_sum&0xFFFF);
			DSP_Act_Num++;
			DSP_Act_Cnt=100;
			if (DSP_Act_Num >= 3 && DSP_Act_State == 0) { 
				DSP_Act_State = 1;
				DSP_Act_Num = 0;
				usbprintf("O");
				vs_set_volume(Master_Volume, Master_Volume);
			}
		}		

		//usbprintf("CNT %d\n\r", DSP_Act_Cnt);
		//usbprintf("-");
		if(DSP_Act_Cnt >= 1) DSP_Act_Cnt--;
		if(DSP_Act_Cnt == 0  && DSP_Act_State == 1) {
			DSP_Act_State = 0;
			usbprintf("X");
			vs_set_volume(0xff, 0xff);
		}

			
		//usbprintf("DAC_left = 0x%04X,\n\r", dsp_adc_decim_data_left);
		//usbprintf("0x%04X,\n\r", dsp_adc_decim_data_left);
		//usbprintf("DAC_left = 0x%04X,\t", ReadSci(SCI_WRAM));
		//dsp_adc_decim_data_right = ReadSci(SCI_WRAM);
		//usbprintf("dsp_adc_decim_data_right = 0x%04X\r\n", dsp_adc_decim_data_right);
		DSP_ADC_DECIM_Timer = 0;
	}
#endif
*/
//Need to check new board lsh_test

}


// USART0 Receiver buffer
#define RX_BUFFER_SIZE0 20

char rx_buffer0[RX_BUFFER_SIZE0];
unsigned char rx_wr_index0,rx_rd_index0,rx_counter0;


#define  USART_BT USARTE0

/*
//Need to check new board lsh_test
ISR(USARTE0_RXC_vect)
{
	char status,data;
	data = USARTE0.DATA;
	rx_buffer0[rx_wr_index0]=data;
	//usbprintf("rx_counter0 = %d, %c\n\r", rx_counter0, data);
	//usbprintf("rx_wr_index0 = %d, %c\n\r", rx_wr_index0, data);
	if (++rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
	if (++rx_counter0 == RX_BUFFER_SIZE0)
	{
		rx_counter0=0;
		//rx_buffer_overflow0=1;
	}
}
*/



char getchar_bt()
{
	char data;
	//while (rx_counter0==0);
	data=rx_buffer0[rx_rd_index0];
	//usbprintf("rx_rd_index0 = %x, %c\n\r", rx_rd_index0, data);
	//usbprintf("rx_counter0 = %d, %c\n\r", rx_counter0, data);
	if (++rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
	--rx_counter0;
	return data;
}


//#define USART_SERIAL_BAUDRATE_BT 115200 //9600 // for BT
//#define USART_SERIAL_CHAR_LENGTH USART_CHSIZE_8BIT_gc
//#define USART_SERIAL_PARITY USART_PMODE_DISABLED_gc
//#define USART_SERIAL_STOP_BIT false

//bool test_tick=LOW;

void tick_callback()
{

	//  tick_timer_count = standard_timer_count;
	tick_timer_count++;


	BTAPP_Timer1MS_event();
	BLE_Timer1MS_event();
	BT_CommandSend1MS_event();
	BT_CommandDecode1MS_event();
	KEY_Timer1MS_event();
    AnalogAudioDetect_Timer1MS_event();
}



void main_loop() {
	//Button_event_loop();
	Button_Event_task();
	ticktimer_callback();
	//usbprintf("2017 1 30 TEST USB TX  ");
	if(	incoming_cmd_flag == 1 ) {
		command_check();
		incoming_cmd_flag = 0;
	}
}
