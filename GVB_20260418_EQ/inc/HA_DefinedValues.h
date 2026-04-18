/*
 * HA_DefinedValues.h
 *
 *  Author: Rapiarno
 */ 

#ifndef HA_DEFINEDVALUE_H
#define HA_DEFINEDVALUE_H

#include <stdbool.h>
#include "compiler.h"


#include "em_usb.h"

//#define GN1_MASTER        /* Compile for MASTER device mode. */
#define GN1_SLAVE         /* Compile for SLAVE device mode. */

#define GN1_PCB3rd        /* Compile for MASTER device mode. */

#define HA_MODE         /* Compile for Hearing Aids MODEL device mode. */

#define HA_SPP       	/* SPP Com for DSP EQ by smartphone */

//#define DBG_PRINT         /* Compile for SLAVE device mode. */

#define MULTI_USER       	/* SPP Com for DSP EQ by smartphone */

#define CONFIG_SLEEPMGR_ENABLE

#define EFM32_ADC_ENABLE

#define SV2_TX       	/* SV2 TX */
//#define SV2_RX       	/* SV2_RX */


extern char txbuffer[];

//#define usbprintf(...) do { \
//		delayMs(10); \
//        sprintf(txbuffer, __VA_ARGS__); \
//        puts(txbuffer); \
//    } while (0)

#define printf(...) do { \
		delayMs(10); \
        sprintf(txbuffer, __VA_ARGS__); \
        puts(txbuffer); \
    } while (0)

static uint32_t  tmpBuffer;
static uint32_t usb_getc( ){
    USBD_Read(CDC_EP_DATA_OUT, (void*)&tmpBuffer, 1, NULL);
    return tmpBuffer;
}

//#define BOARD_XOSC_HZ          4000000
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//#define BOARD_XOSC_STARTUP_US  500000

//#define CONF_BOARD_ENABLE_USARTD0

#define USB_RX_BUF_SIZE                    150
#define COMMAND_BUF_SIZE                    200

#define EEPROM_7EQ_base_nAddr 			    0x00
#define EEPROM_Master_Volume_nAddr 		    0x07
#define EEPROM_SELECTED_EQ_VALUE_nAddr 	    0x0F
#define EEPROM_1st_profile_EQ_base_nAddr 	0x10
#define EEPROM_GAIN_DATA_nAddr 			    0x17
//#define EEPROM_print_nAddr                0x10
#define EEPROM_DSP_AEC_Enable_nAddr 	    0x1F

#define USART_SERIAL_BT &USARTE0	// for Bluetooth UART
#define USART_SERIAL_BAUDRATE_BT 115200 //19200 // 9600 //115200 //9600 // for BT
#define USART_SERIAL_CHAR_LENGTH USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT false

#define VOLUME_LED_OFF_TIMER 	200

static volatile bool main_b_cdc_enable = false;

///////////////////////////////////////////


#  define ADC_CH0    (1U << 0)                 /**< ADC channel 0. */
#  define ADC_CH1    (1U << 1)                 /**< ADC channel 1. */
#  define ADC_CH2    (1U << 2)                 /**< ADC channel 2. */
#  define ADC_CH3    (1U << 3)                 /**< ADC channel 3. */

#define MY_ADC         1  //ADCA
#define MY_ADC_CH    (1U <<0)  //ADC_CH0


#define COLOR_R   0
#define COLOR_G   1
#define COLOR_B   2



///////////////////////////////////////////
//extern bool HA100_system_ready; //sound mute state by Power Short Button
extern long BT_ON_timer_count;
extern long BT_NSPK_ON_timer_count;

extern uint8_t	BT_nspk_link;
extern uint8_t	BT_nspk_event;
extern uint8_t BT_nspk_status;
extern uint8_t Print_BT_CMD_SendState;

extern bool mic_mute_state;
extern bool BIS_state;

extern bool bt_dsp_reset ;


extern long NSPK_Voice_timer_count;
extern long tick_NSPK_Voice_timer_count;


extern long LED_timer_count;
extern long tick_LED_timer_count;

extern long Time_1SEC_timer_count;
extern long tick_1SEC_timer_count;

 extern long SET_VOL_timer_count;
extern long tick_SET_VOL_timer_count;

extern long SOUND_LEVEL_timer_count;
extern long tick_SOUND_LEVEL_timer_count;

extern long BAT_LEVEL_CHECK_timer_count;
extern long tick_BAT_LEVEL_CHECK_timer_count;

extern long RED_LED_timer_count;
extern long tick_RED_LED_timer_count;

extern long AB1565_Power_On_timer_count;
extern long tick_AB1565_Power_On_timer_count;

extern long LED_Blink_ON_timer_count;
extern long tick_LED_Blink_ON_timer_count;

extern long LED_Blink_OFF_timer_count;
extern long tick_LED_Blink_OFF_timer_count;

extern long DSP_INDEX_timer_count;
extern long AUDIOJACK_timer_count;

extern long tick_BT_ON_timer_count;
extern long tick_BT_NSPK_ON_timer_count;

static bool Sound_Mute_State = LOW; //sound mute state by Power Short Button

static bool record_mic_flag=LOW;
static int mic_count=0;

static int event_timer = 0;
static int event_timer_alert_bat_red = 0;	//bool event_value_500ms; // Power Indicator LED

extern bool Key_Button_state;

extern bool BAT_Sound_Flag;

extern bool Sound_Led_Flag;

extern	int a2dpstat;


static int HA100_state;
enum { INIT_MODE=0, HA_RUN_MODE, EQ_SELECT_MODE, EQ_SETUP_MODE, GAIN_SETUP_MODE };

static int ADC_Battery_timer = 0; //Battery Read interval timer
static int LED_Off_Timer= 0;
static int DSP_ADC_DECIM_Timer = 0;


static int Sine_tone_interval = 0; //Sine tone interval timer
static bool Sine_tone_generate = LOW;

static bool currentState_EQ7 = HIGH;
static bool lastState_EQ7 = HIGH;
static bool currentState_UP = HIGH;
static bool lastState_UP = HIGH;
static bool currentState_DOWN = HIGH;
static bool lastState_DOWN = HIGH;

static bool currentState_POWER = HIGH;
static bool lastState_POWER = HIGH;

//static int currentState_LineMic = 1;
static bool lastState_LineMic = LOW;

static int currentState_USB_Power;
static bool lastState_USB_Power = HIGH;


extern bool NC_Index;  // High NC for voice, Low Normal mode

static int currentState_BAT_STAT1;
static int currentState_BAT_STAT2;


static uint8_t DSP_AEC_Enable;

static bool ALERT_RED_LED_BATTERY = HIGH;//LOW = OFF, HIGH = ON


static bool USB_Power_LED = LOW;

static bool RED_LED_STATE = LOW;//LOW = OFF, HIGH = ON
static bool GREEN_LED_STATE = LOW;//LOW = OFF, HIGH = ON
static bool BLUE_LED_STATE = LOW;//LOW = OFF, HIGH = ON

#define MODE_SHORTPRESS_LEN    5  // Min nr of loops for a long press
#define MODE_LONGPRESS_LEN    100  // Min nr of loops for a long press
//#define MODE_SHORTPRESS_LEN    100  // Min nr of loops for a long press
//#define MODE_LONGPRESS_LEN    1000  // Min nr of loops for a long press
enum { MODE_EV_NONE=0, MODE_EV_SHORTPRESS, MODE_EV_LONGPRESS };
static bool MODE_button_was_pressed; // previous state
static int MODE_button_pressed_counter; // press running duration
static bool MODE_long_k_event;


#define VOLUP_SHORTPRESS_LEN    5  // Min nr of loops for a long press
#define VOLUP_LONGPRESS_LEN    600  // Min nr of loops for a long press
//#define VOLUP_SHORTPRESS_LEN    100  // Min nr of loops for a long press
//#define VOLUP_LONGPRESS_LEN    1000  // Min nr of loops for a long press
enum { VOLUP_EV_NONE=0, VOLUP_EV_SHORTPRESS, VOLUP_EV_LONGPRESS };
static bool VOLUP_button_was_pressed; // previous state
static int VOLUP_button_pressed_counter; // press running duration
static bool VOLUP_long_k_event;


#define VOLDOWN_SHORTPRESS_LEN   5  // Min nr of loops for a long press
#define VOLDOWN_LONGPRESS_LEN    600  // Min nr of loops for a long press
//#define VOLDOWN_SHORTPRESS_LEN    100  // Min nr of loops for a long press
//#define VOLDOWN_LONGPRESS_LEN   1000  // Min nr of loops for a long press
enum { VOLDOWN_EV_NONE=0, VOLDOWN_EV_SHORTPRESS, VOLDOWN_EV_LONGPRESS };
static bool VOLDOWN_button_was_pressed; // previous state
static int VOLDOWN_button_pressed_counter; // press running duration
static bool VOLDOWN_long_k_event;


//int MODE_event, VOLUP_event, VOLDOWN_event, BUTTON_event;
int MODE_event, VOLUP_event, VOLDOWN_event;


// SCI registers value
uint16_t SCI_Mode;
uint16_t SCI_Status;
uint16_t SCI_Bass;
uint16_t SCI_Clockf;
uint16_t SCI_Decode_time;
uint16_t SCI_Audata;
uint16_t SCI_Wram;
uint16_t SCI_Wramaddr;
uint16_t SCI_Hdat0;
uint16_t SCI_Hdat1;
uint16_t SCI_Aiaddr;
uint16_t SCI_Vol;
uint16_t SCI_Aictrol0;
uint16_t SCI_Aictrol1;
uint16_t SCI_Aictrol2;
uint16_t SCI_Aictrol3;

static unsigned char sine_test_on[] = {0x53, 0xef, 0x6e, 126, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };	// 5168Hz.
static unsigned char sine_test_off[] = {0x45, 0x78, 0x69, 0x74, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static unsigned char new_sine_test[] = {0x02, 0x0a, 0x40, 0x22};

#define EQ_MAX_NUMBER 7	//12
extern uint16_t EQ_Select_index, EQ_DB_index;		// EEPROM

extern uint16_t NC_DB_index;		// Noise reduction


//To set a 7-band filter which greatly emphasizes bass and treble, the following code could be used:
// 64 160 400 1000 2500 6250 15600 Hz
#if 0
static char EQ7_case[EQ_MAX_NUMBER][7] = { {25, 25, 	25, 	25, 	25, 	25, 	25}, // +5dB
{30,	30, 	30, 	30, 	30, 	38, 	30}, // +10dB {0, 0, 0, 0, 0, 0, 8, 0};  
{35,	35, 	30, 	40, 	35, 	35, 	25}, // +dB {15, 	15, 	10, 	20, 	15, 	15, 	5};  //base 20dB
{35, 35, 	30, 	30, 	40, 	35, 	30}, // +dB {15, 	15, 	10, 	10, 	20, 	15, 	10}; //base 20dB
{45,	45, 	40, 	35, 	25, 	30, 	30}, // +dB {25, 	25, 	20, 	15, 	5, 	10, 	10}; //base 20dB
{0, 	0, 	0, 	30, 	47, 	62, 	70}, // +dB {0, 	0, 	0, 	10, 	27, 	42, 	50}; //base 20dB
{0, 	0, 	0, 	40, 	70, 	80, 	75}, // +dB {0, 	0, 	0, 	20, 	50, 	60, 	55}; //base 20dB
{50, 55, 	50, 	45, 	47, 	52, 	50}, // +dB {40, 	45, 	40, 	35, 	37, 	42, 	40};  //base 10dB
{30,	35, 	52, 	58, 	70, 	70, 	70}, // +dB {30, 	35, 	52, 	58, 	70, 	70, 	70}; //base 0 dB
{50,	50, 	50, 	50, 	50, 	50, 	50}, // +30dB {70, 	70, 	70, 	70, 	70, 	70, 	70}; //base 0 dB
{30, 30, 	30, 	30, 	30, 	30, 	30}}; // +10dB
#endif

static signed char EQ7_Voice_case[EQ_MAX_NUMBER][7] = {  // -36...+18dB ( 0 ~ 44)
	// 64 160 400 1000 2500 6250 15600 Hz
{-20, -10, 0, 0, 0, 0, -20}, // CASE 2
{-20, -10, 1, 4, 5, 13, -20}, // CASE 1
{-20, -10, 12, 14, 15, 0, -20}, // CASE 2
{-20, -10, 14, 10, 0, 5, -20}, // CASE 3
{-20, -10, 2, 5, 13, 15, -20}, // CASE 4 
{-20, -10, 8, 11, 13, 16, -20}, // CASE 5
{-20, -10, 10, 6, 6, 14, -20} // CASE 6
};  					//   No 12


static signed char EQ7_case[EQ_MAX_NUMBER][7] = {  // -36...+18dB ( 0 ~ 44)
// 64 160 400 1000 2500 6250 15600 Hz
#if 0
{10, 10, 10, 10, 10, 10, -5}, // 증촉
{-2, -2, 12, 14, 15, 0, -2}, // CASE 2  // 음성
{5, 10, 10, 15, 20, 19, -5}, // CASE 2  // 저음1
{15, 15, 14, 10, 0, 5, 5}, // CASE 3  // 저음2
//{0, 0, 5, 10, 15, 18, -5}, // CASE 2   // 고주파 난청
{5, 5, 8, 11, 13, 16, -6}, // CASE 5   // 노인성 난청
{5, 5, 8, 11, 13, 16, 13}, // CASE 5  // 고음 
{15, 13, 10, 6, 6, 14, 13} // CASE 6  // 산형 
#else
{0, 0, 0, 0, 0, 0, 0}, // CASE 2
//{-5, 0, 15, 15, 15, 0, -5}, // CASE 2
{-1, 0, 10, 14, 15, 5, -2}, // CASE 1
{-2, -2, 12, 14, 15, 0, -2}, // CASE 2
{5, 10, 14, 10, 0, 5, 0}, // CASE 3
{-5, -4, 4, 10, 17, 14, 6}, // CASE 4    {0, 0, 5, 10, 18, 22, 25},
{-5, 0, 5, 14, 18, 16, 6}, // CASE 5
{15, 13, 10, 6, 6, 14, 5} // CASE 6
#endif
};  					//   No 12
//struct pwm_config pwm_cfg_red;
//struct pwm_config pwm_cfg_green;
//struct pwm_config pwm_cfg_blue;

static uint32_t tick_timer_count=0;

#endif
