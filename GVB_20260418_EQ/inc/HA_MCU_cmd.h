/*
 * HA_MCU_cmd.h
 *
 *  Author: Rapiarno
 */ 

#ifndef HA_MCU_CMD_H
#define HA_MCU_CMD_H



#define UP_BUTTON_PORT     (2)      //PC15
#define UP_BUTTON_PIN      (15)


#define DOWN_BUTTON_PORT     (2)      //PC14
#define DOWN_BUTTON_PIN      (14)

#define MODE_BUTTON_PORT     (0)      //PA2
#define MODE_BUTTON_PIN      (2)

#define LED_BLUE_PORT (1)      //PB13 
#define LED_BLUE_PIN  (13)

extern uint8_t DB_LED_Color[10];

extern char CmdBuf[COMMAND_BUF_SIZE];
extern char argv[10][20];
extern int Cmd_flag, argc;
extern int nRxCnt;
//unsigned char nRxCnt=0;

extern bool dsp_lowpower_flag;

extern char incomingByte;      // a variable to read incoming serial data into
extern char incomingByte_BT;      // a variable to read incoming serial data into
extern bool Sine_tone_state;
extern char incoming_cmd_flag;      // a variable to read incoming serial data into



extern uint8_t MAX_Volume,  MIN_Volume, MAX_VOL_Count, MIN_VOL_Count;
extern uint8_t VOL_Count, VOL_number;
extern uint8_t GAP_Volume;

extern uint8_t MAX_Left_Volume, MIN_Left_Volume, MAX_Left_VOL_Count, MIN_Left_VOL_Count;
extern uint8_t VOL_Left_Count, VOL_Left_number;

extern uint8_t MAX_Right_Volume, MIN_Right_Volume, MAX_Right_VOL_Count, MIN_Right_VOL_Count;
extern uint8_t VOL_Right_Count, VOL_Right_number;

extern uint8_t Master_Volume;
extern uint8_t Master_Left_Volume, Master_Right_Volume;
extern uint8_t Gain_Data;

extern uint8_t Rainbow_RGB[7][3];
extern char rcmd[30][20];


extern bool KEY_timer1ms;
extern uint16_t KEY_count_timer1ms;
extern uint16_t KEY_Button_timer1ms;
extern int LED_tmp;

extern uint32_t GN_Working_State;

 

extern uint8_t Group_Zone_Main, Group_Zone_Sub;
	

extern bool a2dpstat_LED;


 // static long tick_timer_count = 0; // 1ms timer counter

static unsigned char timer_vol=0, flag_cnt=0;


// KEY Start
////////////////////////////////////////////////////////////////////////////////

typedef struct
{
    uint8_t id;

    union
    {
        struct
        {
            uint8_t mode      :4;
            uint8_t state     :1;
        };

        uint8_t flags;
    };

    uint16_t time;

    uint16_t time_last_off; //if now is released, counting released time; if now is pressing, store last time released time(only for treble action)
    uint8_t last_action;    //KEY_EVENT_PRIMARY or KEY_EVENT_SECONDARY or KEY_EVENT_DOUBLE_CLICK
}
KEY_STATE;

////////////////////////////////////////////////////////////////////////////////

#define KEY_MODE_DISABLED       0
#define KEY_MODE_CONTINUOUS     1
#define KEY_MODE_AUTOREPEAT     2
#define KEY_MODE_SINGLE_ACTION  3
#define KEY_MODE_DUAL_ACTION    4
#define KEY_MODE_TREBLE_ACTION  5

#define KEY_EVENT_PRIMARY           1        //KEY_MODE_CONTINUOUS,KEY_MODE_AUTOREPEAT,KEY_MODE_SINGLE_ACTION,KEY_MODE_DUAL_ACTION.TREBLE action don't use this event
#define KEY_EVENT_SECONDARY         2        //KEY_MODE_DUAL_ACTION, KEY_MODE_TREBLE_ACTION support this event
#define KEY_EVENT_SINGLE_CLICK      3        //KEY_MODE_TREBLE_ACTION support only
#define KEY_EVENT_DOUBLE_CLICK      4        //KEY_MODE_TREBLE_ACTION support only
#define KEY_EVENT_SECONDARY_RELEASED    5       //KEY_MODE_DUAL_ACTION, assit to KEY_EVENT_SECONDARY event
#define ADDITIONAL_EVENT_HOLD_FOR_10S   0x10    //event that KEY is holding for 10s

#define KEY_SINGLE_ACTION_DELAY     1000     // all in ms
#define KEY_AUTOREPEAT_DELAY        500
#define KEY_AUTOREPEAT_RATE         100     // = 10 per sec
#define KEY_TREBLE_ACTION_OFF_DELAY 200

#define KEY_TASK_PERIOD         10

////////////////////////////////////////////////////////////////////////////////

// TODO change codes
enum{
    KEY_ID_MFB,
    KEY_ID_PLAY_PAUSE,
    KEY_ID_VOL_UP,
    KEY_ID_VOL_DN,
    KEY_ID_NEXT,
    KEY_ID_PREVIOUS,
    KEY_ID_SW22,
    KEY_ID_SW39,
    KEY_ID_SW40,
};

enum {
    GN_STATUS_IDLE,   //
    GN_STATUS_GROUP_ZONE,
    GN_STATUS_MSPK,		// "GroupSpeakerSoundSync" or "Add Third SPK"
    GN_STATUS_DB_INDEX,
    GN_STATUS_BT_OFF,
    GN_STATUS_BT_ON,
    GN_STATUS_L_VOLUME,
    GN_STATUS_R_VOLUME,
    GN_STATUS_TEST_MODE
}GN_WORKINGSTATE;


////////////////////////////////////////////////////////////////////////////////
void KEY_Timer1MS_event(void);
void KEY_Initialize (void );
void KEY_Task ( void );
void KEY_Process ( KEY_STATE *key, uint16_t time );

void KEY_Handler ( uint8_t key, uint8_t event );

////////////////////////////////////////////////////////////////////////////////
// KEY END

void 	Write_DSP_Memory();
void 	Read_DSP_Memory();
void 	Read_T_Memory();

unsigned int get_eeprom_f();
unsigned int t_sine_f();

int	MODE_button_handle();
int	VOL_UP_button_handle();
int	VOL_DOWN_button_handle();
void main_loop();
void Connector_check();
void Button_event_loop();
void SetFilter(unsigned int , unsigned int , double , double , signed int , signed int );
int setfilter_f();
void gpioSetup(void);
void Button_Event_task(void);
void Button_event_MSPK(void);
void Button_event_L_Volume(void);
void Button_event_R_Volume(void);
void Button_event_Group_Zone_ID(void);
void Button_event_DB_Index(void);
void Button_event_BT_ON(void);
void Button_event_Test_Mode(void);
void LED_RED_Blink_Count(int i);
void LED_GREEN_Blink_Count(int i);
void LED_BLUE_Blink_Count(int i);
#endif
