/*
 * LED_PWM.h
 *
 *  Created on: 2018. 10. 19.
 *      Author: ki
 */

#ifndef LED_PWM_H_
#define LED_PWM_H_


// Define PWM frequency value
#define PWM_FREQ	20000
#define PWM_MAX 	1000
#define PWM_10		100
#define PWM_20		200
#define PWM_30		300
#define PWM_40		400
#define PWM_50		500
#define PWM_60		600
#define PWM_70		700
#define PWM_80		800
#define PWM_90		900
#define PWM_100		1000


// Define PWM Color channel
#define PWM_RED 	0
#define PWM_GREEN 	1
#define PWM_BLUE 	2

#define PAM8019_VOL 2


#define ZERO_Value 	0

extern char blink_color;


//uint8_t Rainbow_RGB[8][3] =
//{
//	{0,   100, 100},	//RED
//	{0,    64, 100},	//ORG
//	{0,     0, 100},	//YEL
//	{100,  50, 100},	//GRN
//	{100,  50,   0},	//CYN
//	{100, 100,   0},	//BLU
//	{ 20, 100,  80},	//VLT
//	{ 50,  50,  50}		//WHT
//};

enum {
    BLACK,	//0
    RED,	//1
    ORANGE,	//2
    YELLOW,	//3
    GREEN,	//4
    CYAN,	//5
    BLUE,	//6
    INDIGO,	//7
    MAGENTA,//8
    VIOLET,	//9
    WHITE	//10
};

void PWM_init();
void init_timer1(void);
void pwm_start(unsigned int ch, uint32_t duty_cycle);
void pwm_LED(uint32_t RedDuty, uint32_t GreenDuty, uint32_t BlueDuty);
void SetColor(char col);
void PWM_PortON(void);
void PWM_PortOFF(void);
void  PAM8019_Volume(uint32_t amp_volume);

#endif /* LED_PWM_H_ */
