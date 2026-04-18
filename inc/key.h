/*
 * key.h
 *
 *  Created on: 2018. 3. 13.
 *      Author: ki
 */

#ifndef KEY_H_
#define KEY_H_

//
// BUTTON pushed delays
//
#define BOUND_KEY_DELAY   30	// de-bounds delay
#define SHORT_KEY_DELAY 2000	// short  key required delay
#define MIDLE_KEY_DELAY 8000	// middle key required delay
#define LONGS_KEY_DELAY 10000	// long   key required delay

//
// bit3~2 button code numbers
// bit1~0 each button mode code numbers
//
#define NONE_push_event 	0b0000	// 0
#define UP_push_event   	0b0100	// 4
#define DOWN_push_event 	0b1000	// 8
#define MODE_push_event 	0b1100	// 12
#define UP_middle_event 	0b0101	// 5
#define DOWN_middle_event	0b1001	// 9
#define MODE_middle_event	0b1101	// 13
#define UP_long_event 		0b0110	// 6
#define DOWN_long_event 	0b1010	// 10
#define MODE_long_event 	0b1110	// 14

#define BACK_to_event 		0b1111	// 15

//#define DEBUG_KEY	// show Key response for USART_Tx(USART1, x);

void KeyScan( uint32_t imask);
void KEY_Timer1MS_event( void );
void KeyScan_Button(void);

//extern  int BUTTON_event;
extern int BUTTON_event;

#endif /* KEY_H_ */
