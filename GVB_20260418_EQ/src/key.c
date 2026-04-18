/*
 * key.c
 *
 *  Created on: 2018. 3. 13.
 *      Author: ki
 */


#include <stdbool.h>
#include <stdint.h>
#include <em_gpio.h>

#include "key.h"
#include "em_usart.h"
#include "HA_DefinedValues.h"
#include "HA_MCU_cmd.h"
#include "HA300_BSP.h"
#include "LED_PWM.h"

int BUTTON_event;
bool Old_Up_Key_Button_state=0;   // 0 is release state, 1 is push state
bool Old_Down_Key_Button_state=0;   // 0 is release state, 1 is push state
bool Old_Mode_Key_Button_state=0;   // 0 is release state, 1 is push state

extern bool BT_working_flag;

void KEY_Timer1MS_event( void )
{


    if(KEY_timer1ms){
//        KEY_count_timer1ms++;
		KEY_Button_timer1ms++;
		if( KEY_Button_timer1ms > BOUND_KEY_DELAY) // LED effect for Key inputs
		{
			if( KEY_Button_timer1ms < SHORT_KEY_DELAY ) { 
				//SetColor(GREEN);	//	SET_LED_RED(1); SET_LED_GREEN(0); SET_LED_BLUE(1);
			}
			else if( KEY_Button_timer1ms < MIDLE_KEY_DELAY ) { 
				SetColor(WHITE);		//	SET_LED_RED(1); SET_LED_GREEN(1); SET_LED_BLUE(0);   //20230904 test
			}
			else { 
				SetColor(RED);	//	SET_LED_RED(1); SET_LED_GREEN(1); SET_LED_BLUE(0);
			}
		}else {
			SetColor(BLACK);	//	SET_LED_RED(1); SET_LED_GREEN(1); SET_LED_BLUE(1);
			}
   	}

    /*if(TimeCommandReq.ReqDisableDiscoverableTimer)
        --TimeCommandReq.ReqDisableDiscoverableTimer;*/
}

void KeyScan( uint32_t interruptMask)
{
	bool Key_Button_state=0;   // 0 is release state, 1 is push state

  /* Act on interrupts */
  if (interruptMask & (1 << UP_BUTTON_PIN))
  {
  	  Key_Button_state = GPIO_PinInGet(UP_BUTTON_PORT, UP_BUTTON_PIN);
	  if(!Key_Button_state)
	  {
		  KEY_timer1ms = true;		  KEY_Button_timer1ms = 0;
	  }else
	  {
		  if( KEY_Button_timer1ms > BOUND_KEY_DELAY)
		  {
		   	   	   if( KEY_Button_timer1ms < SHORT_KEY_DELAY ) BUTTON_event = UP_push_event;
		   	  else if( KEY_Button_timer1ms < MIDLE_KEY_DELAY ) BUTTON_event = UP_middle_event;
		   	  else BUTTON_event = UP_long_event;
		  } 
		  KEY_timer1ms = false;

	  }
  }

  if (interruptMask & (1 << DOWN_BUTTON_PIN))
  {
	  Key_Button_state = GPIO_PinInGet(DOWN_BUTTON_PORT, DOWN_BUTTON_PIN);
	  if(!Key_Button_state)
	  {
		  KEY_timer1ms = true;		  KEY_Button_timer1ms = 0;
	  }else
	  {
		  if( KEY_Button_timer1ms > BOUND_KEY_DELAY)
		  {
		   	  if( KEY_Button_timer1ms < SHORT_KEY_DELAY ) BUTTON_event = DOWN_push_event;
		   	  else if( KEY_Button_timer1ms < MIDLE_KEY_DELAY ) BUTTON_event = DOWN_middle_event;
		   	  else BUTTON_event = DOWN_long_event;
		  } 
		  KEY_timer1ms = false;
	  }
  }

  if (interruptMask & (1 << MODE_BUTTON_PIN))
  {
  	  Key_Button_state = GPIO_PinInGet(MODE_BUTTON_PORT, MODE_BUTTON_PIN);
	  if(Key_Button_state)
	  {
		  KEY_timer1ms = true;		  KEY_Button_timer1ms = 0;
	  }else
	  {
		  if( KEY_Button_timer1ms > BOUND_KEY_DELAY)
		  {
		   	   	   if( KEY_Button_timer1ms < SHORT_KEY_DELAY ) BUTTON_event = MODE_push_event;
		   	  else if( KEY_Button_timer1ms < MIDLE_KEY_DELAY ) BUTTON_event = MODE_middle_event;
		   	  else BUTTON_event = MODE_long_event;
		  } 
		  KEY_timer1ms = false;
	  }
  }
//  if(BUTTON_event) { SET_LED_RED(1); SET_LED_GREEN(1); SET_LED_BLUE(1); } // Clear All LEDs.
}


void KeyScan_Button()
{
	bool Up_Key_Button_state=0;   // 0 is release state, 1 is push state
	bool Down_Key_Button_state=0;   // 0 is release state, 1 is push state
	bool Mode_Key_Button_state=0;   // 0 is release state, 1 is push state

	int i;

	/* Act on interrupts */
	  Up_Key_Button_state = GPIO_PinInGet(UP_BUTTON_PORT, UP_BUTTON_PIN);
	  if(Up_Key_Button_state != Old_Up_Key_Button_state) {
		  if(!Up_Key_Button_state)
		  {
			  KEY_timer1ms = true;		  
			  KEY_Button_timer1ms = 0;
			  if (BT_working_flag == false) {
				vs_set_volume(0xFE, 0xFE);
			  }
		  }else
		  {
			  if( KEY_Button_timer1ms > BOUND_KEY_DELAY)
			  {
			   	  if( KEY_Button_timer1ms < SHORT_KEY_DELAY ) BUTTON_event = UP_push_event;
			   	  else if( KEY_Button_timer1ms < MIDLE_KEY_DELAY ) BUTTON_event = UP_middle_event;
			   	  else BUTTON_event = UP_long_event;
			  } 
			  KEY_timer1ms = false;
			  if (BT_working_flag == false) {
#if 0
					for(i=0; i<=VOL_Left_number;i++) {
						for(i=0; i<=VOL_Right_number;i++) {
							delayMs(20);
							Master_Left_Volume = MIN_Left_Volume - i * GAP_Volume;
							Master_Right_Volume = MIN_Right_Volume - i * GAP_Volume;
//							vs_set_volume(Master_Volume, Master_Volume);
							vs_set_volume(Master_Left_Volume, Master_Right_Volume);
						}
					}
#else
							Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;
							Master_Right_Volume = MIN_Right_Volume -VOL_Right_number * GAP_Volume;

			delayMs(100);
			vs_set_volume(Master_Left_Volume, Master_Right_Volume);
#endif
			} 
		  }
	  }

	  Down_Key_Button_state = GPIO_PinInGet(DOWN_BUTTON_PORT, DOWN_BUTTON_PIN);
	  if(Down_Key_Button_state != Old_Down_Key_Button_state) {
		  if(!Down_Key_Button_state)
		  {
			  KEY_timer1ms = true;		  
			  KEY_Button_timer1ms = 0;
			  if (BT_working_flag == false) {
				vs_set_volume(0xFE, 0xFE);
			  }
		  }else
		  {
			  if( KEY_Button_timer1ms > BOUND_KEY_DELAY)
			  {
			   	  if( KEY_Button_timer1ms < SHORT_KEY_DELAY ) BUTTON_event = DOWN_push_event;
			   	  else if( KEY_Button_timer1ms < MIDLE_KEY_DELAY ) BUTTON_event = DOWN_middle_event;
			   	  else BUTTON_event = DOWN_long_event;
			  } 
			  KEY_timer1ms = false;
			  if (BT_working_flag == false) {
#if 0
					for(i=0; i<=VOL_Left_number;i++) {
						for(i=0; i<=VOL_Right_number;i++) {
							delayMs(20);
							Master_Left_Volume = MIN_Left_Volume - i * GAP_Volume;
							Master_Right_Volume = MIN_Right_Volume - i * GAP_Volume;
//							vs_set_volume(Master_Volume, Master_Volume);
							vs_set_volume(Master_Left_Volume, Master_Right_Volume);
						}
					}
#else
							Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;
							Master_Right_Volume = MIN_Right_Volume -VOL_Right_number * GAP_Volume;


			delayMs(100);
			vs_set_volume(Master_Left_Volume, Master_Right_Volume);
#endif
			} 
		  }
	  }

	  Mode_Key_Button_state = GPIO_PinInGet(MODE_BUTTON_PORT, MODE_BUTTON_PIN);
	  if(Mode_Key_Button_state != Old_Mode_Key_Button_state) {
#if defined( GN1_PCB2ND ) || defined( GN1_PCB3rd )
		  if(!Mode_Key_Button_state)
#else
		  if(Mode_Key_Button_state)
#endif
		  {
			  KEY_timer1ms = true;		  
			  KEY_Button_timer1ms = 0;
			  if (BT_working_flag == false) {
				vs_set_volume(0xFE, 0xFE);
			  }
		  }else
		  {
			  if( KEY_Button_timer1ms > BOUND_KEY_DELAY)
			  {
			   	   	   if( KEY_Button_timer1ms < SHORT_KEY_DELAY ) BUTTON_event = MODE_push_event;
			   	  else if( KEY_Button_timer1ms < MIDLE_KEY_DELAY ) BUTTON_event = MODE_middle_event;
			   	  else BUTTON_event = MODE_long_event;
			  } 
			  KEY_timer1ms = false;
			  if (BT_working_flag == false) {

#if 0
					for(i=0; i<=VOL_Left_number;i++) {
						for(i=0; i<=VOL_Right_number;i++) {
							delayMs(20);
							Master_Left_Volume = MIN_Left_Volume - i * GAP_Volume;
							Master_Right_Volume = MIN_Right_Volume - i * GAP_Volume;
//							vs_set_volume(Master_Volume, Master_Volume);
							vs_set_volume(Master_Left_Volume, Master_Right_Volume);
						}
					}
#else
							Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;
							Master_Right_Volume = MIN_Right_Volume -VOL_Right_number * GAP_Volume;


			delayMs(100);
			vs_set_volume(Master_Left_Volume, Master_Right_Volume);
#endif
			} 
		  }
	  }

	  Old_Up_Key_Button_state = Up_Key_Button_state;
	  Old_Down_Key_Button_state = Down_Key_Button_state;
	  Old_Mode_Key_Button_state = Mode_Key_Button_state;
	//  if(BUTTON_event) { SET_LED_RED(1); SET_LED_GREEN(1); SET_LED_BLUE(1); } // Clear All LEDs.
}



