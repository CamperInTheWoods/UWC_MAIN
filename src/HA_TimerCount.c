/*
 * HA_TimerCount.c
 *
 *  Author: Rapiarno
 */ 


#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_int.h"
#include "em_timer.h"

#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"

#include "bt_app.h"

#include "HA_DefinedValues.h"
#include "HA_TimerCount.h"
#include "compiler.h"
#include "HA_MCU_cmd.h"

#include "HA300_BSP.h"
#include "key.h"
#include "EnergyMode.h"
#include "LED_PWM.h"

extern long E_SaveCount;
extern bool E_Save;
extern long E_1st_SaveCount;
extern bool E_1st_Save;


#define LED_PIN             (0)
#define LED_PORT            (gpioPortA)


volatile uint32_t systick_count = 0;
uint32_t clock_freq;

long tick_BT_ON_timer_count = 0;
long tick_BT_NSPK_ON_timer_count = 0;
long tick_LED_timer_count = 0;
long tick_SET_VOL_timer_count = 0;

long tick_1SEC_timer_count = 0;

 long tick_NSPK_Voice_timer_count = 0;

long tick_SOUND_LEVEL_timer_count = 0;
long tick_SOUND_LEVEL_CHECK_timer_count = 0;

long tick_DSP_INDEX_timer_count = 0;

long tick_RED_LED_timer_count = 0;

long tick_AB1565_Power_On_timer_count = 0;

long tick_LED_Blink_ON_timer_count = 0;
long tick_LED_Blink_OFF_timer_count = 0;

bool Sound_Led_Flag=0;

__STATIC_INLINE void __delay_tick(volatile uint32_t n);
__STATIC_INLINE uint32_t get_systick(void);

void setup_systick_timer(void);
void reset_blink(volatile uint8_t countdown);

void init_clocks(void);
void init_portio(void);

extern int BUTTON_event;
extern uint8_t  BT_linkIndex;

extern long tick_DSP_INDEX_timer_count;

long PI35_timer_count;
unsigned int currentState_3PI5=0;
unsigned int lastState_3PI5 = LOW;


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/



bool test_tick=LOW;

void SysTick_Handler(void)
{

//	USART_Tx(USART1, 'F');

	tick_callback();

#if 0
	if(test_tick == LOW) {	
	    GPIO_PinModeSet(gpioPortA, 2,     gpioModePushPull, 1);
		test_tick = HIGH;
	}
	else {
	    GPIO_PinModeSet(gpioPortA, 2,     gpioModePushPull, 0);
		test_tick = LOW;
	}
#endif

    systick_count++;

    if(	Time_1SEC_timer_count == 0 ) {

    	tick_1SEC_timer_count++;


    }

    if(	LED_timer_count == 0 ) {

    	tick_LED_timer_count++;
    }

    if(	RED_LED_timer_count == 0 ) {

    	tick_RED_LED_timer_count++;
    }

    if(	AB1565_Power_On_timer_count == 0 ) {

    	tick_AB1565_Power_On_timer_count++;
    }


    if(	NSPK_Voice_timer_count == 0 ) {

    	tick_NSPK_Voice_timer_count++;
    }

    if(	SET_VOL_timer_count == 0 ) {

    	tick_SET_VOL_timer_count++;
    }

    if(	SOUND_LEVEL_timer_count == 0 ) {

    	tick_SOUND_LEVEL_timer_count++;
    }

    if(	LED_Blink_ON_timer_count == 0 ) {

    	tick_LED_Blink_ON_timer_count++;
    }

    if(	LED_Blink_OFF_timer_count == 0 ) {

    	tick_LED_Blink_OFF_timer_count++;
    }

    if(	BT_ON_timer_count == 0 ) {

    	tick_BT_ON_timer_count++;
    }

    if(	BT_NSPK_ON_timer_count == 0 ) {

    	tick_BT_NSPK_ON_timer_count++;
    }

    if(	DSP_INDEX_timer_count == 0 ) {

    	tick_DSP_INDEX_timer_count++;
    }

    if(	AUDIOJACK_timer_count == 0 ) {
	PI35_timer_count++;
    }


}

__STATIC_INLINE uint32_t get_systick(void)
{
    return systick_count;
}

/******************************************************************************
 * @brief Delay function
 *****************************************************************************/
void __delay_ms(uint32_t milliseconds)
{
    uint32_t start_tick = get_systick();
    while ((get_systick() - start_tick) < milliseconds) {
    }
}


void setup_systick_timer(void)
{
    clock_freq = CMU_ClockFreqGet(cmuClock_CORE);
    /* Enable SysTick interrupt, necessary for __delay_ms() */
    if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

    NVIC_EnableIRQ(SysTick_IRQn);
}


void reset_blink(volatile uint8_t countdown)
{
    volatile uint8_t cnt = countdown / 2;
    while (cnt--) {
        // pwm_start(COLOR_R, 100);
        __delay_ms(20);
        // pwm_start(COLOR_R, 0);
        __delay_ms(20);
    }
    __delay_ms(1000);
}


void init_clocks(void)
{
    // $[HFXO]
    CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOMODE_MASK) | CMU_CTRL_HFXOMODE_XTAL;

    CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK)
            | CMU_CTRL_HFXOBOOST_50PCENT;

    SystemHFXOClockSet(32000000);
   // SystemHFXOClockSet(24000000UL);

    // $[Use oscillator source]
//    CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_LFXOMODE_MASK) | CMU_CTRL_LFXOMODE_XTAL;
    // [Use oscillator source]$

    // $[LFXO Boost Percent]
 //   CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_LFXOBOOST_MASK)
//            | CMU_CTRL_LFXOBOOST_100PCENT;
    // [LFXO Boost Percent]$

    // $[LFXO enable]
//    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
    // [LFXO enable]$

    // $[HFXO enable]
//    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
    // [HFXO enable]$

    // $[High Frequency Clock select]
    /* Using HFXO as high frequency clock, HFCLK */
//    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Enable peripheral clock */
    CMU_ClockEnable(cmuClock_HFPER, true);

    // [High Frequency Clock select]$

    // $[Peripheral Clock enables]
    /* Enable clock for TIMER0 */
    CMU_ClockEnable(cmuClock_TIMER0, true);

    /* Enable clock for GPIO by default */
    CMU_ClockEnable(cmuClock_GPIO, true);

    // [Peripheral Clock enables]$
}


void init_portio(void)
{
    // $[Port A Configuration]
    /* Pin PA0 is configured to Push-pull */
    GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE0_MASK)
            | GPIO_P_MODEL_MODE0_PUSHPULL;

    /* Pin PA2 is configured to Push-pull */
    GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
            | GPIO_P_MODEL_MODE2_PUSHPULL;
    // [Port A Configuration]$

    // $[Route Configuration]
    /* Enable signals CC0, CC2 */
    TIMER0->ROUTE |= TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC2PEN;
    // [Route Configuration]$
}


/*
void TIMER1_IRQHandler(void)
{

   uint32_t compareValue;
//   printf("f=%d\n\r",CMU_ClockFreqGet(cmuClock_CORE));
//   printf("f=%d\n\r",10);
//   USART_Tx(USART1, 's');
//   EUSART_Write('T');
   // Clear flag for TIMER0 overflow interrupt
  TIMER_IntClear(TIMER1, TIMER_IF_OF);

  compareValue = TIMER_CaptureGet(TIMER1, 0);
// // increment duty-cycle or reset if reached TOP value
  if( compareValue == TIMER_TopGet(TIMER1))    TIMER_CompareBufSet(TIMER1, 0, 0);
  else TIMER_CompareBufSet(TIMER1, 0, ++compareValue);

//  tick_timer_count= compareValue;

}
*/
void TIMER0_IRQHandler(void)
{

   uint32_t compareValue;
//   printf("f=%d\n\r",CMU_ClockFreqGet(cmuClock_CORE));
//   printf("f=%d\n\r",10);
//   USART_Tx(USART1, 's');
//   EUSART_Write('T');
   // Clear flag for TIMER0 overflow interrupt
//  TIMER_IntClear(TIMER0, TIMER_IF_OF);

//  compareValue = TIMER_CaptureGet(TIMER0, 0);
// // increment duty-cycle or reset if reached TOP value
//  if( compareValue == TIMER_TopGet(TIMER0))
//    TIMER_CompareBufSet(TIMER0, 0, 0);
//  else
//    TIMER_CompareBufSet(TIMER0, 0, ++compareValue);

//  tick_timer_count= compareValue;

}

int Blink_Count;

void LED_Blink_ON() {
	if(tick_LED_Blink_ON_timer_count >= 200) {  // 1msec * 500 = 500msec
		SetColor(blink_color);
		tick_LED_Blink_ON_timer_count = 0;
		LED_Blink_OFF_timer_count = 0;
		LED_Blink_ON_timer_count = 1;
	}

	if(tick_LED_Blink_OFF_timer_count >= 200) {  // 1msec * 500 = 500msec
		SetColor(BLACK);
		tick_LED_Blink_OFF_timer_count = 0;

		LED_Blink_OFF_timer_count = 1;

		if (Blink_Count == 1) {	LED_Blink_ON_timer_count = 1; Blink_Count = 0;}
		else { Blink_Count--; LED_Blink_ON_timer_count = 0; }


	}
}

void ticktimer_callback()
{
	if(E_1st_Save)
	{
		if(E_SaveCount)E_SaveCount--;
		if(E_SaveCount==1) E_Save=1;
	}
	if(E_1st_SaveCount)E_1st_SaveCount--;
	if(E_1st_SaveCount==1)
	{
			E_1st_Save=1;
			E_Save=0;
			ClearSleepCounter(WORK_SHORT_TIME);
	}
	//systick_count;


	if(tick_1SEC_timer_count >= 1000) {  
		
		tick_1SEC_timer_count = 0;

		if (a2dpstat == 1 || a2dpstat == 2 ) {
			blink_color = RED;
			Blink_Count = 1;
			LED_Blink_ON_timer_count = 0;
		}
	}

 
	if(tick_LED_timer_count >= 2000) {  // 1msec * 500 = 500msec
		
		tick_LED_timer_count = 0;
		LED_timer_count = 1;
		SET_LED_RED(1);
		SET_LED_GREEN(1);
		SET_LED_BLUE(1);

	}

	if(tick_SET_VOL_timer_count >= 3000) {  // 1msec * 500 = 500msec
		
		tick_SET_VOL_timer_count = 0;
		SET_VOL_timer_count = 1;
		vs_set_volume(Master_Volume, Master_Volume);

	}

	if(tick_SOUND_LEVEL_timer_count >= 500) {  // 1msec * 500 = 500msec
		
		tick_SOUND_LEVEL_timer_count = 0;

		Sound_Led_Flag=1;

	}

	if(tick_NSPK_Voice_timer_count >= 500) {  // 1msec * 500 = 500msec
		
		tick_NSPK_Voice_timer_count = 0;
		NSPK_Voice_timer_count = 1;
		BT_generate_specific_tone (0x1e);   


	}

	if(Blink_Count > 0) {
		LED_Blink_ON();
	}

	if(tick_RED_LED_timer_count >= 2000) {  // 1msec * 1000 = 1000msec
		tick_RED_LED_timer_count = 0;
		blink_color = RED;
		Blink_Count = 1;
		LED_Blink_ON_timer_count = 0;
	}

	if(tick_DSP_INDEX_timer_count >= 5000) {  // 1msec * 2000 = 2000msec
		
		tick_DSP_INDEX_timer_count = 0;
		DSP_INDEX_timer_count = 1;
		BUTTON_event = BACK_to_event;

	}

	if(tick_BT_ON_timer_count > 100) {  // BTAPP_TaskState
		if(BTAPP_TaskState == BT_STATE_POWER_OFF) {  // BTAPP_TaskState
			//printf("BTAPP_TaskState = BT_STATE_POWER_OFF\r\n");
			tick_BT_ON_timer_count = 0;
			BT_ON_timer_count = 1;


			//printf("Auto Running BT_REQ_SYSTEM_ON\n\r");
			BM64_UART_TX_ready();		 	
	        BTAPP_TaskReq(BT_REQ_SYSTEM_ON);

//			LED_BLUE_Blink_Count(2);
#if 0 //defined( GN1_SLAVE)
			int i;
			for(i=0; i<1;i++) {
				Master_Volume = MIN_Volume - i * GAP_Volume;
				vs_set_volume(Master_Volume, Master_Volume);
				delayMs(200);
			}
#endif
			
		}

	}

	if(tick_BT_NSPK_ON_timer_count > 1000) {  // BTAPP_TaskState
//		if(BTAPP_TaskState == BT_STATE_BT_RUNNING) {  // BTAPP_TaskState
			tick_BT_NSPK_ON_timer_count = 0;
			BT_NSPK_ON_timer_count = 1;

#if defined( GN1_MASTER )
		#if 0
 				printf("Trigger NSPK Master\r\n");
                BT_MMI_ActionCommand ( 0xF5, BT_linkIndex); //MMI(Enter NSPK mode)  02 00 F5  
                BT_MMI_ActionCommand ( 0xE0, BT_linkIndex); //MMI(Trigger Master Role)  02 00 E0  
		#else
			BTAPP_BroadcastBtnLongPress();
		#endif
				//BTAPP_BroadcastBtnLongPress();
#else
//			printf("\n\rBTAPP_task_status !!! = %d\r\n",  BTAPP_task_status);
//			delayMs(1000);

		#if 0
 				printf("Trigger NSPK Slave\r\n");
                BT_MMI_ActionCommand ( 0xF5, BT_linkIndex); //MMI(Enter NSPK mode)  02 00 F5  
				delayMs(500);
                BT_MMI_ActionCommand ( 0xE1, BT_linkIndex); //MMI(Trigger Slave Role)  02 00 E1   
		#else
			BTAPP_BroadcastBtnLongPress();
		#endif
/*
	            //BT_MMI_ActionCommand ( 0xE1, BT_linkIndex); //Trigger NSPK Slave 

			//BUTTON_event = MODE_middle_event;
//                BT_MMI_ActionCommand ( 0xE0, BT_linkIndex); //Trigger NSPK Master 
*/			
#endif			
//			LED_GREEN_Blink_Count(2);

#if 0//defined( GN1_SLAVE)
			int i;
			for(i=0; i<1;i++) {
				Master_Volume = MIN_Volume - i * GAP_Volume;
				vs_set_volume(Master_Volume, Master_Volume);
				delayMs(200);
			}
#endif

	}

}

void TimerCount_init() {
#if 0	
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, ticktimer_callback);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);

	tc_write_period(&TCC0, 469 );  // 10ms
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_HI);
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1024_gc);	// prescale clk : 48000000 / 1024
#endif
}





