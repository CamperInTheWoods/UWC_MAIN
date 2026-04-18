/*
 * LED_PWM.c
 *
 *  Created on: 2018. 10. 19.
 *      Author: ki
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

#include "HA300_BSP.h"
#include "LED_PWM.h"


uint32_t timer1_top;

char blink_color;

static uint32_t RED_Value=PWM_100;
static uint32_t GREEN_Value=PWM_100;
static uint32_t BLUE_Value=PWM_100;
static uint32_t COLOR_Value=BLACK;


void  pwm_start(unsigned int ch, uint32_t duty_cycle)
{
    if (duty_cycle <= PWM_MAX)
        TIMER_CompareBufSet(TIMER1, ch, ((PWM_MAX-duty_cycle) * timer1_top) / PWM_MAX);

}
void  pwm_LED(uint32_t RedDuty, uint32_t GreenDuty, uint32_t BlueDuty)
{
	if( RedDuty >= PWM_MAX ) RedDuty=PWM_MAX;
    TIMER_CompareBufSet( TIMER1, PWM_RED, ( (PWM_MAX - RedDuty) * timer1_top ) / PWM_MAX );
	if( GreenDuty >= PWM_MAX ) GreenDuty=PWM_MAX;
    TIMER_CompareBufSet( TIMER1, PWM_GREEN, ( (PWM_MAX - GreenDuty) * timer1_top ) / PWM_MAX );
#if 0
	if( BlueDuty >= PWM_MAX ) BlueDuty=PWM_MAX;
    TIMER_CompareBufSet( TIMER1, PWM_BLUE, ( (PWM_MAX - BlueDuty) * timer1_top ) / PWM_MAX );
#endif	
}


void  PAM8019_Volume(uint32_t amp_volume)	// MAX value : 1000
{
	if(amp_volume==0) { TIMER1->ROUTE &= ~(TIMER_ROUTE_CC2PEN);}
	else { TIMER1->ROUTE |= TIMER_ROUTE_CC2PEN;}

	if( amp_volume >= PWM_MAX ) amp_volume=PWM_MAX;
    TIMER_CompareBufSet( TIMER1, PAM8019_VOL, ( (PWM_MAX - amp_volume) * timer1_top ) / PWM_MAX );
}



//uint8_t DB_LED_Color[10] = { RED, ORANGE, YELLOW, GREEN, BLUE, INDIGO, VIOLET, WHITE,  CYAN, MAGENTA };  

void SetColor(char col)
{
	if(col==BLACK) PWM_PortOFF();
	else PWM_PortON();
	switch(col)
	{
#if 0
	case BLACK:		pwm_LED( ZERO_Value,	ZERO_Value,			ZERO_Value		);	break;
	case RED:		pwm_LED( RED_Value,		ZERO_Value,			ZERO_Value		);	break;
	case ORANGE:	pwm_LED( RED_Value,		GREEN_Value/15,	ZERO_Value		);	break;
	case YELLOW:	pwm_LED( RED_Value,		GREEN_Value/7,		ZERO_Value		);	break;
	case GREEN:		pwm_LED( ZERO_Value,	GREEN_Value/3,		ZERO_Value		);	break;
	case CYAN:		pwm_LED( ZERO_Value,	GREEN_Value/2,		BLUE_Value		);	break;
	case BLUE:		pwm_LED( ZERO_Value,	GREEN_Value/2,		BLUE_Value		);	break;
	//case BLUE:		pwm_LED( ZERO_Value,	ZERO_Value,		BLUE_Value		);	break;
	case INDIGO:		pwm_LED( ZERO_Value,	ZERO_Value,		2*BLUE_Value/3	);	break;
	case MAGENTA:	pwm_LED( RED_Value/3,	ZERO_Value,			2*BLUE_Value/3	);	break;
	case VIOLET:	pwm_LED( 3*RED_Value/5,	ZERO_Value,			BLUE_Value/5	);	break;
//	case WHITE:		pwm_LED( RED_Value/2,	GREEN_Value/2,		BLUE_Value/2	);	break;
	case WHITE:		pwm_LED( RED_Value/2,	GREEN_Value/3,		BLUE_Value/3	);	break;
#else
//uint8_t DB_LED_Color[10] = { RED, ORANGE, YELLOW, GREEN, BLUE, INDIGO, VIOLET, WHITE,  CYAN, MAGENTA };  
	case BLACK:		pwm_LED( ZERO_Value,	ZERO_Value,			ZERO_Value		);	break;
	case RED:		pwm_LED( RED_Value,		ZERO_Value,			ZERO_Value		);	break;
	case ORANGE:	pwm_LED( RED_Value,		GREEN_Value/15,		ZERO_Value		);	break;
	case YELLOW:	pwm_LED( RED_Value,		GREEN_Value/7,		ZERO_Value		);	break;
	case GREEN:		pwm_LED( ZERO_Value,	GREEN_Value/3,		ZERO_Value		);	break;
	case CYAN:		pwm_LED( ZERO_Value,	GREEN_Value/2,		BLUE_Value		);	break;
	case BLUE:		pwm_LED( ZERO_Value,	GREEN_Value/2,		BLUE_Value		);	break;
	case INDIGO:		pwm_LED( ZERO_Value,	ZERO_Value,			2*BLUE_Value/3	);	break;
	case MAGENTA:	pwm_LED( RED_Value/3,	ZERO_Value,		2*BLUE_Value/3	);	break;
	case VIOLET:		pwm_LED( 3*RED_Value/5,	ZERO_Value,	BLUE_Value/5	);	break;
	//case WHITE:		pwm_LED( RED_Value/2,	GREEN_Value/4,	BLUE_Value*6/20	);	break;
	case WHITE:		pwm_LED( RED_Value/2,	GREEN_Value/2,	BLUE_Value/2	);	break;
#endif

	}
}

void PWM_init()
{

//    init_portio();
	SET_LED_GREEN(1);
	SET_LED_BLUE(1);
	SET_LED_RED(1);
	GPIO_PinModeSet(gpioPortB, 11,     gpioModePushPull, 1);
		
	init_timer1();


	// $[Route Configuration]
    /* Enable signals CC0, CC1, CC2 */
    TIMER1->ROUTE |= TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN| TIMER_ROUTE_CC2PEN;
    // [Route Configuration]$
    TIMER1->ROUTE |= TIMER_ROUTE_LOCATION_LOC3;



    /* Ensure core frequency has been updated */
//    SystemCoreClockUpdate();
    /* Start Systick Timer */
//    setup_systick_timer();
    /* Set Top Value */
    uint32_t freq = CMU_ClockFreqGet(cmuClock_HFPER);
    timer1_top = freq / PWM_FREQ;
    TIMER_TopSet(TIMER1, timer1_top);
    CMU_ClockEnable(cmuClock_TIMER1, true);
    SetColor(BLACK);
}
void PWM_PortON(void)
{
	// $[Route Configuration]
    /* Enable signals CC0, CC1, CC2 */
    TIMER1->ROUTE |= TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN| TIMER_ROUTE_CC2PEN;
    /* Enable signals CC0, CC1 */
    //TIMER1->ROUTE |= TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN;
    // [Route Configuration]$
//    TIMER1->ROUTE |= TIMER_ROUTE_LOCATION_LOC3;

}
void PWM_PortOFF(void)
{
	// $[Route Configuration]
    /* Enable signals CC0, CC1, CC2 */
    TIMER1->ROUTE &= ~(TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN| TIMER_ROUTE_CC2PEN);
    /* Enable signals CC0, CC1 */
    //TIMER1->ROUTE &= ~(TIMER_ROUTE_CC0PEN | TIMER_ROUTE_CC1PEN);
    // [Route Configuration]$
//    TIMER1->ROUTE &= ~TIMER_ROUTE_LOCATION_LOC3;

}

void init_timer1(void)
{
    // $[TIMER1 initialization]
    TIMER_Init_TypeDef init = TIMER_INIT_DEFAULT;

    init.enable = 1;
    init.debugRun = 1;
    init.dmaClrAct = 0;
    init.sync = 0;
    init.clkSel = timerClkSelHFPerClk;
    //init.prescale = timerPrescale64;
    init.prescale = timerPrescale1;
    init.fallAction = timerInputActionNone;
    init.riseAction = timerInputActionNone;
    init.mode = timerModeUp;
    init.quadModeX4 = 0;
    init.oneShot = 0;
    TIMER_Init(TIMER1, &init);    // [TIMER1 initialization]$

    // $[TIMER1 CC0 init]
    TIMER_InitCC_TypeDef initCC0 = TIMER_INITCC_DEFAULT;

    initCC0.prsInput = false;
    initCC0.prsSel = timerPRSSELCh0;
    initCC0.edge = timerEdgeRising;
    initCC0.mode = timerCCModePWM;
    initCC0.eventCtrl = timerEventEveryEdge;
    initCC0.filter = 0;
    initCC0.cofoa = timerOutputActionClear;
    initCC0.cufoa = timerOutputActionNone;
    initCC0.cmoa = timerOutputActionSet;
    initCC0.coist = 0;
    initCC0.outInvert = 0;
    TIMER_InitCC(TIMER1, 0, &initCC0);
    // [TIMER1 CC0 init]$

    // $[TIMER1 CC1 init]
    TIMER_InitCC_TypeDef initCC1 = TIMER_INITCC_DEFAULT;

    initCC1.prsInput = false;
    initCC1.prsSel = timerPRSSELCh0;
    initCC1.edge = timerEdgeBoth;
    initCC1.mode = timerCCModePWM;
    initCC1.eventCtrl = timerEventEveryEdge;
    initCC1.filter = 0;
    initCC1.cofoa = timerOutputActionClear;
    initCC1.cufoa = timerOutputActionNone;
    initCC1.cmoa = timerOutputActionSet;
    initCC1.coist = 0;
    initCC1.outInvert = 0;
    TIMER_InitCC(TIMER1, 1, &initCC1);
    // [TIMER1 CC1 init]$

    // $[TIMER1 CC2 init]
    TIMER_InitCC_TypeDef initCC2 = TIMER_INITCC_DEFAULT;

    initCC2.prsInput = false;
    initCC2.prsSel = timerPRSSELCh0;
    initCC2.edge = timerEdgeBoth;
    initCC2.mode = timerCCModePWM;
    initCC2.eventCtrl = timerEventEveryEdge;
    initCC2.filter = 0;
    initCC2.cofoa = timerOutputActionClear;
    initCC2.cufoa = timerOutputActionNone;
    initCC2.cmoa = timerOutputActionSet;
    initCC2.coist = 0;
    initCC2.outInvert = 0;
    TIMER_InitCC(TIMER1, 2, &initCC2);
    // [TIMER1 CC2 init]$

    /* Enable overflow interrupt */
//    TIMER_IntEnable(TIMER1, TIMER_IF_OF);

    /* Enable TIMER0 interrupt vector in NVIC */
//    NVIC_EnableIRQ(TIMER1_IRQn);

}
