#include "udelay.h"
#include "em_timer.h"
#include "em_device.h"
#include "em_chip.h"
#include "HA_bt.h"
#include "delay.h"
//#include "ustimer.h"
#include "em_cmu.h"
#include "HA300_BSP.h"
#include "HA_DefinedValues.h"

#define RTC_FREQ 32768
#define TIMER_FREQ 14000000
//#define TIMER_FREQ 48000000
//#define TIMER_FREQ 24000000


/**********************************************************
 * Enables clocks used for delay functions.
 **********************************************************/
void initDelay(void)
{
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
}

extern volatile uint32_t systick_count;


extern void data_printf(uint8_t data);

void USART1_Tx(USART_TypeDef *usart, uint8_t data)
{

	data_printf(data);

  /* Check that transmit buffer is empty */
  while (!(usart->STATUS & USART_STATUS_TXBL))
    ;
  usart->TXDATA = (uint32_t)data;
  while (!(usart->STATUS & USART_STATUS_TXC))
    ;
  return;
}


void XMOS_Reset(){
  SET_XMOS_RESET(0);	//	GPIO_PinModeSet(gpioPortA, 0,     gpioModePushPull, 1);  
  delayMs(50);
  SET_XMOS_RESET(1);	//	GPIO_PinModeSet(gpioPortA, 0,     gpioModePushPull, 1);  
}

void BT_MakeCall(){
}
