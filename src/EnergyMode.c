//
//  EnergyMode.c
//
//   Created on: 2018. 6. 25.
//       Author: ki
//
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "EnergyMode.h"
#include "key.h"
#include "bt_app.h"
#include "HA300_BSP.h"
#include "LED_PWM.h"


// Structs for modules used
CMU_TypeDef     *cmu    = CMU;
RTC_TypeDef     *rtc    = RTC;
TIMER_TypeDef   *timer  = TIMER0;


volatile uint8_t rtcInterrupt = 0;	// Global flag to indicate rtc interrupt
bool E_Save=0;
long E_SaveCount=0;
bool E_1st_Save=0;
long E_1st_SaveCount=0;
void InitEnergyMode()
{
	  E_SaveCount=0;
	  E_Save=0;
	  E_1st_SaveCount=SLEEP_INHBIT_TIME;
	  E_1st_Save=0;
}
void RunEnergyMode()
{
	if( !E_1st_Save ) return;
	if( BUTTON_event != NONE_push_event ){	ClearSleepCounter(WORK_LONG_TIME);	}
	if( !E_Save ) return;
	E_Save=0;

	setupRTCTimeout(RTC_COUNT_TO_WAKEUP);
	SetColor(BLACK);	//SET_LED_RED(1); SET_LED_GREEN(1); SET_LED_BLUE(1);
	SaveEnergyForPorts();
	EMU_EnterEM3(1);
	RestoreEnergyForPorts();
	ClearSleepCounter(WORK_SHORT_TIME);

}
void ClearSleepCounter(long time)
{
	  E_SaveCount=time;
}
/***************************************************************************//**
 * @brief
 *   Set the mode for a GPIO pin.
 *
 * @param[in] port
 *   The GPIO port to access.
 *
 * @param[in] pin
 *   The pin number in the port.
 *
 * @param[in] mode
 *   The desired pin mode.
 *
 * @param[in] out
 *   Value to set for pin in DOUT register. The DOUT setting is important for
 *   even some input mode configurations, determining pull-up/down direction.
 ******************************************************************************/
void GPIO_PinModeSetOnly(GPIO_Port_TypeDef port,
                     unsigned int pin,
                     GPIO_Mode_TypeDef mode  )
{
  EFM_ASSERT(GPIO_PORT_PIN_VALID(port, pin));

  /* There are two registers controlling the pins for each port. The MODEL
   * register controls pins 0-7 and MODEH controls pins 8-15. */
  if (pin < 8)
  {
    GPIO->P[port].MODEL = (GPIO->P[port].MODEL & ~(0xF << (pin * 4)))
                          | (mode << (pin * 4));
  }
  else
  {
    GPIO->P[port].MODEH = (GPIO->P[port].MODEH & ~(0xF << ((pin - 8) * 4)))
                          | (mode << ((pin - 8) * 4));
  }
}
void RestoreEnergyForPorts()
{
}
void SaveEnergyForPorts()
{
}
// ===========================================================================
// @brief disables all clocks and waits
// ===========================================================================
void disableAllClocks(void)
{
  CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFRCO);  // Switching the CPU clock source to HFRCO

 //  Disabling every oscillator except hfrco and lfxo
 //  NOTE: MAKE SURE NOT TO DISABLE THE CURRENT CPU CLOCK!!
  CMU_OscillatorEnable(cmuOsc_AUXHFRCO, false, true);
  CMU_OscillatorEnable(cmuOsc_HFXO, false, true);
  CMU_OscillatorEnable(cmuOsc_LFRCO, false, true);

 //  Disabling all unused clocks. The LE clock must be on in order to use the RTC
  cmu->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE;
  cmu->HFPERCLKEN0  = 0;

//  waitForRTC();	// Wait
}



// ===========================================================================
//  @brief  Start LFXO for RTC
//  Starts the LFXO and routes it to the RTC.
// ===========================================================================
void startLFXOForRTC(void)
{
//  CMU_OscillatorEnable(cmuOsc_LFXO, true, true); // Starting LFXO and waiting until it is stable
  CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true); // Starting ULFRCO and waiting until it is stable

//  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);  // Routing the LFXO clock to the RTC
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);  // Routing the ULFRCO clock to the RTC
  CMU_ClockEnable(cmuClock_RTC, true);

  CMU_ClockEnable(cmuClock_CORELE, true);  // Enabling clock to the interface of the low energy modules
}



// ===========================================================================
// * @brief  Sets up the RTC
// *
// ===========================================================================
void setupRTCTimeout(uint32_t value)
{
	rtcInterrupt = 0;
  RTC_CompareSet(0, value);  // Setting up RTC
  RTC_IntEnable(RTC_IFC_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_Enable(true);
}



// ===========================================================================
// * @brief waits for rtc trigger
// ===========================================================================
void waitForRTC(void)
{
  rtcInterrupt = 0;
  setupRTCTimeout(RTC_COUNT_TO_WAKEUP);

  while (rtcInterrupt == 0) ;	// Waiting for rtc interrupt
}


void RTC_IRQHandler(void)
{
  RTC_IntClear(RTC_IFC_COMP0);  // Clear interrupt source
  __DSB();  // Flushing instructions to make sure that the interrupt is not re-triggered
  	  	  	// This may be required when the peripheral clock is slower than the core
  NVIC_DisableIRQ(RTC_IRQn);	// Disabling interrupts from RTC
  RTC_Enable(false);  			// Disabling RTC
  rtcInterrupt = 1;		// Asserting that an RTC interrupt has occured.
}

