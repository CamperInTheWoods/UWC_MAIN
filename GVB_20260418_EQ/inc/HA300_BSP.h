/*
 * HA300_BSP.h
 *
 *  Created on: 2018. 2. 9.
 *      Author: user
 */

#ifndef HA300_BSP_H_
#define HA300_BSP_H_

#include "HA_DefinedValues.h"

#define SET_XMOS_RESET(x)	GPIO_PinModeSet(gpioPortA, 0,     gpioModePushPull, x)
#define SET_DSP_RESET(x)	GPIO_PinModeSet(gpioPortD, 7,     gpioModePushPull, x)
#define SET_DSP_SCLK(x)		GPIO_PinModeSet(gpioPortE, 12,     gpioModePushPull, x)
#define SET_DSP_CS(x)		GPIO_PinModeSet(gpioPortE, 13,     gpioModePushPull, x)
#define SET_DSP_MISO(x)		GPIO_PinModeSet(gpioPortE, 11,     gpioModeInputPull, x)
#define SET_DSP_MOSI(x)		GPIO_PinModeSet(gpioPortE, 10,     gpioModePushPull, x)

#define SET_LED_RED(x)		GPIO_PinModeSet(gpioPortB, 7,     gpioModePushPull, x)
#define SET_LED_GREEN(x)	GPIO_PinModeSet(gpioPortB, 8,     gpioModePushPull, x)
#define SET_LED_BLUE(x)		GPIO_PinModeSet(gpioPortB, 13,     gpioModePushPull, x)

#define SET_PAM8019_MUTE(x) GPIO_PinModeSet(gpioPortB, 14,     gpioModePushPull, x)
#define SET_PAM8019_EN(x)	GPIO_PinModeSet(gpioPortD, 6,     gpioModePushPull, x)

#endif /* HA300_BSP_H_ */
