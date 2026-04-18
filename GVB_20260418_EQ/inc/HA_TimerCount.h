/*
 * HA_TimerCount.h
 *
 *  Author: Rapiarno
 */ 

#ifndef HA_TIMERCOUNT_H
#define HA_TIMERCOUNT_H

extern long P3I5_timer_count;
extern unsigned int currentState_3PI5;
extern unsigned int lastState_3PI5;
extern int Blink_Count;
	
extern long tick_DSP_INDEX_timer_count;

// static long TC_scale=0;

//void adc_handler(ADC_t *adc, uint8_t ch_mask, adc_result_t result);
void __delay_ms(uint32_t milliseconds);
void TimerCount_init();
void pulse_out(unsigned int ch, uint32_t duty_cycle);
void ticktimer_callback(void);

#endif
