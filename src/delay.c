
#include "delay.h"

/**********************************************************
 * Delay a number of milliseconds
 **********************************************************/
//extern int mseconds_tick;
#define One_ms_in_us (1000)
void delayMs(unsigned int ms)
{
    while(ms--) delayUs(One_ms_in_us);
}

/**********************************************************
 * Delay a number of microseconds
 * 10 us is minimum 65535 us is maximum
 * dead time is 8.7us at 14Mhz clock
 **********************************************************/
#define Minimum_us 10

void delayUs(unsigned int us)
{
		if(us<Minimum_us) us=Minimum_us;
		us = us - Minimum_us;
		FiveNop();
		FiveNop();
		FiveNop();
		while(us--){ FiveNop(); FiveNop(); Nop(); }
}

