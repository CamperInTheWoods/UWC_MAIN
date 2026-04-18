/*
 * HA_SystemClock.c
 *
 *  Author: Rapiarno
 */ 

/*! \brief This function changes the prescaler configuration.
 *
 *  Change the configuration of the three system clock
 *  prescaler is one single operation. The user must make sure that
 *  the main CPU clock does not exceed recommended limits.
 *
 *  \param  PSAfactor   Prescaler A division factor, OFF or 2 to 512 in
 *                      powers of two.
 *  \param  PSBCfactor  Prescaler B and C division factor, in the combination
 *                      of (1,1), (1,2), (4,1) or (2,2).
 */


void clk_2m(void)
{
    /*  Select 2 MHz RC oscillator as main clock source and diable
     *  unused clock.
     */

}

void clk_32m(void)
{
    /*  Enable internal 32 MHz ring oscillator and wait until it's
     *  stable. Divide clock by two with the prescaler C and set the
     *  32 MHz ring oscillator as the main clock source. Wait for
     *  user input while the LEDs toggle.
     */


}
 
 
void clk_32k(void)
{
    /*  Enable internal 32 kHz calibrated oscillator and check for
     *  it to be stable and set prescaler A, B and C to none. Set
     *  the 32 kHz oscillator as the main clock source. Wait for
     *  user input while the LEDs toggle.
     */


} 



