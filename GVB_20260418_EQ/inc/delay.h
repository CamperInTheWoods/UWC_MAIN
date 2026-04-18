
#define Nop() __asm__ __volatile__ ("nop")
#define FiveNop() Nop(); Nop(); Nop(); Nop(); Nop()

void delayMs(unsigned int ms);
void delayUs(unsigned int us);
