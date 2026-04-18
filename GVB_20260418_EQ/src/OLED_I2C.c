#include "em_gpio.h"
#include "delay.h"

#include "OLED_I2C.h"

/*
void Step(void) interrupt 0 using 0
{
	Delay1(300);
	if(int0==0)
 // {	while(IE0);
	log=!log;
 // }
   while(!int0);
  return;

}
*/


void I2C_Delay(void)
{
    for (volatile int i = 0; i < 50; i++);
}

void SDA_High(void)
{
    GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);
}
void SDA_Low(void)
{
    GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 0);
}
void SDA_Input(void)
{
    GPIO_PinModeSet(gpioPortC, 0, gpioModeInputPullFilter, 1);
}

void SDA_Output(void)
{
    // Open-drain output + pull-up
    GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);
}

void SCL_High(void)
{
    GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);
}
void SCL_Low(void)
{
    GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);
}

void Delay1(ulong n)
{
	while(n--)
	{
		FiveNop();
	}
	return;
}


void Write_number(uchar *n,uchar k,uchar station_dot)
{
	uchar i;
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x40);
			for(i=0;i<8;i++)
				{
				SentByte(*(n+16*k+i));
				}
		Stop();
				

	Set_Page_Address(Start_page+1)	;
    Set_Column_Address(Start_column+station_dot*8);	
			Start();
			SentByte(OLED_Write_Address);
			SentByte(0x40);
			for(i=8;i<16;i++)
			{
				SentByte(*(n+16*k+i));
			}
			Stop();

}
/*
void Delay(uint n)
{
 while(n)
 {    if((key_add==0)||(key_dec==0)) adj_Contrast();
	  n--;
 }
 	for(;log==1;)
	{
 	 if((key_add==0)||(key_dec==0)) adj_Contrast();
	}

	return;
}
*/

#if 0
void Start(void)
{
	GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);	// SDA=1;
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL=1;
//	SDA=1;
//	SCL=1;
	Nop();
	GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 0);	// SDA=0;
//	SDA=0;
	Nop();
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
//	SCL=0;
}
#else
void Start(void)
{
    SDA_High();
    SCL_High();
    I2C_Delay();
    SDA_Low();
    I2C_Delay();
    SCL_Low();
}	
#endif

#if 0
void Stop(void)
{
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
	GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 0);	// SDA=0;
//	SCL=0;
//	SDA=0;
	Nop();
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL=1;
//	SCL=1;
	Nop();
	GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);	// SDA=1;
//	SDA=1;
}	
#else
void Stop(void)
{
    SDA_Low();
    I2C_Delay();
    SCL_High();
    I2C_Delay();
    SDA_High();
    I2C_Delay();
}
#endif

#if 0
uint8_t  Check_Ack(void)//Acknowledge
{
	uint8_t ack=1;
	GPIO_PinModeSet(gpioPortC, 0, gpioModeInputPullFilter, 1);	// SDA=1; with Input mode
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL=1;
//	SDA=1;
//  SCL=1;
	Nop();
    while(ack==1)
    {
    	ack = GPIO_PinInGet(gpioPortC, 0);	// SDA(=PC0)
    	//ack=SDA;
    }
	Nop();
	GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);	// SDA=1;
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
//    SCL=0;
    return ack;
}
#else

uint8_t Check_Ack(void)   // return 0 = ACK, 1 = NACK(또는 타임아웃)
{
    uint32_t timeout = 1000;

    GPIO_PinModeSet(gpioPortC, 0, gpioModeInputPullFilter, 1);
    GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);
    I2C_Delay(); // 최소 4~5us

    // SDA read (0=ACK, 1=NACK)
    while (timeout--)
    {
        uint8_t sda = GPIO_PinInGet(gpioPortC, 0);
        if (sda == 0)
            break;      // OK ACK(0)
    }

    GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);
    I2C_Delay();
    GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);

    return (timeout == 0) ? 1 : 0;
}
#endif

#if 0
void SentByte(unsigned char Byte)
{
	uchar i;
	for(i=0;i<8;i++)
     {
		GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
        if((Byte&0x80)==0x80) GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);	// SDA=1;
       	else				  GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 0);	// SDA=0;
    	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL=1;
    	Nop();
        Byte=Byte<<1;
     }
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
	Check_Ack();
	
}
#else
void SentByte(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        if (data & 0x80) SDA_High();
        else             SDA_Low();

        SCL_High();
        I2C_Delay();
        SCL_Low();
        I2C_Delay();

        data <<= 1;
    }

    // read ACK
    SDA_Input();
    SCL_High();
    I2C_Delay();
    uint8_t ack = GPIO_PinInGet(gpioPortC, 0);
    SCL_Low();
    SDA_High();

    //return ack;   // 0=ACK, 1=NACK
}
#endif
unsigned char ReceiveByte(void)
{
	uchar i,rudata=0;
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
	GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);	// SDA=1;
	for(i=0;i<8;i++)
    {
    	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL=1;
		Nop();
		if ( GPIO_PinInGet(gpioPortC, 0)==1 ) rudata|=0x01;
		else rudata|=0x00;
        rudata=rudata<<1;
    	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
		Nop();
    }
	Send_ACK();
	return rudata;
}

void Send_ACK(void)
{
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
	GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 0);	// SDA=0;
	Nop();
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL=1;
	GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 0);	// SCL=0;
}

uint8_t ReceiveByte_NACK(void)
{
    uint8_t data = 0;

    // 1) 8비트 수신
    SDA_Input();
    for (int i = 0; i < 8; i++)
    {
        SCL_Low();  I2C_Delay();
        SCL_High(); I2C_Delay();

        data <<= 1;
        if (GPIO_PinInGet(gpioPortC, 0))
            data |= 1;
    }

    // 2) NACK 전송 (SDA = 1 while SCL HIGH)
    SCL_Low();
    SDA_Output();
    SDA_High();         // NACK
    I2C_Delay();

    SCL_High();         // NACK 샘플링 구간
    I2C_Delay();

    SCL_Low();

    // 3) SDA release
    SDA_Input();

    return data;
}

uint8_t ReceiveByte_ACK(void)
{
    uint8_t data = 0;

    // 1) 8비트 수신 (Slave가 SDA 구동)
    SDA_Input();
    for (int i = 0; i < 8; i++)
    {
        SCL_Low();  I2C_Delay();
        SCL_High(); I2C_Delay();

        data <<= 1;
        if (GPIO_PinInGet(gpioPortC, 0))
            data |= 1;
    }

    // 2) ACK 전송 (SDA = 0 while SCL HIGH)
    SCL_Low();
    SDA_Output();
    SDA_Low();          // ACK
    I2C_Delay();

    SCL_High();         // ACK 샘플링 구간
    I2C_Delay();

    SCL_Low();

    // 3) SDA release
    SDA_Input();

    return data;
}

uint8_t XMOS_ReadStatus(void)
{
    Start();
    SentByte(0x59);  // 0x2C<<1 | 1
    uint8_t status_xx = ReceiveByte_NACK();
    Stop();
    return status_xx;   // 0x00 = OK
}

// Set page address 0~15
void Set_Page_Address(unsigned char add)
{	
	Start();
	SentByte(OLED_Write_Address);
	SentByte(0x80);
	add=0xb0|add;
	SentByte(add);
	Nop();
	return;
}

void Set_Column_Address(unsigned char add)
{	add+=32;
	Start();
	SentByte(OLED_Write_Address);
	SentByte(0x80);
	SentByte((0x10|(add>>4)));
	SentByte(0x80);
	SentByte((0x0f&add));
 	Stop();
	return;
}



void Set_Contrast_Control_Register(unsigned char mod)
{
	Start();
	SentByte(OLED_Write_Address);
	SentByte(0x80);
	SentByte(0x81);
	SentByte(0x80);
	SentByte(mod);
 	Stop();
	return;
}


//#pragma disable
void Initial(void)
{
/*
	RST=1;
	Delay(2000);
	RST=0;
	Delay(2000);
	RST=1;
*/
	Delay1(2000);

	Start();
	SentByte(OLED_Write_Address);
	SentByte(0x80);
	SentByte(0xae);//--turn off oled panel
	SentByte(0x80);

	SentByte(0xd5);//--set display clock divide ratio/oscillator frequency
	SentByte(0x80);
	SentByte(0x80);//--set divide ratio
	SentByte(0x80);

	SentByte(0xa8);//--set multiplex ratio(1 to 64)
	SentByte(0x80);
	SentByte(0x1f);//--1/32 duty
	SentByte(0x80);

	SentByte(0xd3);//-set display offset
	SentByte(0x80);
	SentByte(0x00);//-not offset
	SentByte(0x80);

	SentByte(0x8d);//--set Charge Pump enable/disable
	SentByte(0x80);
	SentByte(0x14);//--set(0x10) disable
	SentByte(0x80);

	SentByte(0x40);//--set start line address
	SentByte(0x80);

	SentByte(0xa6);//--set normal display
	SentByte(0x80);

	SentByte(0xa4);//Disable Entire Display On
	SentByte(0x80);

	SentByte(0xa1);//--set segment re-map 128 to 0
	SentByte(0x80);

	SentByte(0xC8);//--Set COM Output Scan Direction 64 to 0
	SentByte(0x80);
	
	SentByte(0xda);//--set com pins hardware configuration
	SentByte(0x80);
	SentByte(0x12);
	SentByte(0x80);

	SentByte(0x81);//--set contrast control register
	SentByte(0x80);
	SentByte(Contrast_level);
	SentByte(0x80);

	SentByte(0xd9);//--set pre-charge period
	SentByte(0x80);
	SentByte(0xf1);
	SentByte(0x80);

	SentByte(0xdb);//--set vcomh
	SentByte(0x80);
	SentByte(0x40);

	SentByte(0x80);

	SentByte(0xaf);//--turn on oled panel

 	Stop();



}

void clear(void)
{
	unsigned char i,j;
	for(i=0;i<0x08;i++)
	{
		Set_Page_Address(i);

		Set_Column_Address(0x00);

		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x40);
		for(j=0;j<128;j++)
		{
			SentByte(0);
		}
		 	Stop();
	}
	return;
}

void Display_Chess(unsigned char value)
{
    unsigned char i,j,k;
    for(i=0;i<0x04;i++)
	{
		Set_Page_Address(i);

        Set_Column_Address(0x00);
		
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x40);
		for(j=0;j<0x08;j++)
		{
		    for(k=0;k<0x04;k++)
				SentByte(value);
		    for(k=0;k<0x04;k++)
				SentByte(~value);
		}
		 	Stop();
	}
    return;
}


void Display_Chinese(unsigned char ft[])
{
    unsigned char i,j,k,b,c=0;
	unsigned int	num=0;

for(b=0;b<2;b++)
  {
    for(i=0;i<0x02;i++)
	{	Set_Page_Address(c);
    	Set_Column_Address(0x00);
	    num=i*0x10+b*256;
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x40);
		for(j=0;j<0x04;j++)
		{
            for(k=0;k<0x10;k++)
			{
				SentByte(ft[num+k]);
			}
			num+=0x20;
		}c++;
		Stop();
	}
  }
    return;
}


//Display_Chinese1
void Display_Chinese_Column(unsigned char ft[])
{
    unsigned char i,j,k,num=0x40;
    for(i=0;i<0x04;i++)
	{
		Set_Page_Address(i);
        Set_Column_Address(0x00);
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x40);
		for(j=0;j<0x04;j++)
		{
            for(k=0;k<0x10;k++)
			{
				SentByte(ft[num+k]);
			}
		}
		Stop();
	num+=0x10;
	}
    return;
}


void Display_Picture(unsigned char pic[])
{
    unsigned char i,j,num=0;
	for(i=0;i<0x04;i++)
	{
	Set_Page_Address(i);
    Set_Column_Address(0x00);
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x40);
        for(j=0;j<0x40;j++)
		{
			SentByte(pic[i*0x40+j]);
		}
		Stop();
	}
    return;
}

void move_test(void)
{    unsigned char i;

	for(i=0x0;i<=16;i++)
	{
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(i+0x40);//--set Inverse Display
		SentByte(0x00);
		Stop();
		delayMs(20);// Delay1(2000);
	}
	delayMs(DELAY_WAIT/2);// 	Delay1(35000);

	for(i=17;i<=32;i++)
	{
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(i+0x40);//--set Inverse Display
		SentByte(0x00);
		Stop();
		delayMs(20);// Delay1(2000);
	}

	delayMs(DELAY_WAIT);// Delay1(65000);

		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0x40);//--set Inverse Display
		SentByte(0x00);
		Stop();
}

void show(void)
{
//	IE=0x81;
//	IP=0x01;
//	TCON=0x01;
//	int0=1;
//	Delay1(100);

    Initial();

	clear();
	/*
	Display_Picture(pic);
	Delay1(650000L);
	Display_Picture(pic2);
	Delay1(650000L);
	Display_Picture(pic3);
	Delay1(650000L);
	Display_Picture(pic1);
	Delay1(650000L);
	Display_Chinese(font);
	Delay1(650000L);
	Display_Chess(0x0f);
	Delay1(650000L);
	while(1);
	*/

	while(1)
	{
//		clear();
	
		Display_Picture(pic);
		delayMs(DELAY_WAIT);// Delay1(65000);

		move_test();		
		move_test();
		move_test();

        Display_Picture(pic);
        delayMs(DELAY_WAIT);// Delay1(65000);
	
		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa7);//--set Inverse Display	
		SentByte(0x00);
		Stop();
		delayMs(DELAY_WAIT);// Delay1(65000);

		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa6);//--set normal display
		SentByte(0x00);
		Stop();
		Display_Picture(pic1);
		delayMs(DELAY_WAIT);// Delay1(65000);

		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa7);//--set Inverse Display	
		SentByte(0x00);
		Stop();
		delayMs(DELAY_WAIT);// Delay1(65000);

		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa6);//--set normal display
		SentByte(0x00);
		Stop();
		Display_Picture(pic2);
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa7);//--set Inverse Display	
		SentByte(0x00);
		Stop();
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa6);//--set normal display
		SentByte(0x00);
		Stop();
		Display_Picture(pic3);
		delayMs(DELAY_WAIT);// Delay1(65000);

		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa7);//--set Inverse Display	
		SentByte(0x00);
		Stop();
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa6);//--set normal display
		SentByte(0x00);
		Stop();
		Display_Chess(0x0f);
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa7);//--set Inverse Display	
		SentByte(0x00);
		Stop();
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa6);//--set normal display
		SentByte(0x00);
		Stop();
		Display_Chinese(font);
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa7);//--set Inverse Display	
		SentByte(0x00);
		Stop();	
		Display_Chinese(font);
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa6);//--set normal display
		SentByte(0x00);
		Stop();
		Display_Chinese_Column(font);
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa7);//--set Inverse Display	
		SentByte(0x00);
		Stop();	
		Display_Chinese_Column(font);
		delayMs(DELAY_WAIT);// Delay1(65000);


		Start();
		SentByte(OLED_Write_Address);
		SentByte(0x80);
		SentByte(0xa6);//--set normal display
		SentByte(0x00);
		Stop();
	}
}
