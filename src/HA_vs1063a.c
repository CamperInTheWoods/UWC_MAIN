/*
 * HA_vs1063a.c
 *
 *  Author: Rapiarno
 */ 
#include "em_usart.h"
#include "HA_DefinedValues.h"
#include "HA_vs1063a.h"
#include "em_gpio.h"
#include "HA_bt.h"



#define SPI_CS_LOW() 	GPIO_PinOutClear((GPIO_Port_TypeDef) VS_CS_PORT, VS_CS_PIN);  //CS _LOW
#define SPI_CS_HIGH()   GPIO_PinOutSet((GPIO_Port_TypeDef) VS_CS_PORT, VS_CS_PIN); //CS_HIGH


// pull the VS10xx Data Chip Select line LOW
void VS_XDCS_select_data()
{
	GPIO_PinOutClear((GPIO_Port_TypeDef) VS_BSYNC_PORT, VS_BSYNC_PIN);
}

// pull the VS10xx Data Chip Select line HIGH
void VS_XDCS_deselect_data()
{
	GPIO_PinOutSet((GPIO_Port_TypeDef) VS_BSYNC_PORT, VS_BSYNC_PIN);
}

void USART0_Tx(USART_TypeDef *usart, uint8_t data)
{
  /* Check that transmit buffer is empty */
  while (!(usart->STATUS & USART_STATUS_TXBL))
    ;
  usart->TXDATA = (uint32_t)data;
  while (!(usart->STATUS & USART_STATUS_TXC))
    ;
  return;
}

uint8_t USART0_Reader(USART_TypeDef *usart)
{

  USART0_Tx(USART0, 0xFF);
  while (!(usart->STATUS & USART_STATUS_RXDATAV))
    ;
  return (uint16_t)usart->RXDATA;

}

uint16_t USART0_read_TwoByte(USART_TypeDef *usart){

	uint16_t data;
	uint8_t highbyte;
	uint8_t lowbyte;

	highbyte = USART0_Reader(usart);
	lowbyte = USART0_Reader(usart);

	data = highbyte << 8;
	data |= (lowbyte & 0xff);

	return data;
}


void spi_write_TwoByte(const uint16_t value)
{
	uint8_t byte[2];
	byte[0] = (value & 0xff00) >> 8;
	byte[1] = value & 0x00ff;
	USART0_Tx(USART0, byte[0] );
	USART0_Tx(USART0, byte[1] );
}

void vs_Wait()
{
	while(!GPIO_PinInGet(VS_DREQ_PORT, VS_DREQ_PIN)){
		delayMs(5);
		//UDELAY_Delay(5000);	   //Read VS_DREQ
	}
}

void DSP_Reset()
{
	GPIO_PinModeSet(VS_XRESET_PORT, VS_XRESET_PIN,     gpioModePushPull, 1);   //  RESET  //PC8
	delayMs(10);
	//UDELAY_Delay(10000);
	GPIO_PinModeSet(VS_XRESET_PORT, VS_XRESET_PIN,     gpioModePushPull, 0);   //  RESET  //PC8
	delayMs(5);
	//UDELAY_Delay(5000);	// 1000 --> 100	20150902
	GPIO_PinModeSet(VS_XRESET_PORT, VS_XRESET_PIN,     gpioModePushPull, 1);   //  RESET //PC8
	delayMs(10);
	//UDELAY_Delay(10000);		// DSP initial time ÇÊ¿ä(?)
	vs_Wait();	// Wait DREQ HIGH
}




void WriteSci(uint8_t SCI_address, uint16_t data)
{
	SPI_CS_LOW();
	vs_Wait();	// Wait DREQ HIGH  // Read VS_DREQ
	USART0_Tx(USART0, WRITE );
	USART0_Tx(USART0, SCI_address);
	spi_write_TwoByte(data);
	vs_Wait();	// Wait DREQ HIGH
	SPI_CS_HIGH();
}

uint16_t ReadSci(uint8_t SCI_address)
{
	uint16_t resultvalue = 0;
	SPI_CS_LOW();
	vs_Wait();	// Wait DREQ HIGH Read VS_DREQ
	USART0_Tx(USART0, READ);
	USART0_Tx(USART0, SCI_address);
	resultvalue = USART0_read_TwoByte(USART0);
	vs_Wait();	// Wait DREQ HIGH
	SPI_CS_HIGH();
	return resultvalue;
}


int WriteSdi(uint8_t *data, uint8_t bytes) {
	unsigned char i;

	if (bytes > 32)
		return -1; // Error: Too many bytes to transfer!

	vs_Wait();
	VS_XDCS_select_data();
	for (i=0; i<bytes; i++) {
		USART0_Tx(USART0, *data++);
	}
	VS_XDCS_deselect_data();
	vs_Wait();

	return 0; // Ok
}

// read data rams
uint16_t vs_read_wramaddr(uint16_t address)
{
	uint16_t rv = 0;
	WriteSci(SCI_WRAMADDR, address);
	rv = ReadSci(SCI_WRAM);
	return rv;
}

// write to data rams
void vs_write_wramaddr(uint16_t address, uint16_t value) // *address = value;
{
	WriteSci(SCI_WRAMADDR, address);  	// Set RAM Address		//
	WriteSci(SCI_WRAM, value);  			// Write RAM Data		// *address = value;
}


