/**
  EUSART Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eusart.c

  @Summary
    This is the generated driver implementation file for the EUSART driver using MPLAB?Code Configurator

  @Description
    This header file provides implementations for driver APIs for EUSART.
    Generation Information :
        Product Revision  :  MPLAB?Code Configurator - v2.25.2
        Device            :  PIC18F26K20
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*/

/**
  Section: Included Files
*/
#include "eusart.h"
#include "em_usart.h"
#include "efm32hg310f64.h" //EFM32HG310F64_H
/*
//
//  Section: Macro Declarations
//
//#define EUSART_TX_BUFFER_SIZE 16
#define EUSART_TX_BUFFER_SIZE 100
#define EUSART_RX_BUFFER_SIZE 255

//
//  Section: Global Variables
//
*/
static uint8_t eusartTxHead = 0;
static uint8_t eusartTxTail = 0;
static uint8_t eusartTxBuffer[EUSART_TX_BUFFER_SIZE];
volatile uint8_t eusartTxBufferRemaining;

static uint8_t eusartRxHead = 0;
static uint8_t eusartRxTail = 0;
static uint8_t eusartRxBuffer[EUSART_RX_BUFFER_SIZE];
volatile uint8_t eusartRxCount;
/**/

extern void USART1_Tx(USART_TypeDef *usart, uint8_t data);
/**
  Section: EUSART APIs
*/

void EUSART_Initialize(void)
{
	  memset(eusartRxBuffer,0,sizeof(eusartRxBuffer));
}

uint8_t EUSART_Read(void)
{
    uint8_t readValue  = 0;

    while(0 == eusartRxCount)
    {
    }

    readValue = eusartRxBuffer[eusartRxTail];
    eusartRxBuffer[eusartRxTail] = 0;
	 eusartRxTail++;
    if(EUSART_RX_BUFFER_SIZE <= eusartRxTail)
    {
        eusartRxTail = 0;
    }
    eusartRxCount--;
    return readValue;
}


void EUSART_Write(uint8_t txData)
{
    while(0 == eusartTxBufferRemaining)
    {
    }

    if (0==(USART1->IF & USART_IF_TXBL) )
    {
        USART_Tx(USART1, txData);
    }
    else
    {
    	USART_IntDisable(USART1, USART_IEN_TXBL);
        eusartTxBuffer[eusartTxHead++] = txData;
        if(sizeof(eusartTxBuffer) <= eusartTxHead)
        {
            eusartTxHead = 0;
        }
        eusartTxBufferRemaining--;
    }
    USART_IntEnable(USART1, USART_IEN_TXBL);
}

extern void UART_TransferNextByte( void );
void EUSART_Transmit_ISR(void)
{
    // add your EUSART interrupt custom code
  if (USART1->IF & USART_IF_TXBL)
  {
    if(sizeof(eusartTxBuffer) > eusartTxBufferRemaining)
    {
        USART_Tx(USART1, eusartTxBuffer[eusartTxTail++]);
        if(EUSART_TX_BUFFER_SIZE <= eusartTxTail)
        {
            eusartTxTail = 0;
        }
        eusartTxBufferRemaining++;
        UART_TransferNextByte();//UART_TransferNextByte2();
    }
    else
    {
    	USART_IntDisable(USART1, USART_IEN_TXBL);
        UART_TransferNextByte();
    }
  }
}

/*
void EUSART_Receive_ISR(void)
{
    if(1 == RCSTA2bits.OERR)
    {
        // EUSART error - restart

        RCSTA2bits.CREN = 0;
        RCSTA2bits.CREN = 1;
    }

    // buffer overruns are ignored
    eusartRxBuffer[eusartRxHead++] = RCREG2;
    if(sizeof(eusartRxBuffer) <= eusartRxHead)
    {
        eusartRxHead = 0;
    }
    eusartRxCount++;
}
*/
/**************************************************************************//**
 * @brief UART1 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
	char data;

	/* Check for RX data valid interrupt */
  if (USART1->IF & USART_IF_RXDATAV)
  {

	    // buffer overruns are ignored
		data = USART_Rx(USART1);
		eusartRxBuffer[eusartRxHead++] = data;
	    if(EUSART_RX_BUFFER_SIZE <= eusartRxHead)
	    {
	        eusartRxHead = 0;
	    }
	    eusartRxCount++;
  }

}


/**
  End of File
*/
