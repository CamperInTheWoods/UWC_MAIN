/**************************************************************************//**
 * @file main.c
 * @brief USB CDC Serial Port adapter example project.
 * @version 4.3.0
 ******************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_i2c.h"

#include <em_int.h>
#include "em_usb.h"
#include "usbtocdc.h"
#include "descriptors.h"
#include "udelay.h"
#include "em_usart.h"
#include "em_gpio.h"

#include "key.h"
#include "HA_DefinedValues.h"
#include "HA_TimerCount.h"
#include "HA_MCU_cmd.h"
#include "HA_bt.h"
#include "HA_vs1063a.h"
#include "HA_SineTest.h"
#include "InitDevice.h"

#include "bt_command_send.h"

#include "HA300_BSP.h"

#include "bt_app.h"
#include "EnergyMode.h"
#include "LED_PWM.h"

#include "eusart.h"

#include "em_adc.h"

// ADC channel number
#define ADC_CHANNEL 0

#define DAC_Write_Address 0x30  // (0x18 << 1) Write
#define DAC_Read_Address  0x31  // (0x18 << 1) | 1

#define XVF3800_I2C_ADDR_7BIT 0x2C
#define XVF_READ_CMD_BIT  0x80
#define XVF_STATUS_OK  0

#define RESOURCE_ID_AEC_NUM_FARENDS  0x21
#define CMD_ID_AEC_NUM_FARENDS  0xC8
#define XVF_REPLY_LEN_AEC_NUM_FARENDS  5

//------------------------------------------------------------
/* =========================================================
 *  SSD1315 ����
 * ========================================================= */
#define SSD1315_I2C_ADDR         0x3C

#define OLED_WIDTH               64
#define OLED_HEIGHT              32
#define OLED_PAGES               (OLED_HEIGHT / 8)


#define SSD1315_RAM_WIDTH   128
#define OLED_VISIBLE_PAGES  4   // 32 pixel / 8 = 4 pages


/* I2C �� ����: ���忡 �°� ���� */
#define OLED_I2C                 I2C0

/* SSD1315 control byte */
#define SSD1315_CONTROL_CMD      0x00
#define SSD1315_CONTROL_DATA     0x40

/* ȭ�� ����: 64 x 32 / 8 = 256 bytes */
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];


/*
 * SSD1315 ���� RAM�� 128 �÷��� �� �����Ƿ�
 * ���� ǥ�� ���� �÷��� ������ �� �ְ� ��
 *
 * ��:
 * 0   = RAM 0~63�� �׸�
 * 32  = RAM 32~95�� �׸�
 *
 * ȭ���� ����/���������� �и��� �� �� ����
 */
#define OLED_X_OFFSET           32

static uint8_t g_contrast = 0x7F;

/* =========================================================
 *  "Hello"�� �ʿ��� �ּ� 5x7 ��Ʈ
 *  �� ���ڴ� 5����Ʈ, LSB�� ���� �ȼ�
 * ========================================================= */
static const uint8_t font_H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
static const uint8_t font_e[5] = {0x38, 0x54, 0x54, 0x54, 0x18};
static const uint8_t font_l[5] = {0x00, 0x41, 0x7F, 0x40, 0x00};
static const uint8_t font_o[5] = {0x38, 0x44, 0x44, 0x44, 0x38};
static const uint8_t font_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

/* EQ status display용 추가 폰트 (5x7, LSB=top) */
static const uint8_t font_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};  /* E */
static const uint8_t font_Q[5] = {0x3E, 0x41, 0x49, 0x45, 0x3B};  /* Q */
static const uint8_t font_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};  /* O */
static const uint8_t font_N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};  /* N */
static const uint8_t font_F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};  /* F */

//------------------------------------------------------------



extern bool E_Save;
extern long E_SaveCount;
extern bool E_1st_Save;
extern long E_1st_SaveCount;

extern bool BT_working_flag;


extern uint32_t BT_SPP_RxCounter;


// eeprom emulation begin
#include "eeprom_emulation.h"

void moveInterruptVectorToRam(void);

extern unsigned char pic[];
extern unsigned char geniesori_logo_64x32[];
extern unsigned char num[]; // made by CLAUDE

void Display_Picture(unsigned char pic[]);
void Initial(void);
void clear(void);


EE_Variable_TypeDef var1, var2, var3, boolVar; // Define the non-volatile variables.
EE_Variable_TypeDef DB_Index, Group_ID_Sub, MicBT_Index; // Define the non-volatile variables.
EE_Variable_TypeDef DB_Vol, DB_NC; // Define the non-volatile variables.
EE_Variable_TypeDef DB_Vol_Left, DB_Vol_Right; // Define the non-volatile variables.

uint16_t             readValue;
uint16_t             DB_Index_readValue;
uint16_t             Group_Zone_readValue;
uint16_t             MicBT_Index_readValue=0;  // Mic = 0, BT = 1


uint16_t             DB_Vol_readValue;
uint16_t             DB_NC_readValue;
uint16_t             DB_Vol_Left_readValue;
uint16_t             DB_Vol_Right_readValue;

// eeprom emulation end

/// USTIMER configuration option. Use this define to select a TIMER resource.
#define USTIMER_TIMER USTIMER_TIMER3

//#include "ustimer.h"

/**************************************************************************//**
 *
 * This example shows how a CDC based USB to Serial port adapter can be
 * implemented.
 *
 * Use the file EFM32-Cdc.inf to install a USB serial port device driver
 * on the host PC.
 *
 * This implementation uses DMA to transfer data between USART1 and memory
 * buffers.
 *
 *****************************************************************************/
// USBD_Read(CDC_EP_DATA_OUT, (void*)uartRxData, 2, NULL);

//int rx_buf[200];
int	rValue; // return command running value
//char E_Save=0;
//long E_SaveCount=0;
extern uint8_t BT_nspk_status;

extern volatile uint8_t eusartRxCount;

static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,
  .usbStateChange  = USBCDC_StateChangeEvent,
  .setupCmd        = USBCDC_SetupCmd,
  .isSelfPowered   = NULL,
  .sofInt          = NULL
};

static const USBD_Init_TypeDef usbInitStruct =
{
  .deviceDescriptor    = &USBDESC_deviceDesc,
  .configDescriptor    = USBDESC_configDesc,
  .stringDescriptors   = USBDESC_strings,
  .numberOfStrings     = sizeof(USBDESC_strings)/sizeof(void*),
  .callbacks           = &callbacks,
  .bufferingMultiplier = USBDESC_bufferingMultiplier,
  .reserved            = 0
};

/** Flag used to indicate ADC is finished */
static volatile bool adcConversionComplete = false;

volatile uint32_t msTicks; /* counts 1ms timeTicks */

/* Local prototypes */
void Delay(uint32_t dlyTicks);

#if 1
int putchar(char c) {
	// USART_Tx(USART1, c);
}
int puts(char *c) {
	while(*c) putchar(*c++);
}
#else
int putchar(char c) {
	USART_Tx(USART1, c);
}

int puts(char *c) {

	uint16_t len;
	while(*c) len++;

	BT_SendSPPData(c, len, 0);
}
#endif


/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
//void Delay(uint32_t dlyTicks)
//{
//  uint32_t curTicks;
//
//  curTicks = msTicks;
//  while ((msTicks - curTicks) < dlyTicks) ;
//}

char txbuffer[127]="abcdefghijklmnopqrstuvwxyz0123456789";
char rxbuffer[USB_RX_BUF_SIZE];


bool mic_mute_state = true;
bool BIS_state = false;
bool bt_dsp_reset = false;
bool int_flag = false;
bool dsp_lowpower_flag = false;

bool BAT_Sound_Flag = false;


extern uint8_t BTAPP_task_status;
extern uint8_t ang_input;   // 0 : Line In connect, 1 : Line In Disconnect
extern uint8_t  BT_linkIndex;


//__STATIC_INLINE void USBD_Write(CDC_EP_DATA_IN,(uint8_t*)txbuffer, strlen(txbuffer),NULL);

//char txbuffer[127];

/*
#define printf(...) do { \
        sprintf(txbuffer, __VA_ARGS__); \
        USBD_Write(CDC_EP_DATA_IN,(uint8_t*)txbuffer, strlen(txbuffer),NULL); \
    } while (0)
*/
//#define putch(CH) USBD_Write(CDC_EP_DATA_IN,CH, 1,NULL)

void command_check(){

	int i;

//	printf("Cmd=%s\r\n", CmdBuf);


	for(i=0; i<nRxCnt; i++) {
		if(CmdBuf[i] == 0x0a || CmdBuf[i] == 0x0d ) CmdBuf[i] = '\0';
	}

	Cmd_flag = command_compare();
	rValue = HA_command_running();
	if( rValue < 0 ) {
		//printf("[NG]=%d\r\n", Cmd_flag);
	} else {
		//printf("[OK]=%d\r\n", Cmd_flag);
	}
//	printf("Cmd=%s\r\n", CmdBuf);

	// rx command buffer �ʱ�ȭ
	memset(CmdBuf,0,COMMAND_BUF_SIZE);
	memset(rxbuffer,0,USB_RX_BUF_SIZE);
	for(i=0; i<10;i++) memset(argv[i],0,strlen(argv[i]));
	nRxCnt = 0;

}

void receivedprecess() {

	int i=0;

#if 1	
	while(rxbuffer[i]!= 0x0a) {
		CmdBuf[nRxCnt++] = rxbuffer[i++];  // to upper case 20150417
		//if(i>(COMMAND_BUF_SIZE-1)) i=0;
	}
	CmdBuf[nRxCnt++] = rxbuffer[i];  // to upper case 20150417
	if(rxbuffer[i]== 0x0a) {
		incoming_cmd_flag = 1;
	}
#else
	CmdBuf[nRxCnt++] = rxbuffer[0];  // to upper case 20150417
	incomingByte = rxbuffer[0];

	if (incomingByte == 0x0a || incomingByte == 0x0d)
	{

		incoming_cmd_flag = 1;
		//command_check();

	}
#endif
}


/**************************************************************************//**
* @brief Unified GPIO Interrupt handler (pushbuttons)
*        PB0 Starts selected test
*        PB1 Cycles through the available tests
*****************************************************************************/
void GPIO_Unified_IRQ(void)
{
	  /* Get and clear all pending GPIO interrupts */
	  uint32_t interruptMask = GPIO_IntGet(); //  return GPIO->IF;
	  delayUs(200);
	//  KeyScan(interruptMask);
	  GPIO_IntClear(interruptMask);           // GPIO->IFC = flags;
}


/**************************************************************************//**
* @brief GPIO Interrupt handler for even pins
*****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
* @brief GPIO Interrupt handler for odd pins
*****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
* @brief Setup GPIO interrupt for pushbuttons.
*****************************************************************************/
void gpioSetup(void)
{

  /* Configure PC14 as input and enable interrupt  */  //PC14 Up Button
  GPIO_PinModeSet(UP_BUTTON_PORT, UP_BUTTON_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(UP_BUTTON_PORT, UP_BUTTON_PIN, true, true, true);

  /* Configure PC15 as input and enable interrupt */   //PC15 Down Button
  GPIO_PinModeSet(DOWN_BUTTON_PORT, DOWN_BUTTON_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(DOWN_BUTTON_PORT, DOWN_BUTTON_PIN, true, true, true);

  /* Configure PD7 as input and enable interrupt */   //PD7 MODE Button
  GPIO_PinModeSet(MODE_BUTTON_PORT, MODE_BUTTON_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(MODE_BUTTON_PORT, MODE_BUTTON_PIN, true, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);   //PB0 Interrupt .. 0/2/4/6/8/
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);    //PB10 Interrupt .. 1/3/5/7/9/
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}



/***************************************************************************//**
* @brief
*   Configure ADC usage for this application.
*******************************************************************************/
static void ADCConfig(void)
{
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 7 MHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(7000000, 0);

  /* WARMUPMODE must be set to Normal according to ref manual before */
  /* entering EM2. In this example, the warmup time is not a big problem */
  /* due to relatively infrequent polling. Leave at default NORMAL, */

  ADC_Init(ADC0, &init);

  /* Init for single conversion use, measure VDD/3 with 1.25 reference. */
  singleInit.reference  = _ADC_SINGLECTRL_REF_5VDIFF;
  //singleInit.input      = adcSingleInpVDDDiv3;
  singleInit.input      = adcSingleInpCh5;
  singleInit.resolution = adcRes12Bit;

  /* The datasheet specifies a minimum aquisition time when sampling vdd/3 */
  /* 32 cycles should be safe for all ADC clock frequencies */
  singleInit.acqTime = adcAcqTime32;

  ADC_InitSingle(ADC0, &singleInit);
}



void initI2C(void)
{
  // Using PD6 (SDA) and PD7 (SCL)
  GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);	// SDA
  GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL


  I2C_Init_TypeDef init = I2C_INIT_DEFAULT;
  init.freq = I2C_FREQ_STANDARD_MAX;   // 100kHz
  I2C_Reset(I2C0);

	
  // Enable pins at location 1 as specified in datasheet
  I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN;
  I2C0->ROUTE = (I2C0->ROUTE & (~_I2C_ROUTE_LOCATION_MASK)) | I2C_ROUTE_LOCATION_LOC4;

  I2C_Init(I2C0, &init);

}


static void I2C0_InitPins(void)
{
    /*
     * ���� ��:
     * SCL = PC1
     * SDA = PC0
     * ���� ���忡 �°� �ٲټ���.
     */
    GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);	// SDA
    GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);	// SCL

    I2C_Init_TypeDef init = I2C_INIT_DEFAULT;
    init.freq = I2C_FREQ_STANDARD_MAX;   // 100kHz

    I2C_Reset(I2C0);

    I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC4;
    I2C_Init(I2C0, &init);
}


static bool I2C_WriteBytes(uint8_t addr7, const uint8_t *data, uint8_t len)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    seq.addr  = addr7 << 1;
    seq.flags = I2C_FLAG_WRITE;
    seq.buf[0].data = (uint8_t *)data;
    seq.buf[0].len  = len;

    ret = I2C_TransferInit(I2C0, &seq);
    while (ret == i2cTransferInProgress) {
        ret = I2C_Transfer(I2C0);
    }

    return (ret == i2cTransferDone);
}


bool I2C_WriteReadBytes(uint8_t addr7,
                        const uint8_t *wdata, uint8_t wlen,
                        uint8_t *rdata, uint8_t rlen)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    seq.addr  = addr7 << 1;
    seq.flags = I2C_FLAG_WRITE_READ;

    seq.buf[0].data = (uint8_t *)wdata;
    seq.buf[0].len  = wlen;

    seq.buf[1].data = rdata;
    seq.buf[1].len  = rlen;

    ret = I2C_TransferInit(I2C0, &seq);
    while (ret == i2cTransferInProgress) {
        ret = I2C_Transfer(I2C0);
    }

    return (ret == i2cTransferDone);
}


/*
 * XVF3800�� Ư�� int32 read command�� �д� ���� �Լ�
 *
 * XMOS DFU over I2C ������ �Ϲ� read command ���Ŀ� �°� ����:
 *  1) WRITE phase:
 *     [resource_id]
 *     [command_id | 0x80]
 *     [expected_reply_len]
 *
 *  2) READ phase (repeated START):
 *     [status]
 *     [payload0]
 *     [payload1]
 *     [payload2]
 *     [payload3]
 */
static bool XVF3800_ReadInt32(uint8_t resource_id,
                              uint8_t command_id,
                              int32_t *out_value,
                              uint8_t *out_status)
{
    uint8_t tx[3];
    uint8_t rx[5];

    tx[0] = resource_id;
    tx[1] = (uint8_t)(command_id | XVF_READ_CMD_BIT);
    tx[2] = XVF_REPLY_LEN_AEC_NUM_FARENDS;

    if (!I2C_WriteReadBytes(XVF3800_I2C_ADDR_7BIT, tx, sizeof(tx), rx, sizeof(rx))) {
        return false;
    }

    if (out_status) {
        *out_status = rx[0];
    }

    /*
     * payload�� int32 1��
     * ������� XMOS control param payload�� ���� �������� ���� ���õ� �������� �� ã�ұ� ������
     * �Ϲ����� little-endian �������� ó��.
     * ���� ���� ���������� ������ endian�� Ȯ���ؾ� ��.
     */
    *out_value =
        ((int32_t)rx[1]) |
        ((int32_t)rx[2] << 8) |
        ((int32_t)rx[3] << 16) |
        ((int32_t)rx[4] << 24);

    return true;
}


bool XVF3800_Read_AEC_NUM_FARENDS(int32_t *num_farends)
{
    uint8_t status = 0xFF;

    if (!XVF3800_ReadInt32(RESOURCE_ID_AEC_NUM_FARENDS,
                           CMD_ID_AEC_NUM_FARENDS,
                           num_farends,
                           &status)) {
        return false;
    }

    if (status != XVF_STATUS_OK) {
        return false;
    }

    return true;
}


 void initial_setup()
{
	 MSC_Init();

	VOL_Count=MAX_VOL_Count - MIN_VOL_Count + 1;
//	GAP_Volume = (int) ((MIN_Volume - MAX_Volume + 1) /VOL_Count);
	GAP_Volume = (int) ((MIN_Volume - MAX_Volume) /VOL_Count);

}


 void initial_DSP(void)
 {
// 	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

// 	CMU_ClockEnable(cmuClock_HFPER, true);
// 	CMU_ClockEnable(cmuClock_USART0, true);
// 	CMU_ClockEnable(cmuClock_GPIO, true);

 	USART_InitSync_TypeDef initsync = USART_INITSYNC_DEFAULT;
     initsync.msbf = 1;
 	USART_InitSync(USART0, &initsync);

 	SET_DSP_SCLK(1);	// GPIO_PinModeSet(gpioPortE,    12,    gpioModePushPull,1);    //  SCLK          //PE12
 	SET_DSP_CS(1);		// GPIO_PinModeSet(gpioPortE,    13,    gpioModePushPull,1);    //  SCS
 	SET_DSP_MISO(0);	// GPIO_PinModeSet(gpioPortE,    11,    gpioModeInputPull,0);   //  MISO //RX     //PE11
 	SET_DSP_MOSI(1);	// GPIO_PinModeSet(gpioPortE,    10,    gpioModePushPull,1);    //  MOSI //TX     //PE10

  	// Enable signals CLK, RX, TX
 	USART0->ROUTE |= USART_ROUTE_CLKPEN | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

 }





/* =========================================================
 *  SSD1306 ������ �Լ�
 * ========================================================= */
static bool ssd1315_write_command(uint8_t cmd)
{
    uint8_t packet[2];
    packet[0] = SSD1315_CONTROL_CMD;
    packet[1] = cmd;
    return I2C_WriteBytes(SSD1315_I2C_ADDR, packet, 2);
}

static bool ssd1315_write_command_list(const uint8_t *cmds, uint16_t count)
{
    uint16_t i;
    for (i = 0; i < count; i++) {
        if (!ssd1315_write_command(cmds[i])) {
            return false;
        }
    }
    return true;
}

static bool ssd1315_write_data(const uint8_t *data, uint16_t len)
{
    uint8_t packet[17]; /* control 1 + data �ִ� 16����Ʈ�� ���� */
    uint16_t offset = 0;

    while (offset < len) {
        uint16_t chunk = len - offset;
        if (chunk > 16) {
            chunk = 16;
        }

        packet[0] = SSD1315_CONTROL_DATA;
        memcpy(&packet[1], &data[offset], chunk);

        if (!I2C_WriteBytes(SSD1315_I2C_ADDR, packet, chunk + 1)) {
            return false;
        }

        offset += chunk;
    }
    return true;
}

/* =========================================================
 *  SSD1306 �ʱ�ȭ
 * ========================================================= */
static bool ssd1315_init(void)
{
    /* Display OFF */
    if (!ssd1315_write_command(0xAE)) return false;

    /* Set Display Clock Divide Ratio / Oscillator Frequency */
    if (!ssd1315_write_command(0xD5)) return false;
    if (!ssd1315_write_command(0x80)) return false;

    /* Set Multiplex Ratio = 32 lines */
    if (!ssd1315_write_command(0xA8)) return false;
    if (!ssd1315_write_command(0x1F)) return false;   /* 0x1F + 1 = 32 */

    /* Set Display Offset = 0 */
    if (!ssd1315_write_command(0xD3)) return false;
    if (!ssd1315_write_command(0x00)) return false;

    /* Set Display Start Line = 0 */
    if (!ssd1315_write_command(0x40)) return false;

    /* Charge Pump Setting
       SSD1315 datasheet: 8Dh ���� 14h/94h/95h �迭�� enable ���� */
    if (!ssd1315_write_command(0x8D)) return false;
    if (!ssd1315_write_command(0x14)) return false;

    /* Memory Addressing Mode = Horizontal */
    if (!ssd1315_write_command(0x20)) return false;
    if (!ssd1315_write_command(0x00)) return false;

    /* Segment Re-map */
    if (!ssd1315_write_command(0xA1)) return false;

    /* COM Output Scan Direction remap */
    if (!ssd1315_write_command(0xC8)) return false;

    /* COM Pins Hardware Configuration
       64x32 �гο� �ǹ��� */
    if (!ssd1315_write_command(0xDA)) return false;
    if (!ssd1315_write_command(0x12)) return false;  // changed

    /* Contrast */
    if (!ssd1315_write_command(0x81)) return false;
    if (!ssd1315_write_command(0x7F)) return false;  //g_contrast

    /* Pre-charge Period */
    if (!ssd1315_write_command(0xD9)) return false;
    if (!ssd1315_write_command(0xF1)) return false;

    /* VCOMH Deselect Level */
    if (!ssd1315_write_command(0xDB)) return false;
    if (!ssd1315_write_command(0x40)) return false;

    /* Resume to RAM content display
       SSD1315 datasheet: A4h = GDDRAM ���� ǥ�� */
    if (!ssd1315_write_command(0xA4)) return false;

    /* Normal Display */
    if (!ssd1315_write_command(0xA6)) return false;

    /* Deactivate Scroll */
    if (!ssd1315_write_command(0x2E)) return false;

    /* Display ON */
    if (!ssd1315_write_command(0xAF)) return false;

    return true;
}

/* =========================================================
 *  �׷��� �Լ�
 * ========================================================= */
static void ssd1315_clear_buffer(void)
{
    memset(oled_buffer, 0x00, sizeof(oled_buffer));
}

static void ssd1315_draw_pixel(uint8_t x, uint8_t y, bool color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return;
    }

    uint16_t index = x + (y / 8) * OLED_WIDTH;
    uint8_t bit = 1 << (y & 7);

    if (color) {
        oled_buffer[index] |= bit;
    } else {
        oled_buffer[index] &= ~bit;
    }
}

static void ssd1315_draw_char_5x7(uint8_t x, uint8_t y, const uint8_t glyph[5])
{
    uint8_t col, row;
    for (col = 0; col < 5; col++) {
        for (row = 0; row < 7; row++) {
            bool pixel_on = (glyph[col] >> row) & 0x01;
            ssd1315_draw_pixel(x + col, y + row, pixel_on);
        }
    }
}

static void ssd1315_draw_char(uint8_t x, uint8_t y, char c)
{
    switch (c) {
        case 'H': ssd1315_draw_char_5x7(x, y, font_H); break;
        case 'e': ssd1315_draw_char_5x7(x, y, font_e); break;
        case 'l': ssd1315_draw_char_5x7(x, y, font_l); break;
        case 'o': ssd1315_draw_char_5x7(x, y, font_o); break;
        case ' ': ssd1315_draw_char_5x7(x, y, font_space); break;
        case 'E': ssd1315_draw_char_5x7(x, y, font_E); break;
        case 'Q': ssd1315_draw_char_5x7(x, y, font_Q); break;
        case 'O': ssd1315_draw_char_5x7(x, y, font_O); break;
        case 'N': ssd1315_draw_char_5x7(x, y, font_N); break;
        case 'F': ssd1315_draw_char_5x7(x, y, font_F); break;
        default:  ssd1315_draw_char_5x7(x, y, font_space); break;
    }
}

static void ssd1315_draw_string(uint8_t x, uint8_t y, const char *str)
{
    while (*str) {
        ssd1315_draw_char(x, y, *str);
        x += 6; /* 5�ȼ� + 1�ȼ� ���� */
        str++;
    }
}

/*
 * XVF3800 EQ 상태를 OLED에 표시
 * eq_on=true  → "EQ ON"  (상단 좌측)
 * eq_on=false → "EQ OFF"
 * 기존 화면 위에 덮어쓰지 않고 상단 영역(y=0~8)만 갱신
 */
void oled_show_eq_status(bool eq_on)
{
    ssd1315_clear_buffer();
    if (eq_on) {
        ssd1315_draw_string(4, 12, "EQ ON");
    } else {
        ssd1315_draw_string(4, 12, "EQ OFF");
    }
    ssd1315_flush_to_display();
}

/* 숫자 폰트(num[]) 한 자리를 (x,y) 위치에 그리기 (8x16 px) */ // made by CLAUDE
static void ssd1315_draw_digit(uint8_t x, uint8_t y, uint8_t digit)
{
    uint8_t col, row;
    for (col = 0; col < 8; col++) {
        uint8_t top = num[16 * digit + col];
        uint8_t bot = num[16 * digit + 8 + col];
        for (row = 0; row < 8; row++) {
            ssd1315_draw_pixel(x + col, y + row,     (top >> row) & 1);
            ssd1315_draw_pixel(x + col, y + 8 + row, (bot >> row) & 1);
        }
    }
}

static bool ssd1315_update_screen(void)
{
    /*
     * 64x32 ���� ����
     * column: 0 ~ 63
     * page  : 0 ~ 3
     */
    if (!ssd1315_write_command(0x21)) return false; /* Set column address */
    if (!ssd1315_write_command(0x00)) return false; /* Start column */
    if (!ssd1315_write_command(OLED_WIDTH - 1)) return false; /* End column */

    if (!ssd1315_write_command(0x22)) return false; /* Set page address */
    if (!ssd1315_write_command(0x00)) return false; /* Start page */
    if (!ssd1315_write_command(OLED_PAGES - 1)) return false; /* End page */

    return ssd1315_write_data(oled_buffer, sizeof(oled_buffer));
}

/* oled_buffer를 실제 표시 영역(col 32~95)에 전송 */ // made by CLAUDE
static bool ssd1315_flush_to_display(void)
{
    /* Horizontal addressing mode */
    if (!ssd1315_write_command(0x20)) return false;
    if (!ssd1315_write_command(0x00)) return false;

    if (!ssd1315_write_command(0x21)) return false;
    if (!ssd1315_write_command(OLED_X_OFFSET)) return false;
    if (!ssd1315_write_command(OLED_X_OFFSET + OLED_WIDTH - 1)) return false;

    if (!ssd1315_write_command(0x22)) return false;
    if (!ssd1315_write_command(0x00)) return false;
    if (!ssd1315_write_command(OLED_PAGES - 1)) return false;

    return ssd1315_write_data(oled_buffer, sizeof(oled_buffer));
}

/* 64x32 ǥ�� ������ clear */
static bool ssd1315_clear_visible_area_64x32(void)
{
    uint8_t zero[OLED_WIDTH];
    for (uint16_t i = 0; i < OLED_WIDTH; i++) {
        zero[i] = 0x00;
    }

    /* Horizontal addressing mode */
    if (!ssd1315_write_command(0x20)) return false;
    if (!ssd1315_write_command(0x00)) return false;

    /* Column: 0 ~ 63 */
    if (!ssd1315_write_command(0x21)) return false;
    if (!ssd1315_write_command(0x00)) return false;
    if (!ssd1315_write_command(0x3F)) return false;

    /* Page: 0 ~ 3  (32 pixels / 8) */
    if (!ssd1315_write_command(0x22)) return false;
    if (!ssd1315_write_command(0x00)) return false;
    if (!ssd1315_write_command(0x03)) return false;

    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        if (!ssd1315_write_data(zero, OLED_WIDTH)) {
            return false;
        }
    }

    return true;
}

static bool ssd1315_clear_all_ram_128x32(void)
{
    uint8_t zero_line[SSD1315_RAM_WIDTH];

    for (uint16_t i = 0; i < SSD1315_RAM_WIDTH; i++) {
        zero_line[i] = 0x00;
    }

    /* Horizontal addressing mode */
    if (!ssd1315_write_command(0x20)) return false;
    if (!ssd1315_write_command(0x00)) return false;

    /* Column address: 0 ~ 127 */
    if (!ssd1315_write_command(0x21)) return false;
    if (!ssd1315_write_command(0x00)) return false;
    if (!ssd1315_write_command(0x7F)) return false;

    /* Page address: 0 ~ 3 */
    if (!ssd1315_write_command(0x22)) return false;
    if (!ssd1315_write_command(0x00)) return false;
    if (!ssd1315_write_command(0x03)) return false;

    for (uint8_t page = 0; page < OLED_VISIBLE_PAGES; page++) {
        if (!ssd1315_write_data(zero_line, SSD1315_RAM_WIDTH)) {
            return false;
        }
    }

    return true;
}

/* =========================================================
 * 64x32 �̹��� ǥ��
 * page format ������ �״�� ����
 * ========================================================= */
static bool ssd1315_draw_image_64x32(const uint8_t *img, uint8_t x_offset)
{
    /*
     * img�� 64x32 = 64 * 4 = 256 bytes
     * ���������� 64����Ʈ�� ����
     */
    for (uint8_t page = 0; page < OLED_PAGES; page++) {

        if (!ssd1315_write_command(0x20)) return false;   /* Horizontal addressing */
        if (!ssd1315_write_command(0x00)) return false;

        if (!ssd1315_write_command(0x21)) return false;   /* Column address */
        if (!ssd1315_write_command(x_offset)) return false;
        if (!ssd1315_write_command(x_offset + OLED_WIDTH - 1)) return false;

        if (!ssd1315_write_command(0x22)) return false;   /* Page address */
        if (!ssd1315_write_command(page)) return false;
        if (!ssd1315_write_command(page)) return false;

        if (!ssd1315_write_data(&img[page * OLED_WIDTH], OLED_WIDTH)) {
            return false;
        }
    }

    return true;
}

/****************************************************************************
 * @brief main - the entrypoint after reset.
 *****************************************************************************/

int main(void)
{

	int i, j;
	bool Mode_Key_Button_initial_state=0;  
	bool Up_Key_Button_initial_state=0;  
	bool Down_Key_Button_initial_state=0;  
	bool BT_LED1_initial_state=0;  

	uint32_t sample;

	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO ); 
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
 	CMU_ClockEnable(cmuClock_HFPER, true);
 	CMU_ClockEnable(cmuClock_USART0, true);
 	CMU_ClockEnable(cmuClock_GPIO, true);
      CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_RTC, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);

 
#if defined( EFM32_ADC_ENABLE )
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADCConfig();

	// Configure the GPIO pin used for analog input  PD5
	GPIO_PinModeSet(gpioPortD, 5, gpioModeInput, 0);
#endif


	/* Setup SysTick Timer for 10 msec interrupts  */
	//  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)){while (1) ; }
	//  NVIC_EnableIRQ(SysTick_IRQn);
	setup_systick_timer();

#if 1 // EEPROM Emulation ON/OFF
	/* Move the interrupt vector table to RAM to safely handle interrupts
	* while performing write/erase operations on flash */

	moveInterruptVectorToRam();
	/* Enables the flash controller for writing. */
	MSC_Init();

	/* Initialize the eeprom emulator using 3 pages. */
	if ( !EE_Init(3) ) {

	/* If the initialization fails we have to take some measure
	 * to obtain a valid set of pages. In this example we simply
	 * format the pages */
	EE_Format(3);
	}


	EE_DeclareVariable(&DB_Index);
	EE_DeclareVariable(&Group_ID_Sub);
	EE_DeclareVariable(&MicBT_Index);

	EE_DeclareVariable(&DB_Vol);

	EE_DeclareVariable(&DB_Vol_Left);
	EE_DeclareVariable(&DB_Vol_Right);
	EE_DeclareVariable(&DB_NC);
#endif


	gpioSetup();
	PWM_init();
 
	// VS8063A initial
 	USART_InitSync_TypeDef initsync = USART_INITSYNC_DEFAULT;
     initsync.msbf = 1;
 	USART_InitSync(USART0, &initsync);
	GPIO_PinModeSet(VS_DREQ_PORT,  VS_DREQ_PIN,  gpioModeInput,0);    //DREQ_SET
 	SET_DSP_SCLK(1);	// GPIO_PinModeSet(gpioPortE,    12,    gpioModePushPull,1);    //  SCLK  
 	SET_DSP_CS(1);		// GPIO_PinModeSet(gpioPortE,    13,    gpioModePushPull,1);    //  SCS
 	SET_DSP_MISO(0);	// GPIO_PinModeSet(gpioPortE,    11,    gpioModeInputPull,0);   //  MISO 
 	SET_DSP_MOSI(1);	// GPIO_PinModeSet(gpioPortE,    10,    gpioModePushPull,1);    //  MOSI 
  	// Enable signals CLK, RX, TX
 	USART0->ROUTE |= USART_ROUTE_CLKPEN | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

//	SetColor(WHITE);
//	PAM8019_Volume(500);
	
//	while(1);

	XMOS_Reset();
	DSP_Reset();  // DSP BT reset(High_Low_High)

	delayMs(2000);

// 	while(1);

	//initI2C();
	I2C0_InitPins();

	delayMs(100);

    if (!ssd1315_init()) {
        while (1) {
            __NOP();
        }
    }

    ssd1315_clear_buffer();

//	SetColor(WHITE);
//	PAM8019_Volume(500);
	SET_LED_GREEN(0);
	ssd1315_write_command(0xA5);
	delayMs(1000);


    /* RAM ǥ�� ������ ���� 0���� ���� */
//	if (!ssd1315_clear_visible_area_64x32()) return false;
	if (!ssd1315_clear_all_ram_128x32()) {
	    while (1) { __NOP(); }
	}
	ssd1315_write_command(0xA4);
	delayMs(500);

	/* �ΰ� ǥ�� */
	if (!ssd1315_draw_image_64x32(geniesori_logo_64x32, OLED_X_OFFSET)) {
	    while (1) { __NOP(); }
	}
	delayMs(1000);
	SET_LED_GREEN(1);
	delayMs(500);


	SET_LED_RED(0);
	if (!ssd1315_clear_all_ram_128x32()) {
	    while (1) { __NOP(); }
	}
	ssd1315_write_command(0xA4);
//	delayMs(1000);
	SET_LED_RED(1);


///////////////////////

    /* "Hello" ��� ��ġ */
//    ssd1315_draw_string(10, 10, "Hello");

///////////////////////

//	Initial();
//	clear();


	VOL_Count=MAX_VOL_Count - MIN_VOL_Count + 1;
	GAP_Volume = (int) ((MIN_Volume - MAX_Volume) /VOL_Count);

	VOL_Left_Count=VOL_Count;
	VOL_Right_Count=VOL_Count;


	EE_Read(&DB_Index, &DB_Index_readValue);
	if (DB_Index_readValue > 6)
	{
		DB_Index_readValue = 4;
	}
	EQ_Select_index = DB_Index_readValue ;


	EE_Read(&DB_Vol_Left, &DB_Vol_Left_readValue);
	if (DB_Vol_Left_readValue > 16)
	{
		DB_Vol_Left_readValue = 6;
	}
	VOL_Left_number = DB_Vol_Left_readValue ;


	EE_Read(&DB_Vol_Right, &DB_Vol_Right_readValue);
	if (DB_Vol_Right_readValue > 16)
	{
			 DB_Vol_Right_readValue = 6;
	}
	VOL_Right_number = DB_Vol_Right_readValue ;


 	GN_Working_State = GN_STATUS_IDLE;

	Time_1SEC_timer_count = 0;  // run 1 sec timer

	mic_mute_state = true;
	SET_PAM8019_MUTE(1);   // 1: outputs Hi-Z, 0: outputs enabled
	SET_PAM8019_EN(0); 	// 0: chip shutdown with outputs Hi-Z, 1: outputs enabled 

	recovery_Mic_input(0); // 1 : Mic, 0: Line   for TX & RX

	VOL_Left_number = 10;
	VOL_Right_number = 10;


	Master_Left_Volume = MIN_Left_Volume - VOL_Left_number * GAP_Volume;
	Master_Right_Volume = MIN_Right_Volume - VOL_Right_number * GAP_Volume;
	vs_set_volume(Master_Left_Volume, Master_Right_Volume);

#if 0
	for (i=0; i<1; i++) {
		SetColor(RED);
		delayMs(500);
		SetColor(BLACK);
		delayMs(500);
	}
#endif

	mic_mute_state = false;
	SET_PAM8019_MUTE(0);   // High: outputs Hi-Z, Low: outputs enabled
	SET_PAM8019_EN(1); 	// Low: chip shutdown with outputs Hi-Z, High: outputs enabled 

	SOUND_LEVEL_timer_count = 0;

	/* �ΰ� ǥ�� */
	if (!ssd1315_draw_image_64x32(geniesori_logo_64x32, OLED_X_OFFSET)) {
	    while (1) { __NOP(); }
	}
	//Display_Picture(pic);
	//Display_Picture(geniesori_logo_64x32);

	/* 1~10을 1초 간격으로 OLED에 표시 */ // made by CLAUDE
	{
		uint8_t n;
		for (n = 1; n <= 10; n++) {
			ssd1315_clear_buffer();
			if (n < 10) {
				/* 한 자리: x=(64-8)/2=28, y=(32-16)/2=8 로 중앙 배치 */
				ssd1315_draw_digit(28, 8, n);
			} else {
				/* "10": '1' + '0', 두 자리(각 8px) + 간격 2px = 18px 폭
				 * x_start = (64-18)/2 = 23 */
				ssd1315_draw_digit(23, 8, 1);
				ssd1315_draw_digit(33, 8, 0);
			}
			ssd1315_flush_to_display();
			delayMs(1000);
		}
	}

	while(1) 	{
		KeyScan_Button();
		main_loop();

		if (Sound_Led_Flag == 1) {
			// Start ADC conversion
			ADC_Start(ADC0, adcStartSingle);

			/* Wait while conversion is active */
			while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;

			/* Get ADC result */
			sample = ADC_DataSingleGet(ADC0);

			if ( sample >= 500) {		
				SET_LED_RED(0);
			}
			else{ 
				SET_LED_RED(1);
			}
			Sound_Led_Flag = 0;
		}

	}

}

