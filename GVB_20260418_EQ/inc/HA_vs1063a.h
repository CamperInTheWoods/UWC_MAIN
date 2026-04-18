/*
 * HA_vs1063a.h
 *
 *  Author: Rapiarno
 */ 

#ifndef HA_VS1063A_H_
#define HA_VS1063A_H_

#include <stdint.h>

#define WREN  6
#define WRDI  4
#define RDSR  5
#define WRSR  1
#define READ  3
#define WRITE 2

// VS10xx SCI Registers
#define SCI_MODE            0x0
#define SCI_STATUS          0x1
#define SCI_BASS            0x2
#define SCI_CLOCKF          0x3
#define SCI_DECODE_TIME     0x4
#define SCI_AUDATA          0x5
#define SCI_WRAM            0x6
#define SCI_WRAMADDR        0x7
#define SCI_HDAT0           0x8
#define SCI_HDAT1           0x9
#define SCI_AIADDR          0xa
#define SCI_VOL             0xb
#define SCI_AICTRL0         0xc
#define SCI_AICTRL1         0xd
#define SCI_AICTRL2         0xe
#define SCI_AICTRL3         0xf

// SCI_MODE
#define SM_DIFF		        0x0001
#define SM_LAYER12	        0x0002
#define SM_RESET	        0x0004
#define SM_CANCEL	        0x0008
#define SM_PDOWN	        0x0010
#define SM_TESTS	        0x0020
#define SM_STREAM	        0x0040
#define SM_PLUSV	        0x0080
#define SM_DACT		        0x0100
#define SM_SDIORD	        0x0200
#define SM_SDISHARE	        0x0400
#define SM_SDINEW	        0x0800
#define SM_ENCODE           0x1000
//#define SM_UNKNOWN        0x2000
#define SM_LINE1            0x4000
#define SM_CLK_RANGE        0x8000

// SCI_STATUS
#define SS_REFERENCE_SEL    0x0001
#define SS_AD_CLOCK         0x0002
#define SS_APDOWN1          0x0004
#define SS_APDOWN2          0x0008
#define SS_VER1             0x0010
#define SS_VER2             0x0020
#define SS_VER3             0x0040
#define SS_VER4             0x0080
//#define SS_UNKNOWN        0x0100
//#define SS_UNKNOWN        0x0200
#define SS_VCM_DISABLE      0x0400
#define SS_VCM_OVERLOAD     0x0800
#define SS_SWING1           0x1000
#define SS_SWING2           0x2000
#define SS_SWING3           0x4000
#define SS_DO_NOT_JUMP      0x8000

// parametric_x addresses translated to WRAMADDR
#define endFillByte         0xc0c6
#define ogg_gain_offset     0xc0ea


// DSP_REGISTER
#define ADC_CONTROL 		0xC042
#define ADC_DATA_LEFT 		0xC043
#define ADC_DATA_RIGHT 		0xC044

#define ADC_MODU2_PD 			0x0010
#define ADC_MODU1_PD 			0x0008
#define ADC_DECIM_FACTOR_24K 	0x0006
#define ADC_DECIM_FACTOR_48K 	0x0004
#define ADC_DECIM_FACTOR_96K 	0x0002
#define ADC_DECIM_FACTOR_192K 	0x0000
#define ADC_ENABLE 				0x0001

#define VS_XRESET_PORT   (3)      //DSP_XRESET-PD7
#define VS_XRESET_PIN    (7)


#define VS_DREQ_PORT     (0)      //DSP_DREQ -PA1
#define VS_DREQ_PIN      (1)


#define VS_BSYNC_PORT    (5)      //DSP_BSYNC/XDC -PF2
#define VS_BSYNC_PIN     (2)

#define VS_CS_PORT       (4)      //PE13  //DSP_CS
#define VS_CS_PIN        (13)     // (13)

#define SCI_WRAMADDR     0x7
#define SCI_WRAM         0x6

#define READ  3
#define WRITE 2

extern char str1[];


//status_code_t spi_write_OneByte(SPI_t *spi, const uint8_t data);
//status_code_t spi_write_TwoByte(SPI_t *spi, const uint16_t value);
//uint8_t spi_read_OneByte(SPI_t *spi);
//uint16_t spi_read_TwoByte(SPI_t *spi);
void vs_Wait(void);
void DSP_Reset(void);
uint16_t ReadSci(uint8_t SCI_address);
void WriteSci(uint8_t SCI_address, uint16_t data);
void VS_XDCS_select_data(void);
void VS_XDCS_deselect_data(void);
int WriteSdi(uint8_t *data, uint8_t bytes);
uint16_t vs_read_wramaddr(uint16_t address);
void vs_write_wramaddr(uint16_t address, uint16_t value);


#endif /* VS1063A_H_ */
