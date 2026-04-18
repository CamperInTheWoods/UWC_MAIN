
#include "compiler.h"
#include <string.h>
#include "em_msc.h"

#include "efm32hg_msc.h"

#define EEPROM_PAGE_SIZE 32

typedef uint32_t flash_addr_t;
/**
 * \weakgroup nvm_signature_group
 * @{
 */

/*

uint32_t *addr = (uint32_t *)0x0FE00000;
uint32_t data[] = { 0x00EF3200 };
MSC_Init();
MSC_ErasePage(addr);
MSC_WriteWord(addr, data, sizeof(data));
MSC_Deinit();



struct MyStruct
{
  uint8_t data1;
  uint16_t data2;
  uint32_t data3;
}

And I use this function

void WriteToFlash(uint32_t address, uint32_t *data)
{
  uint32_t *addr = (uint32_t *) address;
  MSC_Init();
  MSC_ErasePage(addr);
  MSC_WriteWord(addr, data,  sizeof(data));
  MSC_Deinit();
}
/////////////////////////////////////
void testFlash(void)
{

#define TESTBUFFERSIZE 100
uint32_t data[TESTBUFFERSIZE];
uint32_t *flashstartAddress = (uint32_t *)(0x0fe00000);
uint32_t *flashptr;
uint32_t i;

    for (i = 0; i < TESTBUFFERSIZE; i++)
         data[i] = i;

    // Configure HFRCO Band
    CMU_HFRCOBandSet(cmuHFRCOBand_28MHz); //required for the correct write timing

    MSC_Init(); //Enables the flash controller for writing.

    MSC_ErasePage(flashstartAddress); //erase the page first

    MSC_WriteWord(flashstartAddress, &data, TESTBUFFERSIZE*4); //write all of the data

    flashptr = flashstartAddress;
    for (i = 0 ; i < TESTBUFFERSIZE ; i++)
           data[i] = (*flashptr++); //read it back

}
///////////////////////////////////////
void WriteToFlash(uint32_t address, uint32_t *data, uint32_t size )
{
    uint32_t *addr = (uint32_t *) address;
    MSC_Init();
    MSC_ErasePage(addr);
    MSC_WriteWord(addr, data, size);
    MSC_Deinit();

}

void ReadFromFlash(uint32_t offset, uint32_t *data, uint32_t size)
{
    int i;
    uint32_t *addr;
    for (i = 0; i < size; i++ )
    {
      addr = (uint32_t *)(offset  + i*4);
      data[i] = *addr;
    }
}

///////////////////////////////////////

void ReadFromFlash(uint32_t offset, uint32_t *data, uint32_t size)
{
    int i;
    uint32_t *addr;
    for (i = 0; i < size; i++ )
    {
      addr = (uint32_t *)(offset  + i*4);
      data[i] = *addr;
    }
}
///////////////////////////////////////
oid testFlash(void)
{
#define TESTBUFFERSIZE 100
uint32_t data[TESTBUFFERSIZE];
uint32_t *flashstartAddress = (uint32_t *)(0x0fe00000);
uint32_t *flashptr;
uint32_t i;

    for (i = 0; i < TESTBUFFERSIZE; i++)
         data[i] = i;

    /// Configure HFRCO Band
    CMU_HFRCOBandSet(cmuHFRCOBand_28MHz); //required for the correct write timing

    MSC_Init(); //Enables the flash controller for writing.

    MSC_ErasePage(flashstartAddress); //erase the page first

    MSC_WriteWord(flashstartAddress, &data, TESTBUFFERSIZE*4); //write all of the data

    flashptr = flashstartAddress;
    for (i = 0 ; i < TESTBUFFERSIZE ; i++)
           data[i] = (*flashptr++); //read it back

}
///////////////////////////////////////
 */


/**
 * \brief Read one byte from EEPROM using mapped access.
 *
 * This function reads one byte from EEPROM using mapped access.
 *
 * \param  addr       EEPROM address, between 0 and EEPROM_SIZE
 *
 *  \return  Byte value read from EEPROM.
 */
uint8_t nvm_eeprom_read_byte(eeprom_addr_t addr)
{
	uint32_t data;
	data = *((volatile uint32_t*) MAPPED_EEPROM_START +addr);
	return (uint8_t)data;
}



/**
 * \brief Write one byte to EEPROM using IO mapping.
 *
 * This function writes one byte to EEPROM using IO-mapped access.
 * This function will cancel all ongoing EEPROM page buffer loading
 * operations, if any.
 *
 * \param  address    EEPROM address (max EEPROM_SIZE)
 * \param  value      Byte value to write to EEPROM.
 */
void nvm_eeprom_write_byte(eeprom_addr_t address, uint8_t value)
{
    MSC_ErasePage(MAPPED_EEPROM_START); //erase the page first
	MSC_WriteWord((uint32_t *)MAPPED_EEPROM_START+address,value,sizeof(8));
}



