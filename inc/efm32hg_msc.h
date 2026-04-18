
#ifndef EFM32HG_MSC_H
#define EFM32HG_MSC_H

typedef uint32_t eeprom_addr_t;

#define  MAPPED_EEPROM_START 0x0FE00000

extern uint8_t nvm_eeprom_read_byte(eeprom_addr_t addr);
extern void nvm_eeprom_write_byte(eeprom_addr_t address, uint8_t value);
#endif
