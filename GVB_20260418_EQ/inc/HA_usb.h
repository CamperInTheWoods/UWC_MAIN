/*
 * HA_usb.h
 *
 *  Author: Rapiarno
 */ 

#ifndef HA_USB_H
#define HA_USB_H

#include  <stdbool.h>

//extern int put_char(char c, FILE *stream);
//extern int get_char(FILE *stream);
// Stream I/O : put_char, get_char
//FILE uart_str = FDEV_SETUP_STREAM(put_char, get_char, _FDEV_SETUP_RW);


static bool my_flag_autorize_cdc_transfert = false;

extern void task(void);
extern void usb_init(void);
extern bool my_callback_cdc_enable(void);
extern void my_callback_cdc_disable(void);
extern void task(void);

extern void my_vbus_action(bool b_high);

#endif
