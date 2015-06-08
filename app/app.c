/*
 * app.c
 *
 *  Author: Alexey Vaganov
 */ 
#include "common.h"
#include <avr/interrupt.h>
#include <util/delay.h>

int main(void)
{
	read_settings();
	cli_uart_init();
	modbus_uart_init();
	wifi_uart_init();
	sei();  // enable interrupts
	while(true)
	{
		if (cli_have_data) {
			cli_process();
		}
		if (modbus_have_data) {
			modbus_process();
		}
		if (wifi_have_data) {
			wifi_process();
		}
	}
}