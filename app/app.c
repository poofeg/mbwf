/*
 * Copyright 2015 Alexey Vaganov
 *
 * This file is part of mbwf.
 *
 * mbwf is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * mbwf is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mbwf.  If not, see <http://www.gnu.org/licenses/>.
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
		// check for new character ready
		if (cli_have_data) {
			cli_process();
		}
		// check for new modbus message
		if (modbus_have_data) {
			modbus_process();
		}
		// check for new wi-fi datagram
		if (wifi_have_data) {
			wifi_process();
		}
	}
}