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

#ifndef COMMON_H_
#define COMMON_H_

#include <stdbool.h>
#include <avr/io.h>
#include <avr/wdt.h>

#define soft_reset()        \
do                          \
{                           \
	wdt_enable(WDTO_15MS);  \
	for(;;)                 \
	{                       \
	}                       \
} while(0)

#define F_CPU 3686400

typedef struct config_t {
	char wifi_ssid[33];
	char wifi_key[64];
	char wifi_ip[16];
	char wifi_mask[16];
	char wifi_gateway[16];
	uint16_t wifi_udp_src;
	uint16_t wifi_udp_dst;
	uint32_t mb_baud_rate;
	bool mb_parity_bit;
	bool mb_parity_even;
} config_t;

extern volatile bool cli_have_data;
void cli_uart_init(void);
void cli_process(void);
void cli_uart_puts(const char *send);
void cli_uart_putl(long val);

extern struct config_t config;
void read_settings(void);
void write_settings(void);

extern volatile bool modbus_have_data;
void modbus_uart_init(void);
void modbus_process(void);
void modbus_write(const uint8_t *send, uint16_t count);

extern volatile bool wifi_have_data;
void wifi_uart_init(void);
void wifi_send(const uint8_t *send, uint16_t count);
void wifi_process(void);

#endif /* COMMON_H_ */