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
#include <stdlib.h>
#include <string.h>

enum {
	STATE_START,
	STATE_SSID,
	STATE_NEED_KEY,
	STATE_KEY,
	STATE_IP,
	STATE_MASK,
	STATE_GATEWAY,
	STATE_UDP_SRC,
	STATE_UDP_DST,
	STATE_RATE,
	STATE_NEED_PARITY,
	STATE_PARITY_EVEN,
	STATE_SAVE
};

uint8_t cli_state = STATE_START;
char last_char;
volatile bool cli_have_data = false;
config_t cli_temp_config;

#define CLI_MAX_BUFFER 63
char cli_buffer[CLI_MAX_BUFFER+1];
uint8_t cli_buffer_len;

void cli_uart_putc(char send)
{
	// data waiting in the hardware to be sent
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = send;
}

void cli_uart_puts(const char *send)
{
	// cycle through each character until \0
	while (*send) {
		cli_uart_putc(*send++);
	}
}

void cli_uart_putl(long val)
{
	char buf[12];
	cli_uart_puts(ltoa(val, buf, 10));
}

void cli_uart_putul(unsigned long val)
{
	char buf[12];
	cli_uart_puts(ultoa(val, buf, 10));
}

void cli_print_prompt()
{
	switch(cli_state) {
	case STATE_START:
		cli_uart_puts("\x1B[2J\r\n");
		cli_uart_puts("Press any key to continue...");
		break;
	case STATE_SSID:
		cli_uart_puts("\x1B[2J\r\n");
		cli_uart_puts("You are about to be asked to enter information to configure the device.\r\n");
		cli_uart_puts("You can leave fields blank to save last value.\r\n");
		cli_uart_puts("SSID [");
		cli_uart_puts(config.wifi_ssid);
		cli_uart_puts("]: ");
		break;
	case STATE_NEED_KEY:
		cli_uart_puts("\r\nNetwork have a passphrase? Yes/No [");
		if (strlen(config.wifi_key) > 0) {
			cli_uart_putc('Y');
		} else {
			cli_uart_putc('N');
		}
		cli_uart_puts("]: ");
		break;
	case STATE_KEY:
		cli_uart_puts("\r\nWireless passphrase [");
		cli_uart_puts(config.wifi_key);
		cli_uart_puts("]: ");
		break;
	case STATE_IP:
		cli_uart_puts("\r\nIP address [");
		cli_uart_puts(config.wifi_ip);
		cli_uart_puts("]: ");
		break;
	case STATE_MASK:
		cli_uart_puts("\r\nNetwork mask [");
		cli_uart_puts(config.wifi_mask);
		cli_uart_puts("]: ");
		break;
	case STATE_GATEWAY:
		cli_uart_puts("\r\nGateway [");
		cli_uart_puts(config.wifi_gateway);
		cli_uart_puts("]: ");
		break;
	case STATE_UDP_SRC:
		cli_uart_puts("\r\nUDP source port [");
		cli_uart_putul(config.wifi_udp_src);
		cli_uart_puts("]: ");
		break;
	case STATE_UDP_DST:
		cli_uart_puts("\r\nUDP destination port [");
		cli_uart_putul(config.wifi_udp_dst);
		cli_uart_puts("]: ");
		break;
	case STATE_RATE:
		cli_uart_puts("\r\nModbus baud rate [");
		cli_uart_putul(config.mb_baud_rate);
		cli_uart_puts("]: ");
		break;
	case STATE_NEED_PARITY:
		cli_uart_puts("\r\nParity bit. Yes/No [");
		if (config.mb_parity_bit) {
			cli_uart_putc('Y');
		} else {
			cli_uart_putc('N');
		}
		cli_uart_puts("]: ");
		break;
	case STATE_PARITY_EVEN:
		cli_uart_puts("\r\nParity is even? Yes/No [");
		if (config.mb_parity_even) {
			cli_uart_putc('Y');
		} else {
			cli_uart_putc('N');
		}
		cli_uart_puts("]: ");
		break;
	case STATE_SAVE:
		cli_uart_puts("\r\nWould you like to save changes? Yes/No: ");
		break;
	}
}

void cli_clear_buffer(void)
{
	memset(cli_buffer, 0, CLI_MAX_BUFFER+1);
	cli_buffer_len = 0;
}

void cli_process_buffer(void)
{
	unsigned long temp_long;
	switch(cli_state) {
	case STATE_SSID:
		if (cli_buffer_len > 32) return;
		if (cli_buffer_len == 0) {
			strcpy(cli_temp_config.wifi_ssid, config.wifi_ssid);
		} else {
			strcpy(cli_temp_config.wifi_ssid, cli_buffer);
		}
		cli_state = STATE_NEED_KEY;
		break;
	case STATE_NEED_KEY:
		if (cli_buffer_len == 0) {
			if (strlen(config.wifi_key) > 0) {
				cli_state = STATE_KEY;
			} else {
				cli_state = STATE_IP;
			}
		} else {
			if ((cli_buffer[0] == 'Y') || (cli_buffer[0] == 'y')) {
				cli_state = STATE_KEY;
			} else if ((cli_buffer[0] == 'N') || (cli_buffer[0] == 'n')) {
				cli_state = STATE_IP;
			}
		}
		break;
	case STATE_KEY:
		if (cli_buffer_len > 63) return;
		if (cli_buffer_len == 0) {
			strcpy(cli_temp_config.wifi_key, config.wifi_key);
		} else {
			strcpy(cli_temp_config.wifi_key, cli_buffer);
		}
		cli_state = STATE_IP;
		break;
	case STATE_IP:
		if (cli_buffer_len > 15) return;
		if (cli_buffer_len == 0) {
			strcpy(cli_temp_config.wifi_ip, config.wifi_ip);
		} else {
			strcpy(cli_temp_config.wifi_ip, cli_buffer);
		}
		cli_state = STATE_MASK;
		break;
	case STATE_MASK:
		if (cli_buffer_len > 15) return;
		if (cli_buffer_len == 0) {
			strcpy(cli_temp_config.wifi_mask, config.wifi_mask);
		} else {
			strcpy(cli_temp_config.wifi_mask, cli_buffer);
		}
		cli_state = STATE_GATEWAY;
		break;
	case STATE_GATEWAY:
		if (cli_buffer_len > 15) return;
		if (cli_buffer_len == 0) {
			strcpy(cli_temp_config.wifi_gateway, config.wifi_gateway);
		} else {
			strcpy(cli_temp_config.wifi_gateway, cli_buffer);
		}
		cli_state = STATE_UDP_SRC;
		break;
	case STATE_UDP_SRC:
		if (cli_buffer_len > 5) return;
		if (cli_buffer_len == 0) {
			cli_temp_config.wifi_udp_src = config.wifi_udp_src;
		} else {
			temp_long = atol(cli_buffer);
			if (temp_long == 0) return;
			if (temp_long > 65535) return;
			cli_temp_config.wifi_udp_src = temp_long;
		}
		cli_state = STATE_UDP_DST;
		break;
	case STATE_UDP_DST:
		if (cli_buffer_len > 5) return;
		if (cli_buffer_len == 0) {
			cli_temp_config.wifi_udp_dst = config.wifi_udp_dst;
		} else {
			temp_long = atol(cli_buffer);
			if (temp_long == 0) return;
			if ((temp_long > 65535) || (temp_long < 0)) return;
			cli_temp_config.wifi_udp_dst = temp_long;
		}
		cli_state = STATE_RATE;
		break;
	case STATE_RATE:
		if (cli_buffer_len > 11) return;
		if (cli_buffer_len == 0) {
			cli_temp_config.mb_baud_rate = config.mb_baud_rate;
		} else {
			temp_long = atol(cli_buffer);
			switch (temp_long) {
			case 2400:
			case 4800:
			case 9600:
			case 14400:
			case 19200:
			case 28800:
			case 38400:
			case 57600:
			case 76800:
			case 115200:
			case 230400:
			case 250000:
				break;
			default:
				return;
			}
			cli_temp_config.mb_baud_rate = temp_long;
		}
		cli_state = STATE_NEED_PARITY;
		break;
	case STATE_NEED_PARITY:
		if (cli_buffer_len == 0) {
			cli_temp_config.mb_parity_bit = config.mb_parity_bit;
		} else {
			if ((cli_buffer[0] == 'Y') || (cli_buffer[0] == 'y')) {
				cli_temp_config.mb_parity_bit = true;
			} else if ((cli_buffer[0] == 'N') || (cli_buffer[0] == 'n')) {
				cli_temp_config.mb_parity_bit = false;
			} else {
				return;
			}
		}
		if (cli_temp_config.mb_parity_bit) {
			cli_state = STATE_PARITY_EVEN;
		} else {
			cli_state = STATE_SAVE;
		}
		break;
	case STATE_PARITY_EVEN:
		if (cli_buffer_len == 0) {
			cli_temp_config.mb_parity_even = config.mb_parity_even;
		} else {
			if ((cli_buffer[0] == 'Y') || (cli_buffer[0] == 'y')) {
				cli_temp_config.mb_parity_even = true;
			} else if ((cli_buffer[0] == 'N') || (cli_buffer[0] == 'n')) {
				cli_temp_config.mb_parity_even = false;
			} else {
				return;
			}
		}
		cli_state = STATE_SAVE;
		break;
	case STATE_SAVE:
		if (cli_buffer_len == 0) return;
		if ((cli_buffer[0] == 'Y') || (cli_buffer[0] == 'y')) {
			config = cli_temp_config;
			write_settings();
			soft_reset();
		} else if ((cli_buffer[0] == 'N') || (cli_buffer[0] == 'n')) {
			cli_state = STATE_START;
		} else {
			return;
		}
		break;
	default:
		cli_state++;
		break;
	}
}

void cli_process(void)
{
	if (cli_state == STATE_START) {
		cli_clear_buffer();
		memset(&cli_temp_config, 0, sizeof(config_t));
		cli_state = STATE_SSID;
		cli_print_prompt();
	} else if (last_char == 0x1B) {
		cli_clear_buffer();
		cli_state = STATE_START;
		cli_print_prompt();
	} else if ((last_char == '\n') || (last_char == '\r')) {
		cli_process_buffer();
		cli_clear_buffer();
		cli_print_prompt();
	} else if ((last_char >= 0x20) && (last_char < 0x7F)) {
		if (cli_buffer_len < CLI_MAX_BUFFER) {
			cli_buffer[cli_buffer_len++] = last_char;
			cli_uart_putc(last_char);
		} else {
			cli_uart_putc(0x07);
		}
	}
	cli_have_data = false;
}

void cli_uart_init(void)
{
	// configure USART0
	#define BAUD 9600
	#include <util/setbaud.h>
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

	#if USE_2X
	UCSR0A |= _BV(U2X0);
	#else
	UCSR0A &= ~_BV(U2X0);
	#endif

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);  // 8-N-1
	// Enable RX, TX and RX Complete Interrupt
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);
	
	// print initial prompt
	cli_print_prompt();
}

ISR(USART0_RX_vect)
{
	char new_char;
	new_char = UDR0;
	if ((last_char == '\r') && (new_char == '\n')) {
		return;
	}
	last_char = new_char;
	cli_have_data = true;
}
