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
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>

volatile bool wifi_have_data;
#define WIFI_MAX_BUFFER 1000
uint8_t wifi_rx_buffer[WIFI_MAX_BUFFER+1];
uint16_t wifi_rx_buffer_len;
uint16_t wifi_rx_expect;
bool wifi_rx_start = false;
uint8_t wifi_tx_buffer[WIFI_MAX_BUFFER+1];
uint16_t wifi_tx_buffer_len;
uint16_t wifi_tx_buffer_cnt;

void wifi_puts(const char *send)
{
	while (*send) {
		loop_until_bit_is_set(UCSR2A, UDRE2);
		UDR2 = *send++;
	}
}

void wifi_putl(long val)
{
	char buf[12];
	wifi_puts(ltoa(val, buf, 10));
}

void wifi_send(const uint8_t *send, uint16_t count)
{
	if (count > WIFI_MAX_BUFFER - 17) return;
	char buf[18] = "AT+CIPSEND=";
	// put count str in the end if buf (length 11)
	itoa(count, &buf[11], 10);
	// and add <CR><LF>
	strcat(buf, "\r\n");
	size_t hlen = strlen(buf);
	memcpy(wifi_tx_buffer, buf, hlen);
	// put to the end of buffer data to send
	memcpy(&wifi_tx_buffer[hlen], send, count);
	wifi_tx_buffer_len = hlen + count;
	// send first byte
	wifi_tx_buffer_cnt = 1;
	loop_until_bit_is_set(UCSR2A, UDRE2);
	UDR2 = wifi_tx_buffer[0];
}

void wifi_process(void)
{
	modbus_write(wifi_rx_buffer, wifi_rx_buffer_len);
	wifi_rx_buffer_len = 0;
	wifi_have_data = false;
}

void wifi_module_init(void)
{
	wifi_puts("AT+CWMODE=\"Station\"\r\n");
	_delay_ms(500);
	wifi_puts("AT+RST\r\n");
	_delay_ms(1000);
	wifi_puts("AT+CWJAP=\"");
	wifi_puts(config.wifi_ssid);
	wifi_puts("\",\"");
	wifi_puts(config.wifi_key);
	wifi_puts("\"\r\n");
	_delay_ms(1000);
	wifi_puts("AT+CIPSTA=\"");
	wifi_puts(config.wifi_ip);
	wifi_puts("\",\"");
	wifi_puts(config.wifi_mask);
	wifi_puts("\",\"");
	wifi_puts(config.wifi_gateway);
	wifi_puts("\"\r\n");
	_delay_ms(500);
	wifi_puts("AT+CIPMUX=0\r\n");
	_delay_ms(500);
	wifi_puts("AT+CIPSTART=\"UDP\",\"0.0.0.0\",\"");
	wifi_putl(config.wifi_udp_dst);
	wifi_puts("\",\"");
	wifi_putl(config.wifi_udp_src);
	wifi_puts(",0\r\n");
	_delay_ms(500);
}

void wifi_uart_init(void)
{
	// configure USART2
	#define BAUD 115200
	#include <util/setbaud.h>
	UBRR2H = UBRRH_VALUE;
	UBRR2L = UBRRL_VALUE;

	#if USE_2X
	UCSR2A |= _BV(U2X2);
	#else
	UCSR2A &= ~_BV(U2X2);
	#endif

	UCSR2C = _BV(UCSZ21) | _BV(UCSZ20); /* 8-N-1 */
	// Enable USART2 RX, TX and RX/TX Complete Interrupt
	UCSR2B = _BV(RXEN2) | _BV(TXEN2) | _BV(RXCIE2) | _BV(TXCIE2);
	
	wifi_module_init();
}

ISR(USART2_RX_vect)
{
	uint8_t new_byte;
	new_byte = UDR2;
	// +IPD,<len>:<data>
	if (!wifi_rx_start && (new_byte == '+')) {
		wifi_rx_buffer_len = 0;
		wifi_rx_expect = 0;
		wifi_rx_start = true;
	}
	if (wifi_rx_start && (wifi_rx_buffer_len < WIFI_MAX_BUFFER)) {
		wifi_rx_buffer[wifi_rx_buffer_len++] = new_byte;
		if (wifi_rx_expect) {
			wifi_rx_expect--;
			if (wifi_rx_expect == 0) {
				wifi_have_data = true;
				wifi_rx_start = false;
			}
		} else if (wifi_rx_buffer_len == 5) {
			if (memcmp("+IPD,", wifi_rx_buffer, 5) != 0) {
				wifi_rx_buffer_len = 0;
				wifi_rx_start = false;
			}
		} else if (wifi_rx_buffer_len > 5) {
			if (new_byte == ':') {
				char buf[5];
				memset(buf, 0, 5);
				memcpy(buf, &wifi_rx_buffer[5], wifi_rx_buffer_len-6);
				wifi_rx_expect = atol(buf);
				wifi_rx_buffer_len = 0;
			}
		}
	}
}

ISR(USART2_TX_vect)
{
	if (wifi_tx_buffer_cnt < wifi_tx_buffer_len) {
		// send next byte in buffer
		UDR2 = wifi_tx_buffer[wifi_tx_buffer_cnt++];
	}
}
