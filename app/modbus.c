/*
 * modbus.c
 *
 *  Author: Alexey Vaganov
 */ 
#include "common.h"
#include <string.h>
#include <avr/interrupt.h>

volatile bool modbus_have_data;

#define MODBUS_WAIT_NO 0
#define MODBUS_WAIT_35 1
#define MODBUS_WAIT_15_35 1
bool modbus_wait = MODBUS_WAIT_NO;
#define MODBUS_MAX_BUFFER 256
uint8_t modbus_rx_buffer[MODBUS_MAX_BUFFER];
uint16_t modbus_rx_buffer_len;
uint8_t modbus_tx_buffer[MODBUS_MAX_BUFFER];
uint16_t modbus_tx_buffer_len;
uint16_t modbus_tx_buffer_cnt;
uint8_t modbus_TCCR0B;

#define TIMER1_OCR ((F_CPU) / 10240)

void flash_rx_led(void)
{
	PORTE |= _BV(PE3); // turn on RX LED
	TCNT1 = 0x00; // zero timer
	TCCR1B = _BV(CS12) | _BV(CS10); // start timer with CLK/1024
}

void flash_tx_led(void)
{
	PORTE |= _BV(PE4); // turn on TX LED
	TCNT1 = 0x00; // zero timer
	TCCR1B = _BV(CS12) | _BV(CS10); // start timer with CLK/1024
}

void modbus_start_tx(void)
{
	PORTE |= _BV(PE2);  // set DE on RS485
	loop_until_bit_is_set(UCSR0A, UDRE0);
	// send first byte in buffer
	UDR0 = modbus_tx_buffer[modbus_tx_buffer_cnt++];
	flash_tx_led();
}

void modbus_write(const uint8_t *send, uint16_t count)
{
	// limit bytes count to buffer size
	if (count > MODBUS_MAX_BUFFER) return;
	memcpy(modbus_tx_buffer, send, count);
	modbus_tx_buffer_len = count;
	modbus_tx_buffer_cnt = 0;
	// start tx or send later
	if (modbus_wait == MODBUS_WAIT_NO) {
		modbus_start_tx();
	}
}

void modbus_timer_init(void)
{
	// configure 8-bit timer Timer0 for protocol purposes
	uint16_t clk_div[5] = {1, 8, 64, 256, 1024};
	uint8_t div_TCCR0B[5] = {_BV(CS00), _BV(CS01), _BV(CS00) | _BV(CS01), _BV(CS02), _BV(CS02) | _BV(CS00)};
	uint32_t temp_ocr;
	// we need find minimal divisor for our F_CPU and baud rate
	for (int i = 0; i < 5; i++) {
		temp_ocr = (11 * F_CPU * 35) / (10 * clk_div[i] * config.mb_baud_rate);
		if (temp_ocr <= 0xFF) {
			OCR0A = (11 * F_CPU * 15) / (10 * clk_div[i] * config.mb_baud_rate);
			OCR0B = temp_ocr;
			modbus_TCCR0B = div_TCCR0B[i];
			break;
		}
	}
	TIMSK0 = _BV(OCIE0A) | _BV(OCIE0B); // turn on interrupt
	TCNT0 = 0x00; // zero timer
	TCCR0B = modbus_TCCR0B; // start timer
	modbus_wait = MODBUS_WAIT_35; // block input on start
	
	// configure 16-bit timer Timer1 for LED
	TIMSK1 = _BV(OCIE1A); // turn on interrupt
	OCR1A = TIMER1_OCR; // 0.1 ms
}

void modbus_uart_init(void)
{
	// Enable PE2 output for RE/DE
	DDRE |= _BV(DDE2);
	// Enable PE3 output for RX LED
	DDRE |= _BV(DDE3);
	// Enable PE4 output for TX LED
	DDRE |= _BV(DDE4);
	
	// configure USART0
	uint16_t ubrr;
	bool use2x;
	ubrr = (F_CPU + 8 * config.mb_baud_rate) / (16 * config.mb_baud_rate) - 1;
	use2x = 100 * F_CPU > (16 * (ubrr + 1)) * (100 * config.mb_baud_rate + 
												config.mb_baud_rate * 2);
	use2x |= 100 * (F_CPU) < (16 * (ubrr + 1)) * (100 * config.mb_baud_rate - 
												config.mb_baud_rate * 2);
	if (use2x) {
		ubrr = (F_CPU + 4 * config.mb_baud_rate) / (8 * config.mb_baud_rate) - 1;
		UCSR0A |= _BV(U2X0);
	} else {
		UCSR0A &= ~_BV(U2X0);
	}
	UBRR0H = ubrr >> 8;
	UBRR0L = ubrr & 0xff;

	if (config.mb_parity_bit) {
		if (config.mb_parity_even) {
			UCSR0C = _BV(UCSZ01) | _BV(UCSZ00) | _BV(UPM01); /* 8-E-1 */
		} else {
			UCSR0C = _BV(UCSZ01) | _BV(UCSZ00) | _BV(UPM00) | _BV(UPM01); /* 8-O-1 */
		}
	} else {
		UCSR0C = _BV(UCSZ01) | _BV(UCSZ00) | _BV(USBS0); /* 8-N-2 */
	}
	// Enable RX, TX and RX/TX Complete Interrupt
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0) | _BV(TXCIE0);

	modbus_timer_init();
}

void modbus_process(void)
{
	wifi_send(modbus_rx_buffer, modbus_rx_buffer_len);
	modbus_rx_buffer_len = 0;
	modbus_have_data = false;
}

ISR(USART0_RX_vect)
{
	if (!modbus_have_data && (modbus_rx_buffer_len < MODBUS_MAX_BUFFER)) {
		modbus_rx_buffer[modbus_rx_buffer_len++] = UDR0;
		modbus_wait = MODBUS_WAIT_15_35;
		TCNT0 = 0x00; // zero timer
		TCCR0B = modbus_TCCR0B; // start timer
	} else {
		// we need to flush uart rx buffer anyway
		uint8_t dummy;
		dummy = UDR0;
		(void)dummy;
	}
	flash_rx_led();
}

ISR(USART0_TX_vect)
{
	if (modbus_tx_buffer_cnt < modbus_tx_buffer_len) {
		// send next byte in buffer
		UDR0 = modbus_tx_buffer[modbus_tx_buffer_cnt++];
		flash_tx_led();
	} else {
		PORTE &= ~_BV(PE2);  // unset DE on RS485
		// block I/O for 3.5 chars
		modbus_wait = MODBUS_WAIT_15_35;
		TCNT0 = 0x00; // zero timer
		TCCR0B = modbus_TCCR0B; // start timer
	}
}

ISR(TIMER0_COMPA_vect)
{
	// block new input
	modbus_wait = MODBUS_WAIT_35;
	if (modbus_rx_buffer_len > 0) {
		modbus_have_data = true;
	}
}

ISR(TIMER0_COMPB_vect)
{	
	// stop timer
	TCCR0B = 0x00;
	// remove I/O lock
	modbus_wait = MODBUS_WAIT_NO;
	
	// check for delayed send
	if ((modbus_tx_buffer_cnt == 0) && (modbus_tx_buffer_len > 0)) {
		modbus_start_tx();
	}
}

ISR(TIMER1_COMPA_vect)
{
	// turn off both LEDs
	PORTE &= ~(_BV(PE3) | _BV(PE4));
	// stop timer Timer1
	TCCR0B = 0x00;
}
