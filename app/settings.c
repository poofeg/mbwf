/*
 * settings.c
 *
 *  Author: Alexey Vaganov
 */ 
#include "common.h"
#include <avr/eeprom.h>
#include <string.h>

uint8_t EEMEM ee_wifi_ssid[32];
uint8_t EEMEM ee_wifi_key[63];
uint8_t EEMEM ee_wifi_ip[15];
uint8_t EEMEM ee_wifi_mask[15];
uint8_t EEMEM ee_wifi_gateway[15];
uint16_t EEMEM ee_wifi_udp_src;
uint16_t EEMEM ee_wifi_udp_dst;
uint32_t EEMEM ee_mb_baud_rate;
uint8_t EEMEM ee_mb_parity_bit;
uint8_t EEMEM ee_mb_parity_even;
uint16_t EEMEM ee_magic_bytes;

config_t config;

void write_settings(void)
{
	eeprom_update_word(&ee_magic_bytes, 0x46F4);
	eeprom_update_block(config.wifi_ssid, ee_wifi_ssid, 32);
	eeprom_update_block(config.wifi_key, ee_wifi_key, 63);
	eeprom_update_block(config.wifi_ip, ee_wifi_ip, 15);
	eeprom_update_block(config.wifi_mask, ee_wifi_mask, 15);
	eeprom_update_block(config.wifi_gateway, ee_wifi_gateway, 15);
	eeprom_update_word(&ee_wifi_udp_src, config.wifi_udp_src);
	eeprom_update_word(&ee_wifi_udp_dst, config.wifi_udp_dst);
	eeprom_update_dword(&ee_mb_baud_rate, config.mb_baud_rate);
	eeprom_update_byte(&ee_mb_parity_bit, config.mb_parity_bit);
	eeprom_update_byte(&ee_mb_parity_even, config.mb_parity_even);
}

void read_settings(void)
{
	if (eeprom_read_word(&ee_magic_bytes) == 0x46F4) {
		eeprom_read_block(config.wifi_ssid, ee_wifi_ssid, 32);
		eeprom_read_block(config.wifi_key, ee_wifi_key, 63);
		eeprom_read_block(config.wifi_ip, ee_wifi_ip, 15);
		eeprom_read_block(config.wifi_mask, ee_wifi_mask, 15);
		eeprom_read_block(config.wifi_gateway, ee_wifi_gateway, 15);
		config.wifi_udp_src = eeprom_read_word(&ee_wifi_udp_src);
		config.wifi_udp_dst = eeprom_read_word(&ee_wifi_udp_dst);
		config.mb_baud_rate = eeprom_read_dword(&ee_mb_baud_rate);
		config.mb_parity_bit = eeprom_read_byte(&ee_mb_parity_bit);
		config.mb_parity_even = eeprom_read_byte(&ee_mb_parity_even);
	} else {
		strcpy(config.wifi_ssid, "Default");
		strcpy(config.wifi_key, "");
		strcpy(config.wifi_ip, "192.168.1.100");
		strcpy(config.wifi_mask, "255.255.255.0");
		strcpy(config.wifi_gateway, "192.168.1.1");
		config.wifi_udp_src = 502;
		config.wifi_udp_dst = 501;
		config.mb_baud_rate = 9600;
		config.mb_parity_bit = true;
		config.mb_parity_even = true;
		write_settings();
	}
}
