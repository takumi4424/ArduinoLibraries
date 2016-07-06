/*
 * Software SPI slave library.
 */
#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>

/**
 * Size of rx/tx queue
 */
#define QUEUE_SIZE 128

class SoftwareSPISlave {
public:
	/**
	 * Save previous PCINT status
	 */
	uint8_t pcint_prev;
	/**
	 * Bit mask relative to PCINT
	 */
	uint8_t pcint_mask;

	/**
	 * Bit mask for each SPI pin
	 */
	uint8_t ss_mask;
	uint8_t sck_mask;
	uint8_t mosi_mask;
	uint8_t miso_mask;
	volatile uint8_t *mosi_port;
	volatile uint8_t *miso_port;



	uint8_t rx_buf;
	uint8_t rx_buf_cnt;

	uint8_t tx_buf;
	uint8_t tx_buf_cnt;

	uint8_t rx_queue[QUEUE_SIZE];
	int16_t rx_ptr;
	int16_t rx_num;
	
	uint8_t tx_queue[QUEUE_SIZE];
	int16_t tx_ptr;
	int16_t tx_num;

	SoftwareSPISlave();
	/**
	 * Begin spi communication with specified pins.
	 * SCK and SS pins must be a PCINT pin.
	 * @param MISO_pin Arduino pin number used for SPI MISO
	 * @param MOSI_pin Arduino pin number used for SPI MOSI
	 * @param SCK_pin  Arduino pin number used for SPI SCK (must be a PCINT pin)
	 * @param SS_pin   Arduino pin number used for SPI SS (must be a PCINT pin)
	 */
	void begin(uint8_t MISO_pin, uint8_t MOSI_pin, uint8_t SCK_pin, uint8_t SS_pin);
	void write(uint8_t *data, int16_t len);
	void write(uint8_t data) { write(&data, 1); }
	void read(uint8_t *data, int16_t len);
	uint8_t read() { uint8_t ret; read(&ret, 1); return ret; }
	uint8_t peek(int16_t ptr);
	uint8_t peek() { peek(0); }
	int16_t available() { return rx_num; }
};

extern SoftwareSPISlave SPISlave;
