#include <Arduino.h>
#include "SoftwareSPISlave.h"

/**
 * Toggle MISO pin depending on tx_buf and call set_tx_buf() after 8bit transmission
 */
inline void tx_func();
/**
 * Store receiving data from MOSI pin and call get_rx_buf() after 8bit receiving
 */
inline void rx_func();
/**
 * Enqueue function for rx/tx_queue used in SPI function.
 * @param data  Data write to queue.
 * @param queue Buffer address write to.
 * @param ptr   Head pointer for queue writing.
 * @param num   Number of data stored in queue.
 */
inline void inline_enqueue(uint8_t data, uint8_t *queue, int16_t *ptr, int16_t *num);
/**
 * Denqueue function for rx/tx_queue useid in SPI function.
 * @param  queue Buffer address read from.
 * @param  ptr   Head pointer for queue reading.
 * @param  num   Number of data stored in queue.
 * @return       Oldest data in queue.
 */
inline uint8_t inline_dequeue(uint8_t queue[], int16_t *ptr, int16_t *num);
/**
 * Read data from queue but doesn't decrement stored data size counter
 * @param  queue Buffer address read from.
 * @param  ptr   Head pointer for queue reading.
 * @param  num   Number of data stored in queue.
 * @return       Oldest data in queue.
 */
inline uint8_t inline_peek(uint8_t queue[], int16_t *ptr, int16_t *num);

inline void tx_func() {
	if (SPISlave.tx_buf_cnt == 0) {
		// prepair for transmission
		// read from tx_queue but not dequeue
		SPISlave.tx_buf = inline_peek(SPISlave.tx_queue, &SPISlave.tx_ptr, &SPISlave.tx_num);
	}

	if (SPISlave.tx_buf & 0x80) {
		// PORTB |=  SPISlave.miso_mask; 
		*SPISlave.miso_port |= SPISlave.miso_mask;
	} else {
		// PORTB &= ~SPISlave.miso_mask;
		*SPISlave.miso_port &= ~SPISlave.miso_mask;
	}

	// shift bit or increment counter
	SPISlave.tx_buf_cnt ++;
	if (SPISlave.tx_buf_cnt < 8) {
		SPISlave.tx_buf <<= 1;
	} else {
		// reset counter
		SPISlave.tx_buf_cnt = 0;
		// dequeue transmitted data
		inline_dequeue(SPISlave.tx_queue, &SPISlave.tx_ptr, &SPISlave.tx_num);
	}
}

inline void rx_func() {
	// if (PINB & SPISlave.mosi_mask) {
	if (*SPISlave.mosi_port & SPISlave.mosi_mask) {
		SPISlave.rx_buf |= 0x01;
	}

	SPISlave.rx_buf_cnt ++;
	if (SPISlave.rx_buf_cnt < 8) {
		SPISlave.rx_buf <<= 1 ;
	} else {
		// store data
		inline_enqueue(SPISlave.rx_buf, SPISlave.rx_queue, &SPISlave.rx_ptr, &SPISlave.rx_num);
		// reset counter
		SPISlave.rx_buf_cnt = 0;
		// reset rx buffer
		SPISlave.rx_buf = 0;
	}
}

inline void inline_enqueue(uint8_t data, uint8_t *queue, int16_t *ptr, int16_t *num) {
	(*ptr) ++;
	(*num) ++;

	if (*ptr >= QUEUE_SIZE) *ptr = 0;	
	if (*num > QUEUE_SIZE)  *num = QUEUE_SIZE;

	queue[*ptr] = data;
}

inline uint8_t inline_dequeue(uint8_t queue[], int16_t *ptr, int16_t *num) {
	uint8_t ret;

	if (*num <= 0) {
		ret = 0x00;
	} else if (*ptr - *num + 1 >= 0) {
		ret = queue[*ptr - *num + 1];
		(*num) --;
	} else {
		ret = queue[QUEUE_SIZE + (*ptr - *num + 1)];
		(*num) --;
	}

	return ret;
}

inline uint8_t inline_peek(uint8_t queue[], int16_t *ptr, int16_t *num) {
	uint8_t ret;

	if (*num <= 0) {
		ret = 0x00;
	} else if (*ptr - *num + 1 >= 0) {
		ret = queue[*ptr - *num + 1];
	} else {
		ret = queue[QUEUE_SIZE + (*ptr - *num + 1)];
	}

	return ret;
}

/**
 * PCINT interrupt fucntion.
 * Called when SS/SCK pin toggled.
 */
ISR(PCINT0_vect) {
	// determin change pin
	uint8_t pcint_diff;
	pcint_diff = (PINB & SPISlave.pcint_mask) ^ SPISlave.pcint_prev;

	if (pcint_diff & SPISlave.sck_mask) {
		// triggered bt sck pin
		if (PINB & SPISlave.sck_mask) {
			// raising change interrupt
			rx_func();
		} else {
			// falling change interrupt
			// transmisson data
			tx_func();
		}
		// save status
		SPISlave.pcint_prev ^= SPISlave.sck_mask;
	} else 	if (pcint_diff & SPISlave.ss_mask) {
		// triggered by ss pin
		if (PINB & SPISlave.ss_mask) {
			// end receiving
		} else {
			// start receiving/transmitting
			// reset counter
			SPISlave.rx_buf_cnt = 0;
			SPISlave.tx_buf_cnt = 0;
			// reset rx buffer
			SPISlave.rx_buf = 0x00;
			// transmisson data
			tx_func();
		}
		// save status
		SPISlave.pcint_prev ^= SPISlave.ss_mask;
	}
}


SoftwareSPISlave::SoftwareSPISlave() {
	// reset pointer/counter for queue
	rx_ptr = 0;
	tx_ptr = 0;
	rx_num = 0;
	tx_num = 0;
}

void SoftwareSPISlave::begin(uint8_t MISO_pin, uint8_t MOSI_pin, uint8_t SCK_pin, uint8_t SS_pin) {
	// miso_mask  = pin_to_mask(MISO_pin);
	// mosi_mask  = pin_to_mask(MOSI_pin);
	miso_mask  = digitalPinToBitMask(MISO_pin);
	mosi_mask  = digitalPinToBitMask(MOSI_pin);
	// sck_mask   = pin_to_mask(SCK_pin);
	// ss_mask    = pin_to_mask(SS_pin);
	sck_mask   = digitalPinToBitMask(SCK_pin);
	ss_mask    = digitalPinToBitMask(SS_pin);
	pcint_mask = (sck_mask | ss_mask);

	mosi_port  = portInputRegister(digitalPinToPort(MOSI_pin));
	miso_port  = portOutputRegister(digitalPinToPort(MISO_pin));

	// input MOSI, SCK, SS
	pinMode(MOSI_pin, INPUT);
	pinMode(SCK_pin, INPUT);
	pinMode(SS_pin, INPUT);
	// output MISO
	pinMode(MISO_pin, OUTPUT);

	// enable pcint interrupt
	PCICR |= _BV(PCIE0);
	// set pcint interrupt pin (SCK, SS)
	PCMSK0 |= pcint_mask;

	// store current state of PCINT pins (SS/SCK).
	pcint_prev = PINB & pcint_mask;

	// enable all interrupts
	sei();
}

void SoftwareSPISlave::write(uint8_t *data, int16_t len) {
	for (int16_t i=0; i<len; i++) {
		inline_enqueue(data[i], tx_queue, &tx_ptr, &tx_num);
	}
}

void SoftwareSPISlave::read(uint8_t *data, int16_t len) {
	for (int16_t i=0; i<len; i++) {
		data[i] = inline_dequeue(rx_queue, &rx_ptr, &rx_num);
	}
}

uint8_t SoftwareSPISlave::peek(int16_t ptr) {
	uint8_t ret;
	int16_t ptr_buf;

	ptr_buf = rx_ptr - rx_num + 1 + ptr;

	if (rx_num <= ptr) {
		ret = 0x00;
	} else if (ptr_buf >= 0) {
		ret = rx_queue[ptr_buf];
	} else {
		ret = rx_queue[QUEUE_SIZE + (ptr_buf)];
	}

	return ret;
}

SoftwareSPISlave SPISlave;
