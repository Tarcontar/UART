#pragma once
#include <Arduino.h>
#include "Stream.h"

//registers
#define TXB8	0
#define UPE		2
#define DOR		3
#define FE		4
#define UDRE	5
#define RXC		7

#define UART_BUFFER_SIZE 128

class UART : public Stream
{
public:
	static UART* UART8Bit(uint8_t uart);
	static UART* UART9Bit(uint8_t uart);
	
	~UART();
	
	void begin(int baud = 9600);
	void end();
	int available();
	int peek();
	size_t write(uint8_t data);
	inline size_t write(int data) { return write((uint8_t)data); }
	size_t write9bit(uint16_t data);
	
	int read();
	void flush();

	bool error();
	bool ninthBitSet();
	inline uint8_t getTXPin() { return m_TXn; }
	
	using Print::write; // pull in write(str) and write(buf, size) from Print
	
private:
	UART(uint8_t uart = 1, bool nine_bit = false);

	uint8_t m_TXn;
};
