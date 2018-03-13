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
	UART(uint8_t uart = 1);
	
	~UART();
	
	bool begin(int baud = 9600, bool nine_bit = false);
	void end();
	int available();
	int peek();
	size_t write(uint8_t data);
	inline size_t write(int data) { return write((uint8_t)data); }
	size_t write9bit(uint16_t data);
	
	int read();
	bool readUL(unsigned long *val);
	void flush();

	bool error();
	bool ninthBitSet();
	inline uint8_t getTXPin() { return m_TXn; }
	
	using Print::write; // pull in write(str) and write(buf, size) from Print
	
private:
	uint8_t m_TXn;
	uint8_t m_uart;
	
	uint8_t RXENn;
	uint8_t TXENn;
	uint8_t RXCIEn;
	uint8_t UCSZn0;
	uint8_t UCSZn1;
	uint8_t UCSZn2;
	uint8_t USBSn;
	uint8_t U2Xn;
	
	volatile uint8_t *v_UBRRnH;
	volatile uint8_t *v_UBRRnL;
	volatile uint8_t *v_UCSRnC;
};
