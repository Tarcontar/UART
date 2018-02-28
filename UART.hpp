#pragma once
#include <Arduino.h>

//registers
#define TXB8	0
#define UPE	2
#define DOR	3
#define FE		4
#define UDRE	5
#define RXC	7

struct data
{
	char value;
	char mode;
};

class UART
{
public:
	static UART* UART8Bit(int uart);
	static UART* UART9Bit(int uart);
	
	void begin(int baud);
	bool available();
	int print(String str);
	void write(uint8_t data, int mode = 0);
	int read();
	
private:
	UART(int uart = 1, bool nine_bit = false);

	int m_TXn;
	
	volatile unsigned char  m_RXENn;
	volatile unsigned char  m_TXENn;
	volatile unsigned char  m_U2Xn;
	volatile unsigned char  m_UCSZn0;
	volatile unsigned char  m_UCSZn1;
	volatile unsigned char  m_UCSZn2;
};