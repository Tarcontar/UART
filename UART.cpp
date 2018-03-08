#include "UART.hpp"
#include <Arduino.h>
#include <avr/interrupt.h>

#define BUFFER_SIZE 128

volatile uint16_t v_buffer[BUFFER_SIZE];
volatile uint16_t v_start = 0;
volatile uint16_t v_end = 0;
volatile bool v_error = false;
volatile bool v_ninthBitSet = false;
volatile uint8_t v_uart = 0;

volatile uint8_t *v_UDRn;
volatile uint8_t *v_UBRRnH;
volatile uint8_t *v_UBRRnL;
volatile uint8_t *v_UCSRnA;
volatile uint8_t *v_UCSRnB;
volatile uint8_t *v_UCSRnC;

uint8_t RXENn;
uint8_t TXENn;
uint8_t RXCIEn;
uint8_t UCSZn0;
uint8_t UCSZn1;
uint8_t UCSZn2;
uint8_t USBSn;
uint8_t U2Xn;


UART* UART::UART8Bit(uint8_t uart)
{
	return new UART(uart);
}

UART* UART::UART9Bit(uint8_t uart)
{
	return new UART(uart, true);
}

UART::UART(uint8_t uart, bool nine_bit)
{
	v_uart = uart % 4;
	if (uart == 0)
	{
		m_TXn = 1;
		RXENn = RXEN0;
		TXENn = TXEN0;
		UCSZn0 = UCSZ00;
		UCSZn1 = UCSZ01;
		UCSZn2 = UCSZ02;
		USBSn = USBS0;
		U2Xn = U2X0;
		RXCIEn = RXCIE0;
		
		v_UDRn = &UDR0;
		v_UBRRnH = &UBRR0H;
		v_UBRRnL = &UBRR0L;
		v_UCSRnA = &UCSR0A;
		v_UCSRnB = &UCSR0B;
		v_UCSRnC = &UCSR0C;
	}
	else if (uart == 2)
	{
		m_TXn = 16;
		RXENn = RXEN2;
		TXENn = TXEN2;
		UCSZn0 = UCSZ20;
		UCSZn1 = UCSZ21;
		UCSZn2 = UCSZ22;
		USBSn = USBS2;
		U2Xn = U2X2;
		RXCIEn = RXCIE2;
		
		v_UDRn = &UDR2;
		v_UBRRnH = &UBRR2H;
		v_UBRRnL = &UBRR2L;
		v_UCSRnA = &UCSR2A;
		v_UCSRnB = &UCSR2B;
		v_UCSRnC = &UCSR2C;
	}
	else if (uart == 3)
	{
		m_TXn = 14;
		RXENn = RXEN3;
		TXENn = TXEN3;
		UCSZn0 = UCSZ30;
		UCSZn1 = UCSZ31;
		UCSZn2 = UCSZ32;
		USBSn = USBS3;
		U2Xn = U2X3;
		RXCIEn = RXCIE3;
		
		v_UDRn = &UDR3;
		v_UBRRnH = &UBRR3H;
		v_UBRRnL = &UBRR3L;
		v_UCSRnA = &UCSR3A;
		v_UCSRnB = &UCSR3B;
		v_UCSRnC = &UCSR3C;
	}
	else
	{
		m_TXn = 18;
		RXENn = RXEN1;
		TXENn = TXEN1;
		UCSZn0 = UCSZ10;
		UCSZn1 = UCSZ11;
		UCSZn2 = UCSZ12;
		USBSn = USBS1;
		U2Xn = U2X1;
		RXCIEn = RXCIE1;
		
		v_UDRn = &UDR1;
		v_UBRRnH = &UBRR1H;
		v_UBRRnL = &UBRR1L;
		v_UCSRnA = &UCSR1A;
		v_UCSRnB = &UCSR1B;
		v_UCSRnC = &UCSR1C;
	}
	//set also UDRIE0 ?? 
	*v_UCSRnB |= (1 << RXENn) | (1 << TXENn) | (1 << RXCIEn);
	*v_UCSRnC |= (1 << UCSZn1) | (1 << UCSZn0); //8 bit mode
	if (nine_bit)
		*v_UCSRnB |= (1 << UCSZn2); 
	*v_UCSRnC |= (0 << USBSn); //one stop bit else 1
	*v_UCSRnC |= 0b00000000; //no parity bit
}

UART::~UART()
{
	end();
}

void UART::begin(int baud)
{
	uint16_t baud_setting = (F_CPU / 8 / baud - 1) / 2;
	*v_UBRRnH = baud_setting >> 8;
	*v_UBRRnL = baud_setting;
	*v_UCSRnA &= ~(1 << U2Xn); //disable rate doubler
}

void UART::end()
{
	flush();
	*v_UCSRnB |= (0 << RXENn) | (0 << TXENn) | (0 << RXCIEn);
}

int UART::available()
{
	return ((int)(BUFFER_SIZE + v_end - v_start)) % BUFFER_SIZE;
}

size_t UART::write(uint8_t data)
{
	v_ninthBitSet = false;
	while (!(*v_UCSRnA & (1 << UDRE))) {}
	*v_UDRn = data;
}

size_t UART::write9bit(uint16_t data)
{
	while (!(*v_UCSRnA & (1 << UDRE))) {}
	if (data & 0x100)
		*v_UCSRnB |= (1 << TXB8);
	else
		*v_UCSRnB &= ~(1 << TXB8);
	write((uint8_t)data);
}

int UART::read()
{
	if (v_start == v_end) {
		return -1;
	} else {
		unsigned char c = v_buffer[v_start];
		v_start = (v_start + 1) % BUFFER_SIZE;
		return c;
	}
}

int UART::peek()
{
	if (v_start == v_end) {
		return -1;
	} else {
		v_buffer[v_start];
	}
}

void UART::flush()
{
	v_start = 0;
	v_end = 0;
}

bool UART::error()
{
	return v_error;
}

bool UART::ninthBitSet()
{
	return v_ninthBitSet;
}

ISR(USART0_RX_vect)
{
	if (v_uart == 0)
	{
		char status = *v_UCSRnA;
		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		uint16_t result = ((*v_UCSRnB >> 1) & 0x01) << 8;
		if (result & 0x100)
			v_ninthBitSet = true;
		result |= *v_UDRn;
		v_buffer[v_end] = result;
		v_end = (v_end + 1) % BUFFER_SIZE;
		if (v_end == v_start)
		{
			v_start = (v_start + 1) % BUFFER_SIZE;
		}
	}
}

ISR(USART1_RX_vect)
{
	if (v_uart == 1)
	{
		/*
		char status = *v_UCSRnA;
		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		v_buffer[v_end] = (*v_UCSRnB >> 1) & 0x01;
		v_buffer[v_end++] = *v_UDRn;
		*/
	}
}

ISR(USART2_RX_vect)
{
	if (v_uart == 2)
	{
		/*
		char status = *v_UCSRnA;
		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		v_buffer[v_end] = (*v_UCSRnB >> 1) & 0x01;
		v_buffer[v_end++] = *v_UDRn;
		*/
	}
}

ISR(USART3_RX_vect)
{
	if (v_uart == 3)
	{
		/*
		char status = *v_UCSRnA;
		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		v_buffer[v_end] = (*v_UCSRnB >> 1) & 0x01;
		v_buffer[v_end++] = *v_UDRn;
		*/
	}
}