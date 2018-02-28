#include "UART.hpp"
#include <Arduino.h>
#include <avr/interrupt.h>

#define BUFFER_SIZE 128

volatile data v_buffer[BUFFER_SIZE];
volatile int v_pos = 0;
volatile int v_end = 0;
volatile bool v_error = false;
volatile bool v_msg_complete = false;
volatile int v_uart = 0;

volatile unsigned char *m_UDRn;
volatile unsigned char *m_UBRRnH;
volatile unsigned char *m_UBRRnL;
volatile unsigned char *m_UCSRnA;
volatile unsigned char *m_UCSRnB;
volatile unsigned char *m_UCSRnC;

UART* UART::UART8Bit(int uart)
{
	return new UART(uart);
}

UART* UART::UART9Bit(int uart)
{
	return new UART(uart, true);
}

UART::UART(int uart, bool nine_bit)
{
	v_uart = uart;
	if (uart == 2)
	{
		m_TXn = 16;
		m_UDRn = &UDR2;
		m_RXENn = RXEN2;
		m_TXENn = TXEN2;
		m_UBRRnH = &UBRR2H;
		m_UBRRnL = &UBRR2L;
		m_UCSRnA = &UCSR2A;
		m_UCSRnB = &UCSR2B;
		m_UCSRnC = &UCSR2C;
		m_UCSZn0 = UCSZ20;
		m_UCSZn1 = UCSZ21;
		m_UCSZn2 = UCSZ22;
		m_U2Xn = U2X2;
	}
	else if (uart == 3)
	{
		m_TXn = 14;
		m_UDRn = &UDR3;
		m_RXENn = RXEN3;
		m_TXENn = TXEN3;
		m_UBRRnH = &UBRR3H;
		m_UBRRnL = &UBRR3L;
		m_UCSRnA = &UCSR3A;
		m_UCSRnB = &UCSR3B;
		m_UCSRnC = &UCSR3C;
		m_UCSZn0 = UCSZ30;
		m_UCSZn1 = UCSZ31;
		m_UCSZn2 = UCSZ32;
		m_U2Xn = U2X3;
	}
	else
	{
		m_TXn = 18;
		m_UDRn = &UDR1;
		m_RXENn = RXEN1;
		m_TXENn = TXEN1;
		m_UBRRnH = &UBRR1H;
		m_UBRRnL = &UBRR1L;
		m_UCSRnA = &UCSR1A;
		m_UCSRnB = &UCSR1B;
		m_UCSRnC = &UCSR1C;
		m_UCSZn0 = UCSZ10;
		m_UCSZn1 = UCSZ11;
		m_UCSZn2 = UCSZ12;
		m_U2Xn = U2X1;
	}
	
	*m_UCSRnB = (1 << m_RXENn) | (1 << m_TXENn) | (1 << RXCIE1);
	*m_UCSRnC |= (1 << m_UCSZn1) | (1 << m_UCSZn0); //8 bit mode
	if (nine_bit)
		*m_UCSRnB |= (1 << m_UCSZn2); //9 bit mode
	*m_UCSRnC |= (0 << USBS1); //one stop bit else 1
	//*m_UCSRnC |= 0b00000000; //no parity bit
}

void UART::begin(int baud)
{
	uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
	*m_UBRRnH = baud_setting >> 8;
	*m_UBRRnL = baud_setting;
	*m_UCSRnA &= ~(1 << m_U2Xn); //disable rate doubler
}

bool UART::available()
{
	return ((unsigned int)(BUFFER_SIZE + v_pos - v_end)) % BUFFER_SIZE;
}

int UART::print(String str)
{
	int i;
	for (i = 0; i < str.length(); i++)
		write((uint8_t)str[i]);
	return i;
}

void UART::write(uint8_t data, int mode)
{
	while (!(*m_UCSRnA & (1 << UDRE))) {}
	if (mode)
		*m_UCSRnB |= (1 << TXB8);
	else
		*m_UCSRnB &= ~(1 << TXB8);
	*m_UDRn = data;
}

int UART::read()
{
	if (v_pos == v_end) {
		return -1;
	} else {
		unsigned char c = v_buffer[v_pos].value;
		v_pos = (v_pos + 1) % BUFFER_SIZE;
		return c;
	}
}

//uart 0 causes compiler errors
/*
ISR(USART0_RX_vect)
{
	if (v_uart == 0)
	{
		char status = *m_UCSRnA;

		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		v_buffer[v_end].mode = (*m_UCSRnB >> 1) & 0x01;
		if (v_buffer[v_end].mode == 1)
		{
			//we got an ack or an checksum
			v_msg_complete = true;
		}
		v_buffer[v_end++].value = *m_UDRn;
	}
}
*/

ISR(USART1_RX_vect)
{
	if (v_uart == 1)
	{
		char status = *m_UCSRnA;

		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		v_buffer[v_end].mode = (*m_UCSRnB >> 1) & 0x01;
		if (v_buffer[v_end].mode == 1)
		{
			//we got an ack or an checksum
			v_msg_complete = true;
		}
		v_buffer[v_end++].value = *m_UDRn;
	}
}

ISR(USART2_RX_vect)
{
	if (v_uart == 2)
	{
		char status = *m_UCSRnA;

		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		v_buffer[v_end].mode = (*m_UCSRnB >> 1) & 0x01;
		if (v_buffer[v_end].mode == 1)
		{
			//we got an ack or an checksum
			v_msg_complete = true;
		}
		v_buffer[v_end++].value = *m_UDRn;
	}
}

ISR(USART3_RX_vect)
{
	if (v_uart == 3)
	{
		char status = *m_UCSRnA;

		if (status & ((1 << FE) | (1 << DOR) | (1 << UPE)))
		{
			v_error = true;
		}
		v_buffer[v_end].mode = (*m_UCSRnB >> 1) & 0x01;
		if (v_buffer[v_end].mode == 1)
		{
			//we got an ack or an checksum
			v_msg_complete = true;
		}
		v_buffer[v_end++].value = *m_UDRn;
	}
}