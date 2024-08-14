#include <avr/io.h>
#include "USART.h"
#include <util/setbaud.h>

void initUSART(void) 
{                                
  UBRR0H = UBRRH_VALUE;                       
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   
}

void transmitByte(uint8_t data)
{
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = data;
}

void printByte(uint8_t byte)
{
  transmitByte('0' + (byte / 100));
  transmitByte('0' + ((byte / 10) % 10));
  transmitByte('0' + (byte % 100));
}

void printBinary(uint16_t byte) {
    for (int i = 15; i >= 0; i--) {
        if (byte & (1 << i)) {
            transmitByte('1');
        } else {
            transmitByte('0');
        }
    }
}

void printString(const char myString[]) 
{
  uint8_t i = 0;
  while (myString[i]) {
    transmitByte(myString[i]);
    i++;
  }
}

void printInteger(int32_t num)
{
  char buffer[31];  
  int i = 0;

  if (num == 0) {
    buffer[i++] = '0';
  } else {
    if (num < 0) {
      buffer[i++] = '-';
      num = -num;  
    }
    int start = i;  
    while (num > 0) {
      buffer[i++] = '0' + (num % 10);  
      num /= 10;                       
    }
    int end = i - 1;
    while (start < end) {
      char temp = buffer[start];
      buffer[start] = buffer[end];
      buffer[end] = temp;
      start++;
      end--;
    }
  }
  buffer[i] = '\0';  
  printString(buffer); 
}



