#ifndef BAUD                          
#define BAUD 38400
#endif

#define   USART_HAS_DATA   bit_is_set(UCSR0A, RXC0)
#define   USART_READY      bit_is_set(UCSR0A, UDRE0)

void initUSART(void);
void transmitByte(uint8_t data);
void printByte(uint8_t byte);
void printBinary(uint16_t byte);
void printString(const char myString[]);
void printInteger(int64_t num);
