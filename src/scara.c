#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define MAX_BUFFER 128
#define START_MARKER 0xBE
#define END_MARKER 0xEF

volatile uint8_t stepFlag = 0;
volatile uint32_t stepCount = 0;
volatile uint32_t totalSteps = 0;

volatile int8_t motor[MAX_BUFFER];
volatile uint16_t bufferIndex = 0;
volatile uint8_t receiving = 0;
volatile uint8_t dataReady = 0;

ISR (TIMER1_COMPA_vect)
{
  if (!receiving) stepFlag = 1;
}

ISR (USART_RX_vect)
{
  uint8_t receivedByte = UDR0;

  if (receivedByte == START_MARKER) {
    bufferIndex = 0;
    receiving = 1;
    TIMSK1 &= ~(1 << OCIE1A);

  } else if (receivedByte == END_MARKER) {
    totalSteps = bufferIndex;
    receiving = 0;
    dataReady = 1;
    TIMSK1 |= (1 << OCIE1A);

  } else if (receiving) {
    if (bufferIndex < MAX_BUFFER) {
      if (bufferIndex % 2 == 0) {
        motor[bufferIndex / 2] = (receivedByte - '0') << 1;
      } else {
        motor[bufferIndex / 2] |= (receivedByte - '0');
      }
      bufferIndex++;
    }
  }
}

void setup (void);

int main (void)
{
  setup();
  OCR1A = 50;

  while (1) {
    if (!stepFlag) continue;
    stepFlag = 0;
    
    if (dataReady) {
      for (uint16_t i = 0; i < totalSteps / 2; i++) {
        printInteger(motor[i]);
        printString(" ");
      }
      printString("\r\n");
      _delay_ms(100);
      printString("READY\r\n");
      dataReady = 0;
    }
  }

  return 0;
}


void setup(void)
{
  initUSART();
  initTimer1();
  initStepper();
  sei();
}





