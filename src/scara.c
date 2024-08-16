#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define BUFFER_SIZE 4096
#define START_MARKER 0xBE
#define END_MARKER 0xEF

volatile uint8_t stepFlag = 0;
volatile uint32_t stepCount = 0;
volatile uint16_t bufferIndex = 0;
volatile uint8_t usartReceiving = 0;
volatile 

ISR (TIMER1_COMPA_vect)
{
  if (usartReceiving == 0) stepFlag = 1;
}

ISR (USART_RX_vect)
{
  // disable 16-bit timer interrupt
  TIMSK1 &= ~(1 << OCIE1A);
  usartReceiving = 1;

  // read incoming bytes
  uint8_t receivedByte = UDR0;

  // check for start marker
  if (receivedByte == START_MARKER) {
    bufferIndex = 0;    // reset buffer index for new message
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

    //printInteger(sizeof(motorA));
    //printString("\t");
    //printInteger(sizeof(motorB));
    //printString("\r\n");

    stepMotor(motorA[stepCount], A_DIR, A_STEP);
    stepMotor(motorB[stepCount], B_DIR, B_STEP);
    stepCount++;

    if (stepCount > sizeof(motorA)) {
      stepCount = 0;
      _delay_ms(2000);
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





