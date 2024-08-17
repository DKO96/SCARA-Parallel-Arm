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
volatile uint16_t bIndex = 0;
volatile enum {IDLE, RECEIVING, MOVING} state = IDLE;

void setup (void);    // initialize utility functions

ISR (TIMER1_COMPA_vect)
{
  // 16-bit interrupt for stepper motors
  if (state == MOVING) stepFlag = 1;
}

ISR (USART_RX_vect)
{
  // USART interrupt to receive stepper motor data array
  // read USART byte data
  uint8_t receivedByte = UDR0;

  switch (state) {
    case IDLE:
      if (receivedByte == START_MARKER) {
        bIndex = 0;
        state = RECEIVING;
        TIMSK1 &= ~(1 << OCIE1A);       // disable 16-bit timer interrupt
      }
      break;

    case RECEIVING:
      if (receivedByte == END_MARKER) {
        totalSteps = bIndex;
        state = MOVING;
        TIMSK1 |= (1 << OCIE1A);        // enable 16-bit timer interrupt

      } else if (bIndex < MAX_BUFFER) {
        if (bIndex % 2 == 0) {
          motor[bIndex/2] = (receivedByte - '0') << 1;
        } else {
          motor[bIndex/2] |= (receivedByte - '0');
        }
        bIndex++;
      }
      break;

    default:
      break;
  }
}


int main (void)
{
  setup();
  OCR1A = 50;

  while (1) {
    if (state != MOVING || !stepFlag) {
      continue;
    }

    // move stepper motors
    stepFlag = 0;

    for (uint16_t i = 0; i < totalSteps / 2; i++) {
      printInteger(motor[i]);
      printString(" ");
    }

    printString("\r\n");
    _delay_ms(100);
    printString("DONE\r\n");
    state = IDLE;
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





