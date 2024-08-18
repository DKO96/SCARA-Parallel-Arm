#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define MAX_BUFFER 512
#define STAMARKER 0xBE
#define MIDMARKER 0xC0
#define ENDMARKER 0xDE

volatile uint8_t stepFlag = 0;
volatile uint32_t totalSteps = 0;

volatile uint8_t motorA[MAX_BUFFER];
volatile uint8_t motorB[MAX_BUFFER];
volatile uint16_t bIndex = 0;
volatile enum {IDLE, RECEIVING, MOVING} state = IDLE;
static volatile uint8_t* currentMotor = motorA;

void setup (void);    // initialize utility functions

ISR (TIMER1_COMPA_vect)
{
  // 16-bit interrupt for stepper motors
  if (state == MOVING) stepFlag = 1;
}

ISR (USART_RX_vect)
{
  // USART interrupt to receive stepper motor data array
  uint8_t receivedByte = UDR0;

  switch (state) {
    case IDLE:
      if (receivedByte == STAMARKER) {
        bIndex = 0;
        state = RECEIVING;
        currentMotor = motorA;
        TIMSK1 &= ~(1 << OCIE1A);       // disable 16-bit timer interrupt
      }
      break;

    case RECEIVING:
      if (receivedByte == MIDMARKER) {
        bIndex = 0;
        currentMotor = motorB;

      } else if (receivedByte == ENDMARKER) {
        totalSteps = bIndex;
        state = MOVING;
        TIMSK1 |= (1 << OCIE1A);        // enable 16-bit timer interrupt

      } else if (bIndex < MAX_BUFFER) {
        if (bIndex % 2 == 0) {
          currentMotor[bIndex/2] = (receivedByte - '0') << 1;
        } else {
          currentMotor[bIndex/2] |= (receivedByte - '0');
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
  OCR1A = 6250;

  while (1) {
    if (state != MOVING || !stepFlag) {
      continue;
    }

    // move stepper motors
    stepFlag = 0;

    for (uint16_t i = 0; i < totalSteps/2; i++) {
      stepMotor(motorA[i], A_DIR, A_STEP);
      _delay_us(100);
      stepMotor(motorB[i], B_DIR, B_STEP);
      _delay_us(100);
    }

    //printString("\r\n");
    //_delay_ms(100);
    printString("DONE\r\n");
    state = IDLE;
  }

  return 0;
}







