#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define STAMARKER 0xBE
#define MIDMARKER 0xC0
#define ENDMARKER 0xDE

typedef struct{
  volatile int8_t motorA;
  volatile int8_t motorB;
} Stepper;
Stepper stepper;

volatile enum {IDLE, RECEIVING, MOVING, DONE} state = IDLE;
static volatile int8_t* currentMotor = &stepper.motorA;

ISR (TIMER1_COMPA_vect)
{
  // 16-bit interrupt for stepper motors
  if (state == MOVING) {
    stepMotor(stepper.motorA, A_DIR, A_STEP);
    stepMotor(stepper.motorB, B_DIR, B_STEP);
    state = DONE;
  }
}

ISR (USART_RX_vect)
{
  // USART interrupt to receive stepper motor data 
  uint8_t receivedByte = UDR0;

  switch (state) {
    case IDLE:
      if (receivedByte == STAMARKER) {
        state = RECEIVING;
        currentMotor = &stepper.motorA;
        TIMSK1 &= ~(1 << OCIE1A);       // disable 16-bit timer interrupt
      }
      break;

    case RECEIVING:
      if (receivedByte == MIDMARKER) {
        currentMotor = &stepper.motorB;

      } else if (receivedByte == ENDMARKER) {
        state = MOVING;
        TIMSK1 |= (1 << OCIE1A);        // enable 16-bit timer interrupt

      } else { 
        if (receivedByte == 0x01) {
          *currentMotor = -1;
        } else if (receivedByte == 0x11) {
          *currentMotor = 0;
        } else if (receivedByte == 0x10) {
          *currentMotor = 1;
        }
      }
      break;

    default:
      break;
  }
}


int main (void)
{
  setup();

  while (1) {
    if (state == DONE) {
      printString("N\r\n");
      state = IDLE;
    }
  }

  return 0;
}







