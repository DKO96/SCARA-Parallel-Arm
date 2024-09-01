#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define STAMARKER 0xBE
#define PENUP 0xC0
#define PENDOWN 0xC1
#define ENDMARKER 0xDE

typedef struct{
  volatile int8_t stepperA;
  volatile int8_t stepperB;
  volatile uint8_t servo;
} Motor;
Motor motor;

volatile enum {IDLE, RECEIVING, MOVING, DONE} state = IDLE;
static volatile int8_t* currentMotor = &motor.stepperA;

ISR(TIMER1_COMPA_vect)
{
  // 16-bit interrupt for stepper motors
  if (state == MOVING) {
    stepMotor(motor.stepperA, A_DIR, A_STEP);
    stepMotor(motor.stepperB, B_DIR, B_STEP);
    state = DONE;
  }
}

ISR(TIMER2_COMPA_vect)
{
  static uint16_t counter = 0;

  if (counter < motor.servo) {
    PORTB |= (1 << PB3);
  } else {
    PORTB &= ~(1 << PB3);
  }
  counter++;

  if (counter >= 160) {
    counter = 0;
  }
}

ISR(USART_RX_vect)
{
  // USART interrupt to receive stepper motor data 
  uint8_t receivedByte = UDR0;

  switch (state) {
    case IDLE:
      if (receivedByte == STAMARKER) {
        state = RECEIVING;
        currentMotor = &motor.stepperA;
        TIMSK1 &= ~(1 << OCIE1A);       // disable 16-bit timer interrupt
      }
      break;

    case RECEIVING:
      if (receivedByte == PENUP || receivedByte == PENDOWN) {
        currentMotor = &motor.stepperB;
        
        if (receivedByte == PENUP) {
          motor.servo = 11;
        } else {
          motor.servo = 9;
        }

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


int main(void)
{
  setup();
  motor.servo = 11;
  initScara();

  while (1) {
    if (state == DONE) {
      printString("N\r\n");
      state = IDLE;
    }
  }

  return 0;
}







