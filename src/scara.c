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
volatile uint16_t currentStep = 0;

volatile uint8_t motorA[MAX_BUFFER];
volatile uint8_t motorB[MAX_BUFFER];
volatile uint16_t bIndex = 0;
volatile enum {IDLE, RECEIVING, MOVING, DONE} state = IDLE;
static volatile uint8_t* currentMotor = motorA;

void setup (void);    // initialize utility functions

ISR (TIMER1_COMPA_vect)
{
  // 16-bit interrupt for stepper motors
  if (state == MOVING && currentStep < totalSteps) {
    stepMotor(motorA[currentStep], A_DIR, A_STEP);
    stepMotor(motorB[currentStep], B_DIR, B_STEP);
    currentStep++;
  } else if (currentStep >= totalSteps) {
    state = DONE;
    currentStep = 0;
  }
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
  OCR1A = 10;

  while (1) {
    if (state == DONE) {
      printString("DONE\r\n");
      state = IDLE;
    }
  }

  return 0;
}







