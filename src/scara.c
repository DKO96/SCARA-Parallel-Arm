#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define NTOL(error) (error > TOL || error < -TOL)
#define SRATIO(count, ratio) (count % ratio == 0)
#define SIZE 4

volatile uint8_t stepFlag = 0;
uint8_t state = 0;
static uint16_t iteration = 0;

typedef struct {
  uint8_t upperA, lowerA, upperB, lowerB;
  uint16_t rawA, rawB, measA, measB, setA, setB;
  int16_t errA, errB;
  uint16_t countA, countB, ratioA, ratioB;
} Stepper;

Stepper stepper;

ISR(TIMER1_COMPA_vect)
{
  stepFlag = 1;
}

void updateError(void)
{
  channel(CHANNEL_A);
  readAngle(&stepper.upperA, &stepper.lowerA, &stepper.rawA);
  stepper.measA = offsetAngle_A(&stepper.rawA);
  stepper.errA = wrapAngle(stepper.setA - stepper.measA);

  channel(CHANNEL_B);
  readAngle(&stepper.upperB, &stepper.lowerB, &stepper.rawB);
  stepper.measB = offsetAngle_B(&stepper.rawB);
  stepper.errB = wrapAngle(stepper.setB - stepper.measB);
}

void controlA(void) {
  setDirection(A_DIR, stepper.errA);
  stepMotor(A_STEP);
}

void controlB(void) {
  setDirection(B_DIR, stepper.errB);
  stepMotor(B_STEP);
}

void motorSync(void)
{
  uint16_t stepA = abs(stepper.errA);
  uint16_t stepB = abs(stepper.errB);

  if (stepA > stepB) {
    stepper.ratioA = 1;
    stepper.ratioB = stepA / stepB;
  } else {
    stepper.ratioB = 1;
    stepper.ratioA = stepB / stepA;
  }
}

void motorState(uint8_t* state)
{
  if (!*state) {
    updateError();
    
    if (NTOL(stepper.errA) || NTOL(stepper.errB)) {
      stepper.countA = 0;
      stepper.countB = 0;
      *state = 1;
    }
  } else {
    if (stepFlag) {
      stepFlag = 0;
      updateError();
      motorSync();

      if (!NTOL(stepper.errA) && !NTOL(stepper.errB)) {
        *state = 0;
      }

      if (NTOL(stepper.errA) && SRATIO(stepper.countA, stepper.ratioA)) controlA();
      if (NTOL(stepper.errB) && SRATIO(stepper.countB, stepper.ratioB)) controlB();

      ++stepper.countA;
      ++stepper.countB;
    }
  }
}


int main (void)
{
  initUSART();
  initI2C();
  initTimer1();
  initStepper();
  sei();
  
  uint16_t arrayA[SIZE] = {4041, 4062, 3199, 3342};
  uint16_t arrayB[SIZE] = {2102, 2801, 2944, 2081};
  
  while (1) {
    for (uint8_t i = 0; i < SIZE; ++i) {
      stepper.setA = arrayA[i];
      stepper.setB = arrayB[i];
      state = 1;
      ++iteration;

      while (state == 1) {
        motorState(&state);
      }

      printString("iter: ");
      printInteger(iteration);
      printString("\t");
      
      printString("pos: ");
      printInteger(i);
      printString("\t");

      printString("errA: ");
      printInteger(stepper.errA);
      printString("\t");

      printString("errB: ");
      printInteger(stepper.errB);
      printString("\r\n");

      _delay_ms(1000);
    }
  }

  return 0;
}





