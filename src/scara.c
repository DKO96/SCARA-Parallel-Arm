#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define ATPOINT(error) (error > TOL || error < -TOL)

volatile uint8_t stepFlag = 0;

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
  stepper.measA = getAngle_A(&stepper.rawA);
  stepper.errA = wrapAngle(stepper.setA - stepper.measA);

  channel(CHANNEL_B);
  readAngle(&stepper.upperB, &stepper.lowerB, &stepper.rawB);
  stepper.measB = getAngle_B(&stepper.rawB);
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


int main (void)
{
  initUSART();
  initI2C();
  initTimer1();
  initStepper();
  sei();

  stepper.setA = 0;
  stepper.setB = 2048;
  stepper.setA = 3072;
  stepper.setB = 3072;

  while (1) {
    updateError();
    motorSync();

    if (ATPOINT(stepper.errA) || ATPOINT(stepper.errB)) {
      stepper.countA = 0;
      stepper.countB = 0;

      while (ATPOINT(stepper.errA) || ATPOINT(stepper.errB)) {
        updateError();
        motorSync();

        if (stepFlag) {
          stepFlag = 0;

          if (stepper.countA % stepper.ratioA == 0) {
            controlA();
          }

          if (stepper.countB % stepper.ratioB == 0) {
            controlB();
          }

          ++stepper.countA;
          ++stepper.countB;
        }
      }
    }
  }

  return 0;
}





