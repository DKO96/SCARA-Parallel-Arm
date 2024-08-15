#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

volatile uint8_t stepFlag = 0;
uint32_t stepCount = 0;

ISR(TIMER1_COMPA_vect)
{
  stepFlag = 1;
}

void setup(void);

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





