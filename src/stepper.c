#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define A_DIR PD4
#define B_DIR PD7

volatile uint16_t stepsRemain = 0;
volatile uint8_t stepFlag = 0;
volatile uint16_t millis = 0;

typedef struct {
  uint8_t upperA, lowerA, upperB, lowerB;
  uint16_t rawA, rawB, measA, measB, setA, setB;
  int16_t errA, errB;
} Stepper;

Stepper stepper;

ISR(TIMER1_COMPA_vect)
{
  if (stepsRemain > 0) {
    stepFlag = 1;
    stepsRemain--;
  }
  ++millis;
}

int main(void)
{
  initUSART();
  initI2C();
  initStepper();
  initTimer1();
  sei();

  uint16_t steps = 256;

  while (1) {
    stepsRemain = steps;
    PORTD &= ~(1 << A_DIR);       
    PORTD |= (1 << B_DIR);      
    while (stepsRemain > 0) {
      if (stepFlag) {
        stepFlag = 0;
        printString("stepsRemain: ");
        printInteger(stepsRemain);
        printString("\r\n");
        step();
      }
    }
    _delay_ms(1000);

    printString("\r\n");
    printString("Switching directions");
    printString("\r\n");
    _delay_ms(1000);

    stepsRemain = steps;
    PORTD |= (1 << A_DIR);      
    PORTD &= ~(1 << B_DIR);       
    while (stepsRemain > 0) {
      if (stepFlag) {
        stepFlag = 0;
        printString("stepsRemain: ");
        printInteger(stepsRemain);
        printString("\r\n");
        step();
      }
    }
    _delay_ms(1000);

    printString("\r\n");
    printString("Switching directions");
    printString("\r\n");
    _delay_ms(1000);
  }

  return 0;
}


