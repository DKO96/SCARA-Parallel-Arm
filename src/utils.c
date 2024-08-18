#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

////////////////////////////////////////////////////////////////////////////////
/// Timer Functions
////////////////////////////////////////////////////////////////////////////////
void initTimer1(void)
{
  TCCR1A = 0x00;
  TCCR1B = (1 << WGM12) | (1 << CS12);
  TIMSK1 = (1 << OCIE1A);
}


////////////////////////////////////////////////////////////////////////////////
/// TMC2209 Functions
////////////////////////////////////////////////////////////////////////////////
void initStepper(void)
{
  DDRD |= (1 << A_STEP) | (1 << A_DIR) | (1 << A_ENABLE);
  PORTD &= ~(1 << A_ENABLE);

  DDRD |= (1 << B_STEP) | (1 << B_DIR) | (1 << B_ENABLE);
  PORTD &= ~(1 << B_ENABLE);
}

void stepMotor(int8_t direction, uint8_t dirPin, uint8_t stepPin)
{
  if (direction == 2) {
    POS(dirPin);
  } else if (direction == 1) {
    NEG(dirPin);
  } else {
    return;
  }

  PORTD |= (1 << stepPin);
  _delay_us(10);
  PORTD &= ~(1 << stepPin);
}


////////////////////////////////////////////////////////////////////////////////
/// Setup Functions
////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  initUSART();
  initTimer1();
  initStepper();
  sei();
}






