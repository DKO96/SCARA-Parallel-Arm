#ifndef HELPER_H
#define HELPER_H

#include <avr/io.h>

#define A_ENABLE PD2
#define A_STEP PD3
#define A_DIR PD4
#define B_ENABLE PD5
#define B_STEP PD6
#define B_DIR PD7

#define NEG(PIN) ((PORTD) |= (1 << (PIN))) 
#define POS(PIN) ((PORTD) &= ~(1 << (PIN)))

void initTimer1(void);
void initTimer0(void);

void initStepper(void);
void stepMotor(int8_t direction, uint8_t dirPin, uint8_t stepPin);


void setup(void);

#endif 









