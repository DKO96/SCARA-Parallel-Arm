#ifndef HELPER_H
#define HELPER_H

#include <avr/io.h>

#define TCA_ADDR 0x70
#define CHANNEL_A 4
#define CHANNEL_B 2

#define AS_ADDR_W 0x6C
#define AS_ADDR_R 0x6D
#define CONF 0x07
#define ANGLE_H 0x0E 

#define A_ENABLE PD2
#define A_STEP PD3
#define A_DIR PD4
#define B_ENABLE PD5
#define B_STEP PD6
#define B_DIR PD7
#define STEPPER_DELAY 100

#define POS(PIN) ((PORTD) |= (1 << (PIN))) 
#define NEG(PIN) ((PORTD) &= ~(1 << (PIN)))
#define TOL 2

#define ANGLE_MAX 4096
#define OFFSET_A 2583
#define OFFSET_B 3370

#define CORDIC_TABSIZE 14
#define FP (1 << 14)
#define K 9949

void initTimer1(void);
void initTimer0(void);

void initI2C(void);
void startI2C(void);
void stopI2C(void);
void sendI2C(uint8_t data);
uint8_t readACK(void);
uint8_t readNACK(void);

void channel(uint8_t bus);
void initAS5600(void);
void readAngle(uint8_t *upper, uint8_t *lower, uint16_t *raw_angle);
uint16_t offsetAngle_A(uint16_t *raw_angle);
uint16_t offsetAngle_B(uint16_t *raw_angle);
int16_t wrapAngle(int16_t error);

void initStepper(void);
void setDirection(uint8_t motorDirPin, int16_t error);
void stepMotor(uint8_t motorStepPin);

int16_t abs(int16_t x);
int16_t max(int16_t a, int16_t b);
int16_t min(int16_t a, int16_t b);
int16_t ratio(int16_t a, int16_t b);
int32_t fpsqrt(int32_t r);

int32_t arctan(float fy, float fx);
int32_t arccos(float p);

#endif 









