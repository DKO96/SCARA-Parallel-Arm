#ifndef HELPER_H
#define HELPER_H

#include <avr/io.h>

#define A_ENABLE PD2
#define A_STEP PD3
#define A_DIR PD4
#define B_ENABLE PD5
#define B_STEP PD6
#define B_DIR PD7

#define TCA_ADDR 0x70
#define CHANNEL_A 4
#define CHANNEL_B 2

#define AS_ADDR_W 0x6C
#define AS_ADDR_R 0x6D
#define CONF 0x07
#define ANGLE_H 0x0E 

#define ANGLE_MAX 4096
#define OFFSET_A 540
#define OFFSET_B 1302

#define NEG(PIN) ((PORTD) |= (1 << (PIN))) 
#define POS(PIN) ((PORTD) &= ~(1 << (PIN)))

void initTimer1(void);

void initStepper(void);
void stepMotor(int8_t direction, uint8_t dirPin, uint8_t stepPin);

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

void setup(void);
void initScara(void);
void moveStart(int16_t error, uint8_t dirPin, uint8_t stepPin);
int16_t abs(int16_t x);

#endif 









