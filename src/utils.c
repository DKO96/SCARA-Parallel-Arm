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
  OCR1A = 100;
}

void initTimer2(void)
{
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS21);
  TIMSK2 = (1 << OCIE2A);
  OCR2A = 249;
}

////////////////////////////////////////////////////////////////////////////////
/// TMC2209 Functions
////////////////////////////////////////////////////////////////////////////////
void initMotors(void)
{
  // initialize stepper motors
  DDRD |= (1 << A_STEP) | (1 << A_DIR) | (1 << A_ENABLE);
  PORTD &= ~(1 << A_ENABLE);

  DDRD |= (1 << B_STEP) | (1 << B_DIR) | (1 << B_ENABLE);
  PORTD &= ~(1 << B_ENABLE);

  // initialize servo motor
  DDRB |= (1 << PB3);
}

void stepMotor(int8_t direction, uint8_t dirPin, uint8_t stepPin)
{
  if (direction > 0) {
    POS(dirPin);
  } else if (direction < 0) {
    NEG(dirPin);
  } else {
    return;
  }

  PORTD |= (1 << stepPin);
  _delay_us(10);
  PORTD &= ~(1 << stepPin);
}


////////////////////////////////////////////////////////////////////////////////
/// I2C Functions
////////////////////////////////////////////////////////////////////////////////
void initI2C(void)
{
  TWBR = 0x48;          // bit rate register frequency = 400khz
  TWSR = 0x00;          // set bit rate prescaler = 1
  TWCR = (1 << TWEN);   // enables I2C interface
}

void startI2C(void)
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT))) {};
}

void stopI2C(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void sendI2C(uint8_t data)
{
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT))) {};
}

uint8_t readACK(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT))) {};
  return TWDR;
}

uint8_t readNACK(void)
{
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT))) {};
  return TWDR;
}


////////////////////////////////////////////////////////////////////////////////
/// TCA9548A Functions
////////////////////////////////////////////////////////////////////////////////
void channel(uint8_t bus)
{
  startI2C();
  sendI2C(TCA_ADDR << 1);
  sendI2C(1 << bus);
  stopI2C();
  _delay_us(5);
}


////////////////////////////////////////////////////////////////////////////////
/// AS5600 Functions
////////////////////////////////////////////////////////////////////////////////
void readAngle(uint8_t *upper, uint8_t *lower, uint16_t *raw_angle)
{
  startI2C();
  sendI2C(AS_ADDR_W);
  sendI2C(ANGLE_H);
  startI2C();
  sendI2C(AS_ADDR_R);
  *upper = readACK();
  *lower = readNACK();
  *raw_angle = (*upper << 8) | *lower;
  stopI2C();
}

uint16_t offsetAngle_A(uint16_t *raw_angle)
{
  return (*raw_angle - OFFSET_A + ANGLE_MAX) % ANGLE_MAX;
}

uint16_t offsetAngle_B(uint16_t *raw_angle)
{
  return (*raw_angle - OFFSET_B + ANGLE_MAX) % ANGLE_MAX;
}

int16_t wrapAngle(int16_t error)
{
  if (error > (ANGLE_MAX / 2)) {
    error -= ANGLE_MAX;
  } else if (error < -(ANGLE_MAX / 2)) {
    error += ANGLE_MAX;
  }
  return error;
}


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  initUSART();
  initTimer1();
  initTimer2();
  initMotors();
  initI2C();
  sei();
}

void initScara(void)
{
  typedef struct {
    uint8_t upperA, lowerA, upperB, lowerB;
    uint16_t rawA, rawB, measA, measB;
    int16_t errA, errB;
  } Stepper;
  Stepper stepper;

  uint16_t setA = 1024;
  uint16_t setB = 1024;

  channel(CHANNEL_A);
  readAngle(&stepper.upperA, &stepper.lowerA, &stepper.rawA);
  stepper.measA = offsetAngle_A(&stepper.rawA);
  stepper.errA = wrapAngle(setA - stepper.measA);

  channel(CHANNEL_B);
  readAngle(&stepper.upperB, &stepper.lowerB, &stepper.rawB);
  stepper.measB = offsetAngle_B(&stepper.rawB);
  stepper.errB = wrapAngle(setB - stepper.measB);

  moveStart(stepper.errA, A_DIR, A_STEP);
  moveStart(stepper.errB, B_DIR, B_STEP);
}

void moveStart(int16_t error, uint8_t dirPin, uint8_t stepPin) 
{
  int16_t steps = ((int32_t)error * 1600) / 4096 ;
  int16_t total = abs(steps);
  int8_t dir = (steps > 0) ? 1 : -1;

  for (uint16_t i = 0; i < total; i++) {
    stepMotor(dir, dirPin, stepPin);
    _delay_ms(1);
  }
}

int16_t abs(int16_t x)
{
  int16_t val = x > 0 ? x : -x;
  return val;
}



