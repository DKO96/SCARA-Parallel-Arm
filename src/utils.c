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
  OCR1A = 10;
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
/// Setup Functions
////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  initUSART();
  initTimer1();
  initStepper();
  sei();
}






