#include <avr/io.h>
#include <util/delay.h>
#include "utils.h"

////////////////////////////////////////////////////////////////////////////////
/// Timer Functions
////////////////////////////////////////////////////////////////////////////////
void initTimer1(void)
{
  TCCR1A = 0x00;
  TCCR1B = (1 << WGM12) | (1 << CS12);
  TIMSK1 = (1 << OCIE1A);
  OCR1A = 200;      // 62500 = 1s = 1Hz
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
/// AS5600 Functions
////////////////////////////////////////////////////////////////////////////////
void initAS5600(void)
{
  startI2C();
  sendI2C(AS_ADDR_W);
  sendI2C(CONF);
  sendI2C(0x03);
  stopI2C();
}

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
  return (*raw_angle - OFFSET_A + 4096) % 4096;
}

uint16_t offsetAngle_B(uint16_t *raw_angle)
{
  return (*raw_angle - OFFSET_B + 4096) % 4096;
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
/// TMC2209 Functions
////////////////////////////////////////////////////////////////////////////////
void initStepper(void)
{
  DDRD |= (1 << A_STEP) | (1 << A_DIR) | (1 << A_ENABLE);
  PORTD &= ~(1 << A_ENABLE);

  DDRD |= (1 << B_STEP) | (1 << B_DIR) | (1 << B_ENABLE);
  PORTD &= ~(1 << B_ENABLE);
}

void setDirection(uint8_t motorDirPin, int16_t error)
{
  (error > TOL) ? NEG(motorDirPin) : POS(motorDirPin);
}

void stepMotor(uint8_t motorStepPin)
{
  PORTD |= (1 << motorStepPin);
  _delay_us(STEPPER_DELAY);
  PORTD &= ~(1 << motorStepPin);
  _delay_us(STEPPER_DELAY);
}


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////
int16_t abs(int16_t x)
{
  int16_t val = x > 0 ? x : -x;
  return val;
}

int16_t max(int16_t a, int16_t b)
{
  int16_t max = a > b ? a : b;
  return max;
}

int16_t min(int16_t a, int16_t b)
{
  int16_t min = a < b ? a : b;
  return min;
}

int16_t ratio(int16_t a, int16_t b)
{
  int16_t maxErr = max(abs(a), abs(b));
  int16_t minErr = min(abs(a), abs(b));
  return maxErr / minErr;
}














