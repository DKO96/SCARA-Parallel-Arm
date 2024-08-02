#include <avr/io.h>
#include "USART.h"

#define CORDIC_TABSIZE 14
#define FP (1 << 14)
#define K 9949

uint16_t cordic_tab [] = {
  12868, 7596, 4014, 2037, 1023, 512, 256, 
  128, 64, 32, 16, 8, 4, 2, 1,
};

int32_t fpsqrt(int32_t r)
{
  uint32_t b = 1UL << 30;
  uint32_t q = 0;

  while (b > r)
    b >>= 2;

  while( b > 0 ) {
    uint32_t t = q + b;
    q >>= 1;           

    if( r >= t ) {     
      r -= t;        
      q += b;        
    }

    b >>= 2;
  }

  return q;
}

void cordic(int32_t theta, int32_t* s, int32_t* c)
{
  int32_t x = 1 << 14;
  int32_t y = 0;
  int32_t z = theta;
  int32_t d, tx, ty, tz;

  for (uint8_t i = 0; i < CORDIC_TABSIZE; ++i) {
    d = (z >= 0) ? 1 : -1;
    tx = x - d * (y >> i);
    ty = d * (x >> i) + y;
    tz = z - (d * cordic_tab[i]);

    x = tx; y = ty; z = tz;
  }

  *c = (K * x) >> 14;
  *s = (K * y) >> 14;
}

int32_t arctan(int32_t x, int32_t y)
{
  int32_t z = 0;
  int32_t d, tx, ty, tz;

  for (uint8_t i = 0; i < CORDIC_TABSIZE; ++i) {
    d = (y >= 0) ? -1 : 1;
    tx = x - d * (y >> i);
    ty = d * (x >> i) + y;
    tz = z - (d * cordic_tab[i]);

    x = tx; y = ty; z = tz;
  }

  return z;
}

int32_t arccos(int32_t x)
{
  int32_t y = fpsqrt(1 - x*x);
  int32_t z = 0;
  int32_t d, tx, ty, tz;

  for (uint8_t i = 0; i < CORDIC_TABSIZE; ++i) {
    d = (y >= 0) ? -1 : 1;
    tx = x - d * (y >> i);
    ty = d * (x >> i) + y;
    tz = z - (d * cordic_tab[i]);

    x = tx; y = ty; z = tz;
  }

  return z;
}


int main(void) 
{
  initUSART();

  int32_t s, c;
  int32_t theta = (int32_t)(1.22173 * FP);
  cordic(theta, &s, &c);

  printString("cos: ");
  printInteger(c);
  printString("\t");
  printString("sin: ");
  printInteger(s);
  printString("\r\n");

  int32_t x = 0.1 * FP;
  int32_t y = 0.5 * FP;
  int32_t z = arctan(x, y);

  printString("arctan: ");
  printInteger(z);
  printString("\r\n");
  
  int32_t w = fpsqrt();

  printString("fpsqrt: ");
  printInteger(w);
  printString("\r\n");






  return 0;
}
