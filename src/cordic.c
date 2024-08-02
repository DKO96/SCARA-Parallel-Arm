#include <avr/io.h>
#include "USART.h"

#define CORDIC_TABSIZE 14
#define FP (1 << 14)
#define K 9949

uint16_t cordic_tab [] = {
  12868, 7596, 4014, 2037, 1023, 512, 256, 
  128, 64, 32, 16, 8, 4, 2, 1,
};

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

    printString("x: ");
    printInteger(x);
    printString("\t");

    printString("y: ");
    printInteger(y);
    printString("\t");

    printString("z: ");
    printInteger(z);
    printString("\r\n");
  }

  *c = (K * x) >> 14;
  *s = (K * y) >> 14;

}


int main(void) 
{
  initUSART();

  int32_t s, c;
  int32_t theta = (int32_t)(1.22173 * FP);

  printString("theta: ");
  printInteger(theta);
  printString("\r\n");


  cordic(theta, &s, &c);

  printString("c: ");
  printInteger(c);
  printString("\t");

  printString("s: ");
  printInteger(s);
  printString("\r\n");


  return 0;
}
