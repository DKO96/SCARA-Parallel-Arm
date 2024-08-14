#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"
#include "utils.h"

#define L1 120
#define L2 200
#define D 118


int main(void) 
{
  initUSART();

  int32_t x = 139;
  int32_t y = 180;

  int32_t r1 = fpsqrt(x*x + y*y);
  int32_t theta = arctan(y, x);
  uint32_t gamma = 200 * 200;



  while (1) {
    printString("r_one: ");
    printInteger(r1);
    printString("\r\n");

    printString("theta: ");
    printInteger(theta);
    printString("\r\n");
    
    printString("gamma: ");
    printInteger(gamma);
    printString("\r\n");

    _delay_ms(1000);
  }

  






  return 0;
}
