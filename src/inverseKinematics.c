#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"

#define FP 65536
#define PI_4 51472
#define PI_34 154416
#define PI 205887
#define COEFF1 -45753
#define COEFF2 -57191
#define COEFF3 102944

typedef struct {
  int64_t l1;
  int64_t l2;
  int64_t l3;
  int64_t l4;
  int64_t d;
} Scara;

uint32_t q1, q2, p1, p2;
Scara scara = {
  (int64_t)(0.12*FP), 
  (int64_t)(0.12*FP), 
  (int64_t)(0.12*FP), 
  (int64_t)(0.12*FP), 
  (int64_t)(0.12*FP),
};

int64_t arctan2(int64_t y, int64_t x)
{
  int64_t abs_y = (y > 0) ? y : -y;
  int64_t r, angle;

  if (x >= 0) {
    r = ((x - abs_y) << 8) / (x + abs_y);
    angle = PI_4 - ((PI_4 * r) >> 8);
  }
  else {
    r = ((x + abs_y) << 8) / (abs_y - x);
    angle = PI_34 - ((PI_4 * r) >> 8);
  }

  return (y < 0) ? -angle : angle;
}

int64_t arccos(int64_t x)
{
  if (x > FP) x = FP;
  if (x < -FP) x = -FP;

  int64_t x2 = (int64_t)x * x/ FP;
  int64_t x3 = (int64_t)x2 * x/ FP;
  int64_t result = ((int64_t)COEFF1 * x3 / FP) + ((int64_t)COEFF2 * x / FP) + COEFF3;

  return result;
}

uint32_t bsqrt(int64_t n)
{
  uint64_t i = n * FP;
  uint64_t j = FP;

  while (i > j) {
    i = (i + j) / 2;
    j = n / i;
  }
  
  return i;
}

void computeAngle(int64_t x, int64_t y, uint32_t* q1, uint32_t* q2)
{
  // compute angle for motor A
  int64_t tempC = (int64_t)x*x + (int64_t)y*y;
  int64_t C = bsqrt(tempC);

  uint64_t tempQ1 = (-(scara.l2*scara.l2) + (scara.l1*scara.l1) + C*C) / (2*scara.l1*C);
  *q1 = arctan2(y, x) + arccos(tempQ1);

  // compute angle for motor B
  int64_t tempE0 = scara.d - x;
  int64_t tempE1 = (tempE0) * (tempE0);
  uint64_t tempE2 = y * y; 
  uint64_t tempE = tempE1 + tempE2;
  uint64_t E = bsqrt(tempE);

  //uint64_t tempQ2 = (-(scara.l4*scara.l4) + (scara.l3*scara.l3) + E*E) / (2*scara.l3*E);
  //*q2 = PI - arctan2(y, tempE0) - arccos(tempQ2);
  
  uint64_t l3_sq = scara.l3 * scara.l3;
  uint64_t l4_sq = scara.l4 * scara.l4;
  uint64_t E_sq = E*E;

  int64_t num = -l4_sq + l3_sq + E_sq;
  int64_t div = 2*scara.l3*E;

  printString("E_sq: ");
  printInteger(E_sq);
  printString("\t");

  printString("num: ");
  printInteger(num);
  printString("\t");

  printString("div: ");
  printInteger(div);
  printString("\t");

  int64_t tempQ2 = (-l4_sq + l3_sq + E_sq) / div;
  int64_t at = arctan2(y, tempE0);
  int64_t ac = arccos(tempQ2);

  printString("tempQ2: ");
  printInteger(tempQ2);
  printString("\t");

  printString("at: ");
  printInteger(at);
  printString("\t");

  printString("ac: ");
  printInteger(ac);
  printString("\t");

  //printString("q2: ");
  //printInteger(*q2);
  //printString("\t");

}

void convertAngle(uint32_t* p1, uint32_t* p2)
{
  printString("p1: ");
  printInteger(*p1);
  printString("\t");

  printString("p2: ");
  printInteger(*p2);
  printString("\t");

  *p1 = (uint64_t)(q1 * 4096) / (2*PI / FP);
  *p2 = (uint64_t)(q2 * 4096) / (2*PI / FP);
}


int main(void)
{
  initUSART();

  float yc = 1;
  float xc = 1;

  int64_t x = (int64_t)(xc * FP);
  int64_t y = (int64_t)(yc * FP);
  int64_t n = 144;

  while (1) {
    //printString("x: ");
    //printInteger(x);
    //printString("\t");
    //printString("y: ");
    //printInteger(y);
    //printString("\t");

    //int32_t at = arctan2(y,x);
    //printString("arctan2: ");
    //printInteger(at);
    //printString("\t");
    //
    //int32_t ac = arccos(x);
    //printString("arccos: ");
    //printInteger(ac);
    //printString("\t");

    //uint32_t sr = bsqrt(n);
    //printString("bsqrt: ");
    //printInteger(sr);
    //printString("\r\n");


    computeAngle(x, y, &q1, &q2);
    //convertAngle(&p1, &p2);

    //printString("q1: ");
    //printInteger(q1);
    //printString("\t");

    //printString("q2: ");
    //printInteger(q2);
    //printString("\t");

    //printString("p1: ");
    //printInteger(p1);
    //printString("\t");

    //printString("p2: ");
    //printInteger(p2);
    //printString("\r\n");

    printString("\r\n");
    _delay_ms(100);

  }


  return 0;
}
