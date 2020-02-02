#include <math.h>
#include <Wire.h>

// CONFIG
// comment this out to stop interrupts. This makes loop be called as fast as
// possible.
//#define INTERRUPT

// Number of samples taken to calibrate the gyroscope
const int n_samples = 200;

// If the angle offset from center is greater than this, then start balancing
const double balance_window = 0.5;

// END OF CONFIG

// REGISTER PORTS
#define lsm 0x6B
#define LED_PIN 13
#define BOD_RATE 9600
#define CTRL_REG1_G 0x10

#define axl 0x28
#define axh 0x29
#define ayl 0x2A
#define ayh 0x2B
#define azl 0x2C
#define azh 0x2D

#define gxl 0x18
#define gxh 0x19
#define gyl 0x1A
#define gyh 0x1B
#define gzl 0x1C
#define gzh 0x1D

// wheel radius
// r = 204.2mm

// UTIL
#define TO_DEG 180.0 / M_PI
#define TO_RAD M_PI / 180.0

#define whoami 0xF0

// GLOBALS
double y_g_calibration;

float pgain = 15;
float igain = 1.5;
float dgain = 10;
float gangle, adjuster;

byte balance;

float pid_p, pid_i, pid_d;

int motr, stepr, pulsecountr, pulsememr;
int motl, stepl, pulsecountl, pulsememl;
float error, sp, output, pidde;
float outputl, outputr;

unsigned long looptime;

void setup()
{
  Serial.begin(BOD_RATE);
  Wire.begin();

  // set i2c clock frequency
  TWBR = 12;

  // pinMode 2, 3, 4, 5 set to OUTPUT
  DDRD |= 0b00111100;
  // pinMode 13 set to OUTPUT
  DDRB |= 0b00100000;

  // configure device
  Wire.beginTransmission(lsm);
  Wire.write(CTRL_REG1_G);
  Wire.write(0b10100010);
  Wire.endTransmission();

  for (int i = 0; i < n_samples; i++)
  {
    if (i % 15 == 0) {
      toggleBlink();
    }

    Wire.beginTransmission(lsm);
    Wire.write(gyl);
    Wire.endTransmission();
    Wire.requestFrom(lsm, 2);

    const int lower_g = Wire.read();
    const int higher_g = Wire.read();
    const int calibration = lower_g | (higher_g << 8);
    y_g_calibration += calibration;
    delayMicroseconds(3000);
  }
  y_g_calibration /= n_samples;
  looptime = micros() + 4000;

  // timer registers
  #ifndef INTERRUPT
  TCNT2 = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= 0b00000010;
  TCCR2B |= 0b00000010;
  TIMSK2 |= 0b00000010;
  OCR2A = 39;
  #endif
}

void loop()
{
  // read from accelerometer
  Wire.beginTransmission(lsm);
  Wire.write(axl);
  Wire.endTransmission();
  Wire.requestFrom(lsm, 2);
  const int raxl = Wire.read();
  const int raxh = Wire.read();
  const int ax_value = (raxh << 8) | raxl;

  double ax = -ax_value/8200.0;
  if (ax > 1.0) {
    ax = 1.0;
  }
  else if (ax < -1.0)
  {
    ax = -1.0;
  }

  // read from gyroscope
  Wire.beginTransmission(lsm);
  Wire.write(gyl);
  Wire.endTransmission();
  Wire.requestFrom(lsm, 2);
  const int rgyl = Wire.read();
  const int rgyh = Wire.read();
  const int gy_value = (rgyh << 8) | rgyl;

  double gy = gy_value - y_g_calibration;

  /*
  Serial.println(ax);
  Serial.println(gy);
  Serial.println();
  delay(20);
  //*/

  double aangle = asin(ax) * TO_DEG;

  if (balance == 0 && aangle > -balance_window && aangle < balance_window)
  {
    gangle = aangle;
    balance = 1;
  }

  gangle += gy * 0.000031;
  gangle = gangle * 0.9996 + aangle * 0.0004;
  //gangle = aangle;

  //pid
  error = gangle - adjuster - sp;
  if (output > 5 || output < -5)
  {
    error += output * 0.0045;
  }
  pid_i += igain * error;
  //if(pid_i > 400)pid_i = 400;
  //else if(pid_i < -400)pid_i = -400;
  //p = error
  pid_p = pgain * error;
  pid_d = dgain * (error - pidde);
  output = pid_p + pid_i + dgain * (error - pidde);

  if (output > 500)
    output = 500;
  else if (output < -500)
    output = -500;

  pidde = error;

  if (output < 10 && output > -10)
  {
    output = 0;
  }

  if (gangle < -30 || gangle > 30 || balance == 0)
  {
    output = 0;
    pid_i = 0;
    balance = 0;
    adjuster = 0;
  }
  outputl = output;
  outputr = output;

  /*
  Serial.println(output);
  Serial.println();
  delay(20);
  //*/

  if (sp == 0 && output < 0)
    adjuster += 0.015;
  if (sp == 0 && output > 0)
    adjuster -= 0.015;

  if (outputl > 0)
    outputl = 203 - (1 / (outputl + 9)) * 2750;
  else if (outputl < 0)
    outputl = -203 - (1 / (outputl - 9)) * 2750;

  if (outputr > 0)
    outputr = 203 - (1 / (outputr + 9)) * 2750;
  else if (outputr < 0)
    outputr = -203 - (1 / (outputr - 9)) * 2750;

  if (outputl > 0)
    motl = 200 - outputl;
  else if (outputl < 0)
    motl = -200 - outputl;
  else
    motl = 0;

  if (outputr > 0)
    motr = 200 - outputr;
  else if (outputr < 0)
    motr = -200 - outputr;
  else
    motr = 0;
  /*
  if(outputl > 0)outputl = 505 - (1/(outputl + 9)) * 5500;
  else if(outputl < 0)outputl = -505 - (1/(outputl - 9)) * 5500;

  if(outputr > 0)outputr = 505 - (1/(outputr + 9)) * 5500;
  else if(outputr < 0)outputr = -505 - (1/(outputr - 9)) * 5500;

  if(outputl > 0)motl = 500 - outputl;
  else if(outputl < 0)motl = -500 - outputl;
  else motl = 0;

  if(outputr > 0)motr = 500 - outputr;
  else if(outputr < 0)motr = -500 - outputr;
  else motr = 0;*/
  stepl = motl;
  stepr = motr;

  #ifdef INTERRUPT

  PORTD |= 0b10000000;
  PORTD &= 0b01111111;

  #else

  while (looptime > micros());
  looptime += 2000;

  #endif
}

void toggleBlink() {
  const auto &currentValue = digitalRead(LED_PIN);
  digitalWrite(LED_PIN, !currentValue);
}

/*
int adjustment(){
    int read = analogRead(A0);
    int mapread = map(read, 0, 1023, 0, 1400);
    return mapread;
}
//*/

ISR(TIMER2_COMPA_vect)
{
  pulsecountl++;
  if (pulsecountl > pulsememl)
  {
    pulsecountl = 0;
    pulsememl = stepl;
    if (pulsememl < 0)
    {
      PORTD &= 0b11110111;
      //PORTD |= 0b00001000;
      pulsememl *= -1;
    }
    else
      PORTD |= 0b00001000;
    //else PORTD &= 0b11110111;
  }
  else if (pulsecountl == 1)
    PORTD |= 0b00000100;
  else if (pulsecountl == 2)
    PORTD &= 0b11111011;

  //right motor pulse calculations
  pulsecountr++;
  if (pulsecountr > pulsememr)
  {
    pulsecountr = 0;
    pulsememr = stepr;
    if (pulsememr < 0)
    {
      PORTD |= 0b00100000;
      //PORTD &= 0b11011111;
      pulsememr *= -1;
    }
    else
      PORTD &= 0b11011111;
    //else PORTD |= 0b00100000;
  }
  else if (pulsecountr == 1)
    PORTD |= 0b00010000;
  else if (pulsecountr == 2)
    PORTD &= 0b11101111;
}
