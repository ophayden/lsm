#include <math.h>
#include <Wire.h>

// CONFIG
// comment this out to stop interrupts. This makes loop be called as fast as
// possible.
#define NOINTERRUPT

// Number of samples taken to calibrate the gyroscope
const int n_samples = 200;

// If the angle offset from center is greater than this, then start balancing
const double balance_window = 0.5;

const int loop_time_length = 4000;
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

const double pgain = 15;
const double igain = 1.5;
const double dgain = 10;
double gangle, adjuster;

byte balance;

double pid_i;

int motr = 0,
  stepr = 0,
  pulsecountr = 0,
  pulsememr = 0;

int motl = 0,
  stepl = 0,
  pulsecountl = 0,
  pulsememl = 0;

double err, output, pidde;
double outputl, outputr;

unsigned long loop_time;

void setup()
{
  Serial.begin(BOD_RATE);
  Wire.begin();

  // set i2c clock frequency
  TWBR = 12;

  // pinMode 2, 3, 4, 5 set to OUTPUT
  // pinMode 13 set to OUTPUT
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(13, OUTPUT);

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
  loop_time = micros() + 4000;

  // timer registers
  #ifndef NOINTERRUPT
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
  #ifdef NOINTERRUPT
  PORTD |= 0b10000000;
  PORTD &= 0b01111111;
  #else
  if (loop_time > micros())
    return;
  loop_time += loop_time_length;
  #endif

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
  err = gangle - adjuster;
  if (output > 5 || output < -5)
  {
    err += output * 0.0045;
  }
  pid_i += igain * err;
  //if(pid_i > 400)pid_i = 400;
  //else if(pid_i < -400)pid_i = -400;
  //p = err
  const double pid_p = pgain * err;
  output = pid_p + pid_i + dgain * (err - pidde);

  if (output > 500)
    output = 500;
  else if (output < -500)
    output = -500;

  pidde = err;

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

  if (output < 0)
    adjuster += 0.015;
  if (output > 0)
    adjuster -= 0.015;

  if (outputl > 0)
    outputl = 203 - 2750 / (outputl + 9);
  else if (outputl < 0)
    outputl = -203 - 2750 / (outputl - 9);

  if (outputr > 0)
    outputr = 203 - 2750 / (outputr + 9);
  else if (outputr < 0)
    outputr = -203 - 2750 / (outputr - 9);

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

  noInterrupts();
  stepl = motl;
  stepr = motr;
  interrupts();
}

inline void toggleBlink() {
  const auto &currentValue = digitalRead(LED_PIN);
  digitalWrite(LED_PIN, !currentValue);
}

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
