#include <math.h>
#include <Wire.h>

// Class definitions
#define INITIAL_E 0
#define INITIAL_PID_I 0
class PID
{
public:
  PID(
      const double& _k_p,
      const double& _k_i,
      const double& _k_d,
      const double& initial_e = INITIAL_E,
      const double& initial_pid_i = INITIAL_PID_I) :
      k_p(_k_p),
      k_i(_k_i),
      k_d(_k_d),
      last_e(initial_e),
      last_pid_i(initial_pid_i) {}

  double reset(const double& _last_e = INITIAL_E, const double& _last_pid_i = INITIAL_PID_I)
  {
    last_e = _last_e;
    last_pid_i = _last_pid_i;
  }

  double update(const double& r, const double& pv, const double& dt)
  {
    const double& e = r - pv;
    const double& pid_p = e;
    const double& pid_i = last_pid_i + e * dt;
    const double& pid_d = (e - last_e) / dt;
    const double& u = k_p * pid_p + k_i * pid_i + k_d * pid_d;

    last_e = e;
    last_pid_i = pid_i;

    return u;
  }

private:
  const double k_p, k_i, k_d;
  double last_e, last_pid_i;
};

// CONFIG
// comment this out to stop interrupts. This makes loop be called as fast as
// possible.
#define NOINTERRUPT

// Number of samples taken to calibrate the gyroscope
const int n_samples = 200;

// If the angle offset from center is greater than this, then start balancing
const double balance_window = 0.5;

// The length of time to wait before updating pid (essentially delta t, in
// microseconds)
const int loop_time_length = 4000;
const double loop_time_length_s = loop_time_length / 1.0e6;

// pid config
const double spr = 0.0; // set point for right wheel
const double spl = 0.0; // set point for left wheel
const double k_p = 15.0;
const double k_i = 1.5;
const double k_d = 10.0;
PID pidl(k_p, k_i, k_d);
PID pidr(k_p, k_i, k_d);
// END OF CONFIG

// REGISTER PORTS
#define lsm 0x6B
#define LED_PIN 13
#define BAUD_RATE 9600
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
double gangle, adjuster;
byte balance;
int stepr = 0,
  pulsecountr = 0,
  pulsememr = 0;
int stepl = 0,
  pulsecountl = 0,
  pulsememl = 0;
double output;
unsigned long loop_time;

void setup()
{
  Serial.begin(BAUD_RATE);
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

  double calibration = 0.0;
  for (int i = 0; i < n_samples; i++)
  {
    if (i % 15 == 0) {
      toggleBlink();
    }
    calibration += read_gy();
    delayMicroseconds(3000);
  }
  y_g_calibration = calibration / n_samples;
  const double &gy = read_gy();
  pidr.reset(gy - y_g_calibration);
  pidl.reset(gy - y_g_calibration);
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

double read_ax() {
  // read from accelerometer
  Wire.beginTransmission(lsm);
  Wire.write(axl);
  Wire.endTransmission();
  Wire.requestFrom(lsm, 2);
  const int& raxl = Wire.read();
  const int& raxh = Wire.read();
  const int ax_value = (raxh << 8) | raxl;

  double ax = -ax_value/8200.0;
  if (ax > 1.0) {
    ax = 1.0;
  }
  else if (ax < -1.0)
  {
    ax = -1.0;
  }
  return ax;
}

double read_gy() {
  // read from gyroscope
  Wire.beginTransmission(lsm);
  Wire.write(gyl);
  Wire.endTransmission();
  Wire.requestFrom(lsm, 2);
  const int rgyl = Wire.read();
  const int rgyh = Wire.read();
  const int gy_value = (rgyh << 8) | rgyl;
  return gy_value;
}

// Doesn't modify global output since parameter shadows it.
double adjust_output(double output)
{
  // handle output too large
  if (output > 500)
    output = 500;
  else if (output < -500)
    output = -500;

  // handle output too small
  if (output < 10 && output > -10)
  {
    output = 0;
  }

  // stop balancing if gangle too large
  if (gangle < -30 || gangle > 30 || balance == 0)
  {
    output = 0;
    pidr.reset(gangle);
    pidl.reset(gangle);
    balance = 0;
    adjuster = 0;
  }

  // update adjuster
  if (output < 0)
    adjuster += 0.015;
  if (output > 0)
    adjuster -= 0.015;

  /*
  Serial.println(output);
  Serial.println();
  delay(20);
  //*/
}

/**
 * calculat the step speed of the motor. E.g., motr or motl. Smaller is faster.
 * Pass output as the argument. If speed is too high s in (-5, 5), then the
 * speed is set to zero since the motors wouldn't be spinning anyways.
 */
int calc_speed(double motd)
{
  int mot;
  // adjust speed
  if (motd > 0)
  {
    motd = 203 - 2750 / (motd + 9);
    motd = 200 - motd;
    mot = motd; // cast to int
    if (mot < 5)
      mot = 0;
  }
  else if (motd < 0)
  {
    motd = -203 - 2750 / (motd - 9);
    motd = -200 - motd;
    mot = motd; // cast to int
    if (-5 < mot)
      mot = 0;
  }
  else
  {
    mot = 0;
  }

  return mot;
}

inline void toggleBlink() {
  const auto &currentValue = digitalRead(LED_PIN);
  digitalWrite(LED_PIN, !currentValue);
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

  const double ax = read_ax();
  const double gy = read_gy() - y_g_calibration;

  /*
  Serial.println(ax);
  Serial.println(gy);
  Serial.println();
  delay(20);
  //*/

  // do pid calculation
  double aangle = asin(ax) * TO_DEG;
  if (balance == 0 && aangle > -balance_window && aangle < balance_window)
  {
    gangle = aangle;
    balance = 1;
  }
  gangle += gy * 0.000031;
  gangle = gangle * 0.9996 + aangle * 0.0004;

  double pv = gangle - adjuster;

  // TODO: find a way to get rid of this
  if (output > 5 || output < -5)
  {
    pv += output * 0.0045;
  }

  double outputl = pidl.update(spl, pv, loop_time_length_s);
  outputl = adjust_output(outputl);
  const int& speedl = calc_speed(outputl);

  double outputr = pidr.update(spr, pv, loop_time_length_s);
  outputr = adjust_output(outputr);
  const int& speedr = calc_speed(outputr);

  // TODO: if r and l have different set points, this becomes corrupt since they
  // both share this value. We have to eliminate the output global, which means
  // getting rid of the if statement a few lines above.
  output = outputr;

  // update speeds
  noInterrupts(); // prevents (unlikely) concurrency issues
  stepl = speedl;
  stepr = speedr;
  interrupts();
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
