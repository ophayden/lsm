// r = 204.2mm

#include <Wire.h>
#include "registers.h"

int aoffset = 0;
int goffset = 0;
float pgain = 15;
float igain = 1.5;
float dgain = 10;
float vtop;
float aangle, gangle, tangle, adjuster;
int ax, az, tp, gx, gy, gz;
int raxl, raxh, rgyl, rgyh;

int axcal, azcal, gxcal, gycal, gzcal;

byte balance;

float pid, pid_p, pid_i, pid_d;

int motr, stepr, pulsecountr, pulsememr;
int motl, stepl, pulsecountl, pulsememl;
int calcounter;
float error, sp, gin, output, pidde;
float outputl, outputr;

unsigned long looptime;
int n_samples = 200;
void setup(){
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;

  DDRD |= 0b00111100;
  DDRB |= 0b00100000;


  Wire.beginTransmission(lsm);
  Wire.write(ctrl1g);
  //Wire.write(0b01100001);
  Wire.write(0b10100010);
  Wire.endTransmission();
  

 /* Wire.beginTransmission(lsm);
  Wire.write(ctrl1g);
  Wire.write(0b01100001);
  Wire.endTransmission();
 
*/
 

  for(calcounter = 0; calcounter < n_samples; calcounter++){      
    if(calcounter % 15 == 0)digitalWrite(13, !digitalRead(13));     
    Wire.beginTransmission(lsm);              
    Wire.write(0x1A);                          
    Wire.endTransmission();                   
    Wire.requestFrom(lsm, 6); 
    
    /*gxcal += Wire.read() >> 8 | Wire.read();      
    gycal += Wire.read() >> 8 | Wire.read(); 
    gzcal += Wire.read() >> 8 | Wire.read();          
*/
           
    gxcal += Wire.read() | (Wire.read() << 8);      
    gycal += Wire.read() | (Wire.read() << 8); 
    gzcal += Wire.read() | (Wire.read() << 8);          
    delayMicroseconds(3000);                                 
  }
  gxcal /= n_samples;                                 
  gycal /= n_samples;                                     
  gzcal /= n_samples;
  looptime = micros() + 4000;        

//  /*
  TCNT2 = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= 0b00000010;
  TCCR2B |= 0b00000010;
  TIMSK2 |= 0b00000010;
  OCR2A = 39;
  //sei();
//*/ 

}
void loop(){
  //adj = adjustment();
  Wire.beginTransmission(lsm);
  Wire.write(0x28);
  Wire.endTransmission();
  Wire.requestFrom(lsm, 2);
  //ax = Wire.read() >> 8 | Wire.read();
  raxl = Wire.read();
  raxh = Wire.read();
  ax = raxh << 8 | raxl;
  ax *= -1;
  ///ax = Wire.read() | (Wire.read() << 8);
  
  //ay = Wire.read() | Wire.read() <<8;
  //az = Wire.read() | Wire.read() <<8;
  //tp = Wire.read() | Wire.read() <<8;
  //gx = Wire.read() | Wire.read() <<8;
 //gy = Wire.read() | Wire.read() <<8;
  //gz = Wire.read() | Wire.read() <<8;
  
  Wire.beginTransmission(lsm);
  Wire.write(0x1A);
  Wire.endTransmission();
  Wire.requestFrom(lsm, 2);

  //gy = Wire.read() >> 8 | Wire.read();

  rgyl = Wire.read();
  rgyh = Wire.read();
  gy = rgyh << 8 | rgyl;
  
  //gy = Wire.read() | ( Wire.read() << 8);
 

  
  //ax -= 500;
  //ax *= -1;
  if(ax > 8200)ax = 8200;
  if(ax < -8200)ax = -8200;
  //if(ax == 8200 || ax == -8200)balance = 0;
  //gx -= gxcal;
  gy -= gycal;
  //gy *= -1;
  //gz -= gzcal;
  /*
  Serial.println(ax);
  Serial.println(gy);
  Serial.println();
  delay(20);
*/
  aangle = asin((float)ax/8200)*57.296;
  
  if(balance == 0 && aangle > -0.5 && aangle < 0.5){
      gangle = aangle;
      balance = 1;
    }

  
  gangle += gy * 0.000031;
  gangle = gangle * 0.9996 + aangle * 0.0004;
  //gangle = aangle;

  

  
  //pid
  error = gangle - adjuster - sp;
  if(output > 5 || output < -5){
    error += output * 0.0045;
  }
  pid_i += igain * error; 
  //if(pid_i > 400)pid_i = 400;
  //else if(pid_i < -400)pid_i = -400;
  //p = error  
  pid_p = pgain * error;
  pid_d = dgain * (error - pidde);
  output = pid_p + pid_i + dgain * (error - pidde);

  if(output > 500)output = 500;
  else if(output < -500)output = -500;

  pidde = error ;

  if(output < 10 && output > -10){
    output = 0;
  }

  if(gangle < -30 || gangle > 30 || balance == 0){
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
*/
  if(sp == 0 && output < 0) adjuster += 0.015;
  if(sp == 0 && output > 0)adjuster -= 0.015;

  if(outputl > 0)outputl = 203 - (1/(outputl + 9)) * 2750;
  else if(outputl < 0)outputl = -203 - (1/(outputl - 9)) * 2750;

  if(outputr > 0)outputr = 203 - (1/(outputr + 9)) * 2750;
  else if(outputr < 0)outputr = -203 - (1/(outputr - 9)) * 2750;

  if(outputl > 0)motl = 200 - outputl;
  else if(outputl < 0)motl = -200 - outputl;
  else motl = 0;

  if(outputr > 0)motr = 200 - outputr;
  else if(outputr < 0)motr = -200 - outputr;
  else motr = 0;
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
 // PORTD |= 0b10000000;
  //PORTD &= 0b01111111;
  while(looptime > micros());
  looptime += 2000;

}
/*int adjustment(){
    int read = analogRead(A0);
    int mapread = map(read, 0, 1023, 0, 1400);
    return mapread;
}*/
ISR(TIMER2_COMPA_vect){
pulsecountl ++;
  if(pulsecountl > pulsememl){  
    pulsecountl = 0;
    pulsememl = stepl;
    if(pulsememl < 0){ 
      PORTD &= 0b11110111;
      //PORTD |= 0b00001000; 
      pulsememl *= -1;   
    }
    else PORTD |= 0b00001000; 
    //else PORTD &= 0b11110111;
  }
  else if(pulsecountl == 1)PORTD |= 0b00000100;
  else if(pulsecountl == 2)PORTD &= 0b11111011;
  
  //right motor pulse calculations
  pulsecountr ++;
  if(pulsecountr > pulsememr){
    pulsecountr = 0;
    pulsememr = stepr;
    if(pulsememr < 0){ 
      PORTD |= 0b00100000;
      //PORTD &= 0b11011111;
      pulsememr *= -1; 
    }
    else PORTD &= 0b11011111; 
    //else PORTD |= 0b00100000;
  }
  else if(pulsecountr == 1)PORTD |= 0b00010000;
  else if(pulsecountr == 2)PORTD &= 0b11101111;
}


