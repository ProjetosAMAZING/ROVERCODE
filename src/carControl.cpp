#include <Arduino.h>
#include "carControl.h"
#include <SPI.h>
#include "atomic.h"
#include "TimerOne.h"

void setVoltage(int value);
void resetPID(void);
void security(void);


byte header  = 0b00110000;
byte data;


uint8_t Carsentido =1;

static uint8_t  kickAvailable = 1;

float errSum =0;
float lastErr = 0;
float lastTime = 0;
double Input,Output;

unsigned int out = 0;

const float kp = 0.08;
const float kd = 0.8;
const float ki = 0.00008;

int value = 0;
int lastV = 0;



uint8_t checkInitVoltage(void)
{
    if((analogRead(A1)*5/1024)>=0.90 && ((analogRead(A2)*5/1024)>=4.90))
      return 1;
    else
      return 0;
}

void kickDog(uint8_t st)
{

  if(kickAvailable)
  {
    if(st == 1)
      digitalWrite(12, HIGH);
    else
      digitalWrite(12, LOW);
  }



}
uint8_t PID(uint8_t &v)
{

  Input = (analogRead(A0)*5.0/1024)*100/4.90;

  float error = v - Input;

  unsigned long now = millis();
  float timeChange = (now - lastTime);

  errSum += (error * timeChange);
  float dErr = (error - lastErr) / timeChange;
  Output = kp * error + ki * errSum + kd * dErr;


  Output = (2430-1100)*(Output/100) + 1100;



  if(Output > 2700)
    Output = 2700;
  else if(Output<1150)
    Output = 850;

  out = (int) Output;


  setVoltage(out);

  lastErr = error;
  lastTime = now;

  return 1;
}

uint8_t sentCar()
{
  return Carsentido;
}

void changeSent(uint8_t & sent){
  if(sent == 0)
  {
    setReverse();
  }
  else if (sent == 1){
    setFoward();

  }
}

void startCar(){
  resetPID();

}

uint8_t readVelocity()
{

  return ((analogRead(A0)*5.0/1024)*100/4.90);
}
uint8_t waitToStop()
{

  if((analogRead(A0)*5.0/1024.0) <= 0.07)
    return 1;
  else
    return 0;
}
uint8_t breakCar()
{

setVoltage(800);
return 1;
}


uint8_t isParked(void)
{


  if((analogRead(A0)*5.0/1024.0)<= 0.06)
  {
  return 1;
  }
  else
  return 0;
}


void resetPID(void)
{

  errSum =0;
  lastErr = 0;
  lastTime = millis();
}

uint8_t configCar()
{
  //security
  pinMode(12,OUTPUT);
  pinMode(2, OUTPUT);
 pinMode(2, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(2), security, RISING);
  /*SS DAC*/
  pinMode(SSDAC,OUTPUT);
  /*pinos afectam a direção*/

  pinMode(MotorWay,OUTPUT);
  pinMode(ADCWay,OUTPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  setFoward();

  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

//excitação do acelarador
  setVoltage(800);

  return 1;
}

void security(void)
{
  kickAvailable = 0;
    Serial.println("here");
    detachInterrupt(digitalPinToInterrupt(2));
}

void setFoward(void)
{
  digitalWrite(MotorWay,HIGH);
  digitalWrite(ADCWay,HIGH);
    Carsentido = 1;
}



void setReverse(void)
{

  digitalWrite(MotorWay,LOW);
  digitalWrite(ADCWay,LOW);
    Carsentido = 0;
}

void setVoltage(int value)
{

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    digitalWrite(9,LOW);
    data = highByte(value);

    data = 0x0F & data;
    data = header |data;

    SPI.transfer(data);
    data = lowByte(value);
    SPI.transfer(data);

    digitalWrite(9,HIGH);
  }

}
