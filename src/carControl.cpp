#include <Arduino.h>
#include "carControl.h"
#include <SPI.h>
#include "atomic.h"
#include "TimerOne.h"
#include "rf69_module.h"
#include "gps_data.h"


void setVoltage(int value);
void resetPID(void);
void security(void);


byte header  = 0b00110000;
byte data;


uint8_t Carsentido =1;

static uint8_t kickAvailable = 1;
uint8_t flagKick = 1;
float errSum =0;
float dErr;
float lastErr = 0;
float lastTime = 0;
double Input,Output;

unsigned int out = 0;


const float kd = 0.1; // 8//8/0.5
const float kp = 1.8; // 30//30//1.5
const float ki = 0.015; // 0.08//0.08/0.08

const float kpd =3;
const float kdd =1 ;
const float kid = 0.020;


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
        if(flagKick)
        {
                digitalWrite(12, HIGH);
        }
        else
        {
                digitalWrite(12, LOW);
        }

        if(flagKick == 1)
                flagKick = 0;
        else
                flagKick = 1;
}

uint8_t PID(uint8_t &v)
{
        Input = ((analogRead(A0)*5.0/1024)*(292/47));
        //Input = gpsSpeed();
        //  Serial.println(Input);
        float error = v - Input;

        unsigned long now = millis();
        float timeChange = (now - lastTime);
        //  Serial.println(timeChange);
        errSum += (error * timeChange);
        dErr = (error - lastErr) / timeChange;
        Output = kp * error + ki * errSum + kd * dErr;


        Output = (Output) + 1120;



        if(Output > 2600)
                Output = 2600;
        else if(Output<1120)
                Output = 880;

        out = (int) Output;


        setVoltage(out);

        lastErr = error;
        lastTime = now;

        if((v-gpsSpeed()<2 || v <=3) && GPState() >= 4)
        {
                return 1;
        }
        else
                return 0;
}
uint8_t DynamicPID(uint8_t &v)
{

  //Input = ((analogRead(A0)*5.0/1024)*(292/47));
  Input = gpsSpeed();
  //  Serial.println(Input);
  float error = v - Input;
  if(error <-1.5)
  {
    setVoltage(880);
  }
  else{
  unsigned long now = millis();
  float timeChange = (now - lastTime);
  //  Serial.println(timeChange);
  errSum += (error * timeChange);
  dErr = (error - lastErr) / timeChange;
  Output = kpd * error + kid * errSum + kdd * dErr;


  Output = (Output) + 1120;



  if(Output > 2600)
          Output = 2600;
  else if(Output<1120)
          Output = 880;

  out = (int) Output;


  setVoltage(out);

  lastErr = error;
  lastTime = now;
}
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
        else if (sent == 1) {
                setFoward();

        }
        resetPID();
}

void startCar(){
        resetPID();

}

uint8_t readVelocity()
{
        return uint8_t((analogRead(A0)*5.0/1024)*(292/47));
}
uint8_t waitToStop()
{

        if((analogRead(A0)*5.0/1024.0) <= 0.07)
                return 1;
        else{
                setVoltage(880);
                return 0;
              }
}
uint8_t breakCar()
{
        setVoltage(880);
        return 1;
}


uint8_t isParked(void)
{
        if((analogRead(A0)*5.0/1024.0)<= 0.04)
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
        // Pin 12 onda quadrada para o watchdog
        pinMode(12,OUTPUT);
        digitalWrite(12, HIGH);
        pinMode(2, OUTPUT);
        pinMode(2, INPUT_PULLUP);
        //attachInterrupt(digitalPinToInterrupt(2), security, RISING);
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
        setVoltage(880);

        return 1;
}

uint8_t bateryState()
{
    return (  (((analogRead(A1)/1024)*5)*13)/65)*100;
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
  noInterrupts();
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
      interrupts();
}
