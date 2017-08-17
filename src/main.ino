
#include <Arduino.h>
#include <SPI.h>
#include "rf69_module.h"
#include "TimerOne.h"
#include "atomic.h"
#include "carcontrol.h"


void initVariables(void);
void PIDTimer(void);


const int ledPin =  13;
int ledState = LOW;
long ttt = 0;

uint8_t volatile stateTimer =0;
unsigned long ttime_vel;

uint8_t velocidade;
uint8_t sent;
uint8_t car_state;

void setup() {
  Serial.begin(115200);

//
  pinMode(12,OUTPUT);
  digitalWrite(12, HIGH);

//
  configSPI();
  configModule();
  while(checkModule() == 0)
  {
  //  Serial.println("Module connections maybe wrong, cant communicate via SPI");
    delay(1000);
  }

  initVariables();
  configCar();
  //sendMessage(0x88,0x88);
//  while(checkInitVoltage()!=1);
//checkBattery();

   // Timer 100ms
   Timer1.initialize(100000);
   Timer1.attachInterrupt(PIDTimer);
}
void loop(){

  if(stateTimer == 1)
  {


    switch (car_state) {
      case 0x04:
        kickDog(0);
        PID(velocidade);
        break;
      case 0x03:
        if(waitToStop() == 1)
         {
            kickDog(0);
            changeSent(sent);
            startCar();
            while((millis()-ttt)<=1000);
            car_state =0x04;
         }
         else
         ttt = millis();
         break;
        case 0x00:
          if(isParked())
          {
            kickDog(0);
          }
          break;

    }

      stateTimer = 0;
  }

  else if(stateTimer == 0)
  {
        kickDog(1);
       if(receiveDone() == 1)
     {
         readtoFifo();
         sendMessage(readVelocity(),car_state);
         waiToReceive();
      }

      else if((millis() - ttime_vel)>= 1000)
      {

        checkMessages(sent, velocidade);
        if(velocidade == 0 || (sent != 0 && sent !=1))
        {
          breakCar();
          car_state = 0x00;
        }
        else{
        switch (sent) {

          case 0x00:
            if(car_state == 0x00)
              car_state = 0x03;
            if(sentCar() != sent && car_state == 0x04)
            {
              breakCar();
              car_state = 0x03;
            //  Serial.println("HERE");
            }

              break;
          case 0x01:
          if(car_state == 0x00)
            car_state = 0x03;
            if(sentCar() != sent && car_state == 0x04)
            {
              //Serial.println("HERE");
              breakCar();
              car_state = 0x03;
            }
              break;
        }

      }
        Serial.print("Estado: ");
        Serial.print(car_state);
        Serial.print(" Sentido: ");
        Serial.print(sent);
        Serial.print(" velocidade: ");
        Serial.println(velocidade);


        ttime_vel = millis();
      }
}

}


void PIDTimer(void)
{
  stateTimer = 1;
}

void initVariables(void)
{
  velocidade = 0;
  sent = 3;
  car_state = 0x00;
}
