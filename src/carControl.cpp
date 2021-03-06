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
double batt = 0;

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
        if((analogRead(A2)*5/1024)>=4.90)
                return 1;
        else
                return 0;
}

void kickDog(uint8_t st) // módulo de segurança
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
        Input = ((analogRead(A0)*5.0/1024)*(292/47)); // Leitura do filtro passa baixo
        //Input = gpsSpeed();
        //  Serial.println(Input);
        float error = v - Input; // erro V pretendido - V lido

        unsigned long now = millis();
        float timeChange = (now - lastTime);
        //  Serial.println(timeChange);
        errSum += (error * timeChange);
        dErr = (error - lastErr) / timeChange;
        Output = kp * error + ki * errSum + kd * dErr;


        Output = (Output) + 1120; // janela do PID vai de 1120 até 2600



        if(Output > 2600) // Wind-up
                Output = 2600;
        else if(Output<1120)
                Output = 720;

        out = (int) Output;


        setVoltage(out); // colocar na DAC

        lastErr = error;
        lastTime = now;

        if((v-gpsSpeed()<2 || v <=3) && GPState() >= 4) // convergência para colocar para o PID com a leitura da velocidade apartir do módulo GPS-RTK
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
  if(error <-1.5) //Tentativa de melhoramento do controlo de velocidade - ainda não está aperfeiçoado corretamente.
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
          Output = 720;

  out = (int) Output;


  setVoltage(out);

  lastErr = error;
  lastTime = now;
}
  return 1;
}

uint8_t sentCar() // enviar o sentido do carro no momento
{
        return Carsentido;
}

void changeSent(uint8_t & sent){ // mudança do sentido
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

uint8_t readVelocity() // Ler velocidade filtro-passa baixo
{
        return uint8_t((analogRead(A0)*5.0/1024)*(292/47));
}
uint8_t waitToStop() // Esperar até que a velocidade lida por parte do filtro passa baixo seja perto de zero.
{

        if((analogRead(A0)*5.0/1024.0) <= 0.07)
                return 1;
        else{
                setVoltage(720);
                return 0;
              }
}
uint8_t breakCar() // colocar 0 potência no carro
{
        setVoltage(720);
        return 1;
}


uint8_t isParked(void) // verificar se o carro está em repouso
{
        if((analogRead(A0)*5.0/1024.0)<= 0.04)
        {
                return 1;
        }
        else
                return 0;
}


void resetPID(void) // reset parametro PID
{

        errSum =0;
        lastErr = 0;
        lastTime = millis();
}


uint8_t configCar() //configuração do pinos arduino para o carro
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

        //pinMode(A2, INPUT);// Leitura dos valores bateria

        setFoward();

        pinMode(A0,INPUT);//Leitura da velocidade
        pinMode(A1,INPUT); // Leitura dos valores bateria
        pinMode(A2,INPUT);
        //excitação do acelarador
        setVoltage(720);

        return 1;
}

uint8_t bateryState() // Leitura das baterias
{
    batt = analogRead(A1);
    batt = (batt/1024.0)*5.0*13.57616;

    return (uint8_t)batt;
}


void security(void) // sem efeito
{
        kickAvailable = 0;
        Serial.println("here");
        detachInterrupt(digitalPinToInterrupt(2));
}

void setFoward(void) // configuração para colocar o motor a andar para frente
{
        digitalWrite(MotorWay,HIGH);
        digitalWrite(ADCWay,HIGH);
        Carsentido = 1;
}



void setReverse(void) // configuração para a rotação inversa do motor
{

        digitalWrite(MotorWay,LOW);
        digitalWrite(ADCWay,LOW);
        Carsentido = 0;
}

void setVoltage(int value) // Escrita nos valores da DAC
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
