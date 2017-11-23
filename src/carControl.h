#ifndef carControl_H
#define carControl_H

#include <Arduino.h>

uint8_t DynamicPID(uint8_t &v);
uint8_t checkInitVoltage();
void resetPID2(void);
uint8_t configCar(void);
uint8_t PID(uint8_t &v);
uint8_t configCar();
void setFoward(void);
void setReverse(void);
uint8_t sentCar(void);
uint8_t breakCar(void);
uint8_t waitToStop(void);
void changeSent (uint8_t &sent);
uint8_t readVelocity();
void startCar(void);
uint8_t isParked(void);

void kickDog(uint8_t st);

#define SSDAC 9
#define MotorWay 4
#define ADCWay 3





#endif
