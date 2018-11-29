
#include <Arduino.h>
#include <SPI.h>
#include "rf69_module.h"
#include "TimerOne.h"
#include "atomic.h"
#include "carcontrol.h"
#include "gps_data.h"


void initVariables(void);
void PIDTimer(void);





const int ledPin =  13;
int ledState = LOW;
long ttt = 0;

uint8_t volatile stateTimer =0; //falg of can do timer 100ms
unsigned long ttime_vel; // timer 1s

long time_GPS = 0;
uint8_t gps_conv =0;
//Variaveis Estado
uint8_t velocidade; // velocidade pretendida
uint8_t sent; // sentido pretendido
uint8_t car_state; //Estado actual do carro
uint8_t gps_count = 0;
int nRCV;
//uint8_t PairConclude = 0;
uint8_t empty;
uint8_t count_timeout = 0;
void setup() {
// Serial- Debug
        //Serial.begin(115200);
        //while(!Serial);
//Configurar SPI
        configSPI();

//Configurar o modulo RF
        configModule();
// Verificar se o modulo está bem ligado
        while(checkModule() == 0)
        {
                Serial.println("Module connections maybe wrong, cant communicate via SPI");
                delay(1000);
        }
        Serial.print("passei");
// Inicialazação das variaveis estado
        initVariables();
// Colocar o carro no estado excitado, preparado para iniciar
        configCar();
// Verificar se o carro está bem configurado.
        //checkInitVoltage();
// Check voltage ()
        //checkBattery();



//Start Timer, para a máquina de estados.
        Timer1.initialize(100000);         //Timer para a máquina de estados 100ms
        Timer1.attachInterrupt(PIDTimer);
        initGPS(); //inicializar as variaveis para a leitura do módulo GPS
}
void loop(){

        if(stateTimer == 1) //T=100ms entra na maquina de estados.
        {
                switch (car_state) { //kickDog every time to Low
                  kickDog(1);
                case 0x05: //PID pelo módulo GPS
                        DynamicPID(velocidade);
                        break;
                case 0x04: // Estado RUN
                           //  kickDog(0);
                        if(PID(velocidade)) //PID pelo filto-passa-baixo
                          car_state = 0x05;

                        break;
                case 0x03: // A espera que o carro pare - Trocar sentido se precisar - Estado transitivo entre RUN-PARK
                        if(waitToStop() == 1) // esperar que pare
                        {
                                //    kickDog(0); // Se nao parar em 7s desliga o motor.
                                changeSent(sent);// Mudar de Sentido se necessário
                                startCar(); // Coloca no estado excitado again.
                                while((millis()-ttt)<=3000) ; //Espera 3S para prosseguir só por segurança
                                car_state =0x04; // Vai para o estado RUN
                                resetPID();
                        }
                        else
                                ttt = millis(); // tempo que nao esta parado
                        break;

                case 0x00: //Estado inicial. Supostamente parado
                        if(isParked())
                        {
                                //  kickDog(0); // se nao estiver parado nao faz kick no watchdog.
                        }
                        break;
                case 0x06: // Este estado não é utilizado, objetivo: Esperar pela convergência do sistema GPS antes de iniciar ! basta colocar o estado inicial com o valor 0x06.
                      if(GPState()>=4)
                        car_state = 0x00;
                      else{
                        breakCar();
                      }
                      break;
                }

                stateTimer = 0; // reset a flag time 100ms
        }

        else if(stateTimer == 0) // Se nao estiver na maquina de estado vai ver as mensagens
        {

                //  kickDog(1); // Pin 12 sempre a HIGH
                if(receiveDone() == 1) //Ve se recebeu alguma coisa e coloca num fifo
                {
                        //  Serial.print("RX");
                        readtoFifo();
                        sendMessage(readVelocity(),car_state); // Responde com os dados da telemetria do carro, como ACK
                        waiToReceive(); // Coloca o modulo RF em função RX
                }

                else if((millis() - ttime_vel)>= 1000) // Em 1s em 1s vai ler o fifo e mudar o estados (velocidade,Carestado)
                {
                    //Serial1.flush();

                        checkMessages(sent, velocidade,empty); // leitura do fifo a velocidade vem dentro de parametros 0-30 e sentido 0 - para frente 1 - para tras
                         kickDog(0);
                    if(car_state != 0x06 ){ //apenas se colocar o estado 0x06 como inicial, ignora as mensagens recebidas pelo utilizador
                         if(velocidade == 0 || empty == 1 || sent >1)  //vel = 0 ou sentido diferente do desejado (garantia) estado inicial PARK
                        {
                                breakCar(); // Parar o carro
                                car_state = 0x00; //Mudar o estado actual para PARK
                        }
                        else{
                                switch (sent)
                                { //se velocidade for != 0 entao analisar o sentido

                                case 0x00: // se o sentido do utilizador for 0
                                        if(car_state == 0x00)
                                                car_state = 0x03; // garantia que esta parado, e mudar o sentido se a situaçao anterior foi diferente
                                        if(sentCar() != sent && (car_state == 0x04 || car_state == 0x05)) // Se o carro esta a andar e vai mudar de sentido por estado 3 (transiente)
                                        {
                                                breakCar(); // Travar o carro
                                                car_state = 0x03; // Estado para esperar enquanto o carro nao chega ao estado estacionario
                                        }
                                        break;
                                case 0x01: // mesma coisa mas se o sentido for para o sentido contrario
                                        if(car_state == 0x00)
                                                car_state = 0x03;
                                        if(sentCar() != sent && (car_state == 0x04 || car_state == 0x05))
                                        {
                                                //Serial.println("HERE");
                                                breakCar();
                                                car_state = 0x03;
                                        }
                                        break;
                                }
                        }

                      }
                        // apensa prints de debug
                        //Serial.print("Estado: ");
                        //Serial.print(car_state);
                        //Serial.print(" Sentido: ");
                        //Serial.print(sent);
                        //Serial.print(" velocidade: ");
                        //Serial.println(velocidade);

                        if(ledState == 0) // Pin13 é o LED do Arduino que indica que o programa está a decorrer
                                digitalWrite(ledPin, HIGH);
                        else
                                digitalWrite(ledPin,LOW);
                        if(ledState == 0)
                                ledState = 1;
                        else
                                ledState =0;

                        ttime_vel = millis();
                }
        }

}




void PIDTimer(void) //isr PID TIMER
{
        stateTimer = 1;
}

void initVariables(void) //função para colocar as variaveis iniciais
{
        velocidade = 0; // 0 potencia
        sent = 3; //sem sentido especifico
        car_state = 0x00; // PARK // colocar para o estado 0x06 para esperar pelo sinal GPS se prentender
}
