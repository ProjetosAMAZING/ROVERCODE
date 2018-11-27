
#include <Arduino.h>
#include "atomic.h"

#include <SPI.h>
#include "rf69_module.h"
#include "QueueList.h"
#include "gps_data.h"
//#include "SkyTraqNmeaParser.h"
//#include <SoftwareSerial.h>
#include "carControl.h"


void writeRegister(uint8_t address,uint8_t value);
uint8_t readRegister(uint8_t address);
void setMode(uint8_t state);
void isr0(void);
void waiToReceive (void);



//static const float x3 =  -8.660310006172601;
//static const float x4 =  -8.659699462693688;
//static const float y2 = 40.634400217149938;
//static const float y1 = 40.633934334441228;
uint32_t lat;
uint32_t lg;
uint8_t h;
uint8_t m;
uint8_t s;
uint8_t gpsst;
uint8_t gpspeed;
uint8_t gpsQual;
uint8_t speed;

int sum_packet = 0;
uint8_t sum_sent = 0;
uint8_t contador = 0;
uint8_t npackets = 0;

static QueueList <int> velocityQueue;
static QueueList <uint8_t> carState;

unsigned long ttime;
char msgACK [] = "OK";

int numb_packet_rcv;
int numb_packet_sent;

uint8_t i;
uint8_t rf69_state = M_STDBY;
static volatile uint8_t payload = 0;
volatile uint8_t packetSent = 0;

void configSPI(void){
        SPI.setDataMode(SPI_MODE0);
        SPI.setClockDivider(SPI_CLOCK_DIV16);
        SPI.setBitOrder(MSBFIRST);
        SPI.begin();

        pinMode(slavePin, OUTPUT);
        digitalWrite(slavePin, HIGH);

}

void configModule(void)
{

        writeRegister(RegOpMode,opMode_STDBY); // Sequencer ON - - Listen off - Mode stand by
        setMode(M_STDBY);
        writeRegister(RegDataModul, packetMode | FSK | shapeNONE); // PACKET-MODE - FSK - NO SHAPPING

        writeRegister(RegBitrateMsb,0x02);
        writeRegister(RegBitrateLsb, 0x40); // bitRate = 4.8kbs - defalt ver tabela pagina 20. 0x1A 0x0B

        writeRegister(RegFdevMsb, 0x03); //35Khz -- depois alterar  = Fdev/Fstep // altertar
        writeRegister(RegFdevLsb, 0x33);

        writeRegister(RegFrMsb,Freq_MSB); // Colucar a frequencia a 433Mhz/434Mhz
        writeRegister(RegFrMid,Freq_MID);
        writeRegister(RegFrLsb,Freq_LSB);

//set OutputPower
        writeRegister(RegPaLevel,  PA2_ON | PA1_ON | 0x1F);//13dBm
        writeRegister(0x58,0x2D);

//writeRegister(RegOcp,ocpOFF |0b1010); //default value;

        writeRegister(RegRxBw, RXBW_DCCFREQ_010| RXBW_MANT_16 | RXBW_EXP_2);

//interrupts;
        pinMode(7, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(7), isr0, RISING);
        interrupts();
        writeRegister(RegDioMapping2, CLK_OUT_OFF);
        writeRegister(RegDioMapping1, DIO0_01);

        writeRegister(RegIrqFlags2,FIFO_OVERRUN);

        writeRegister(RegRssiThresh, 220);
        writeRegister(0x18,0x04); //input impedance 50 ohms
        writeRegister(RegSyncConfig,SyncON |AUTO_FIFOFILL| SyncSize_2 );
        writeRegister(RegSyncValue1,0x2D);
        writeRegister(RegSyncValue2,100);

        writeRegister(RegPacketConfig1,0x80 | NONE_CODE | crcON | crc_autoclear_ON);// ver melhor isto

        writeRegister(RegPayloadLength,64);

        writeRegister(RegFifoThresh,0x80 | 0x0F );
        writeRegister(RegPacketConfig2,AUTORXRESTART_ON |AES_OFF);

        //  Serial.println(readRegister(0x10));// gpSatellites
        //  Serial.println(readRegister(RegIrqFlags1));
          //Serial.print(readRegister(0x01));
}

void  resetPackets(){
        numb_packet_rcv = 0;
        numb_packet_sent = 0;
}
uint8_t checkModule(void)
{  //Serial.print(readRegister(RegIrqFlags1)&&MODEREADY);
        if(readRegister(RegIrqFlags1)&&MODEREADY != 0)
                return 1;
        else
                return 0;

}

void waiToReceive (void)
{
        setMode(M_STDBY);
        payload = 0;
        // Serial.println("start Receiving");
        writeRegister(RegDioMapping1, DIO0_01);
        setMode(M_RX);
}

uint8_t receiveDone()
{
        if(rf69_state == M_RX && payload== 1)
        {
                setMode(M_STDBY);
                payload=0;
                //  Serial.print("recebi !");
                return 1;
        }
        else if(rf69_state == M_RX)
                return 0;
        else
        {
                waiToReceive();
                return 0;
        }
}

void readtoFifo()
{
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
                digitalWrite(8, LOW);
                SPI.transfer(0);
                //Serial.print(SPI.transfer(0));
                uint8_t tempp = SPI.transfer(0);
                numb_packet_rcv = SPI.transfer(0)<<7;
                numb_packet_rcv |= SPI.transfer(0);

                //Serial.print("packet rcv: ");
                //Serial.println(numb_packet_rcv);
                if(numb_packet_rcv == 0)
                {
                        resetPackets();
                }
                velocityQueue.push((uint8_t)SPI.transfer(0));
                carState.push((uint8_t)SPI.transfer(0));

                digitalWrite(8, HIGH);
        }
        writeRegister(RegIrqFlags2,FIFO_OVERRUN);
}


void readMessage(uint8_t& vel, uint8_t & carS, int &nprcv)
{
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
                digitalWrite(8, LOW);
                SPI.transfer(0);
                //  Serial.print(SPI.transfer(0));
                uint8_t temp2 = SPI.transfer(0);
                nprcv = SPI.transfer(0)<<7;
                nprcv |= SPI.transfer(0);

                //Serial.println(numb_packet_rcv);
                /* Serial.print("packet loss: ");
                   Serial.println(numb_packet_sent/numb_packet_rcv);
                 */
                vel = (uint8_t)SPI.transfer(0);
                carS = (uint8_t)SPI.transfer(0);

                digitalWrite(8, HIGH);
        }
        writeRegister(RegIrqFlags2,FIFO_OVERRUN);
}

void sendMessage(uint8_t vel, uint8_t car_state)
{
        setMode(M_STDBY);

        writeRegister(RegDioMapping1, DIO0_00);

        //Serial.print("A enviar velocidade: ");
        //Serial.print(vel_pretendida);
        //Serial.print(" seq_frame: ");
        //Serial.println(SEQ_FRAME);
        getPosition(lat,lg,h,m,s,gpsst,gpspeed);
        /*  Serial.print(lat);
           Serial.print(" , ");
           Serial.print(((lat&0x000000FF)<<24));
           Serial.print(" , ");
           Serial.println(lg);*/


        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
                digitalWrite(slavePin,LOW);
                SPI.transfer(RegFifo | SPI_WRITE);
                SPI.transfer(19);
                SPI.transfer((numb_packet_sent&&0xFF00)>>7);
                SPI.transfer((numb_packet_sent&&0xFF));
                //  Serial.print("sent: ");
                //  Serial.println(numb_packet_sent);
                //tamanho
                SPI.transfer(vel);
                SPI.transfer(car_state);

                SPI.transfer(lat&0x00FF);
                SPI.transfer((lat&0x00FF00)>> 8);
                SPI.transfer((lat&0x00FF0000)>>16);
                SPI.transfer((lat&0x00FF000000)>>24);
                SPI.transfer(lg&0x00FF);
                SPI.transfer((lg&0x00FF00)>> 8);
                SPI.transfer((lg&0x00FF0000)>>16);
                SPI.transfer((lg&0x00FF000000)>>24);
                SPI.transfer(h);
                SPI.transfer(m);
                SPI.transfer(s);
                SPI.transfer(gpspeed);
                SPI.transfer(gpsst);
                SPI.transfer(25);


                digitalWrite(slavePin, HIGH);
        }


        setMode(M_TX);
        while(packetSent == 0) ;
        numb_packet_sent++;

        packetSent = 0;
        writeRegister(RegIrqFlags2,FIFO_OVERRUN);


        return;
}


void setMode(uint8_t state)
{
        if(state != rf69_state)
        {
                uint8_t value = readRegister(RegOpMode);
                value &= MODE_BITS;

                switch (state) {
                case M_STDBY:
                        value |= opMode_STDBY;
                        //  Serial.println("STBY");
                        break;
                case M_RX:
                        value |= opMode_RX;
                        //NORMAL MODE WITH RX
                        writeRegister(RegTestPa1,0x55);
                        writeRegister(RegTestPa2,0x70);
                        //   Serial.println("RX");
                        break;
                case M_TX:
                        value |= opMode_TX;
                        //MAX OUTPUT POWER 20DBM
                        writeRegister(RegTestPa1,0x5D);
                        writeRegister(RegTestPa2,0x7C);
                        // Serial.println("TX");
                        break;
                default:
                        Serial.println("Nhient");
                        break;
                }

                writeRegister(RegOpMode,value);
                while(readRegister(RegIrqFlags1)&&MODEREADY == 0) ;


                rf69_state = state;
                return;
        }
        else
                return;
}


void checkMessages(uint8_t &std, uint8_t &velc, uint8_t &empty)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
        //uint8_t vp=0;
        sum_packet = 0;
        contador = 0;
        sum_sent = 0;

        if(velocityQueue.isEmpty() || carState.isEmpty())
        {
                std = 3;
                velc = 0;
                empty = 1;
        }
        else
        {
                empty = 0;
                npackets = velocityQueue.count();
                //Serial.print(npackets);
                while(!velocityQueue.isEmpty())
                {
                        sum_packet += velocityQueue.pop();
                        sum_sent += carState.pop();
                }
                //    Serial.println(sum_packet);

                velc = sum_packet/(npackets);
                //  Serial.print(velc);
                  //Serial.print(" ");

                if(velc > 30 || velc <0)
                        velc = 0;
                std = sum_sent/(npackets);

                //Serial.println(std);
                if(std <0 || std >1)
                        std = 3;


        }
      }
        return;

}
void isr0(void){
        if(rf69_state == M_RX)
        {
                payload = 1;
    //            Serial.print("R");
        }
        else if(rf69_state == M_TX)
        {
                packetSent = 1;
                //Serial.println("T");

        }

}

void writeRegister(uint8_t address, uint8_t value){
        digitalWrite(slavePin, LOW);
        SPI.transfer(SPI_WRITE | address);
        SPI.transfer(value);
        digitalWrite(slavePin,HIGH);
}

uint8_t readRegister(uint8_t address){
        digitalWrite(slavePin, LOW);
        SPI.transfer(address & SPI_READ);
        uint8_t value = SPI.transfer(RegFifo);
        digitalWrite(slavePin,HIGH);
        return value;
}
