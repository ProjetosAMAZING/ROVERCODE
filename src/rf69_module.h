#ifndef rf69_module_H
#define rf69_module_H

#include <Arduino.h>




/*Registos necessários*/
#define RegFifo 0x00
#define RegOpMode 0x01
#define RegDataModul 0x02
#define RegBitrateMsb 0x03
#define RegBitrateLsb 0x04
#define RegFdevMsb  0x05
#define RegFdevLsb 0x06
#define   RegFrMsb      0x07
#define   RegFrMid      0x08
#define   RegFrLsb      0x09
#define   RegOsc1       0x0A
#define   RegAfcCtrl      0x0B
#define   RegListen1        0x0D
#define   RegListen2        0x0E
#define   RegListen3        0x0F
#define   RegVersion        0x10
#define   RegPaLevel        0x11
#define   RegPaRamp       0x12
#define   RegOcp          0x13
#define   RegLna          0x18
#define   RegRxBw       0x19
#define   RegAfcBw      0x1A
#define   RegOokPeak        0x1B
#define   RegOokAvg       0x1C
#define   RegOokFix       0x1D
#define   RegAfcFei       0x1E
#define   RegAfcMsb       0x1F
#define   RegAfcLsb       0x20
#define   RegFeiMsb       0x21
#define   RegFeiLsb       0x22
#define   RegRssiConfig     0x23
#define   RegRssiValue    0x24
#define   RegDioMapping1      0x25
#define   RegDioMapping2      0x26
#define   RegIrqFlags1    0x27
#define   RegIrqFlags2    0x28
#define   RegRssiThresh     0x29
#define   RegRxTimeout1     0x2A
#define   RegRxTimeout2     0x2B
#define   RegPreambleMsb      0x2C
#define   RegPreambleLsb      0x2D
#define   RegSyncConfig     0x2E
#define   RegSyncValue1   0x2F
#define   RegSyncValue2       0x30
#define   RegSyncValue3       0x31
#define   RegSyncValue4       0x32
#define   RegSyncValue5       0x33
#define   RegSyncValue6       0x34
#define   RegSyncValue7       0x35
#define   RegSyncValue8       0x36
#define   RegPacketConfig1  0x37
#define   RegPayloadLength  0x38
#define   RegNodeAdrs     0x39
#define   RegBroadcastAdrs  0x3A
#define   RegAutoModes    0x3B
#define   RegFifoThresh     0x3C
#define   RegPacketConfig2  0x3D
#define   RegTestPa1       0x5A
#define   RegTestPa2       0x5C
#define   RegTestLna      0x58


/*funçoes publicas*/
void configModule(void);
void configSPI(void);
uint8_t checkModule(void);
uint8_t receiveDone();
void startReceive(void);
void readtoFifo();
void sendMessage(uint8_t vel_pretendida, uint8_t carstate);
uint8_t waitToSend(void);
uint8_t SendACK();
uint8_t RecvACK();
void waiToReceive (void);
void readMessage(uint8_t &vel, uint8_t &carS, int &nprcv);
void checkMessages(uint8_t &std, uint8_t &velc,uint8_t &empty);
void  resetPackets();
int nPacketRcv(void);
//uint8_t GPState();
//uint8_t gpsSpeed(void);
/*flow control data*/

#define FRAME0 0x80
#define FRAME1 0x08
#define ACK0   0x40
#define ACK1   0x04
#define NACK   0x01



/*Valores imporatentes para configuração do modulo*/

#define seqOff 0x80;

#define opMode_SLEEP         0x00
#define opMode_STDBY         0x04  // Default
#define opMode_TX            0x0C
#define opMode_RX            0x10

#define packetMode 0x00
#define FSK 0x00
#define OOK 0x08
#define shapeNONE 0x00
#define shape1BT 0x01
#define shape05BT 0x02;
#define shape03BT 0x03;

#define Freq_MSB            0x6C
#define Freq_MID            0x40  //434 Mhz -> 0x80
#define Freq_LSB            0x00

#define PA0_ON 0x80
#define PA1_ON 0x40
#define PA2_ON 0x20

#define ocpON 0x10
#define ocpOFF 0x00


#define RXBW_DCCFREQ_010           0x40

#define RXBW_MANT_16 0x00

#define RXBW_EXP_2 0x02

#define CLK_OUT_OFF 0x07

#define FIFO_OVERRUN 0x10
#define MODEREADY            0x80
#define PAYLOADREADY 0x04

#define SyncON  0x80

#define AUTO_FIFOFILL     0x00
#define MANUAL_FIFOFILL   0x40

#define SyncSize_2 0x08

#define DIO0_00   0x00
#define DIO0_01   0x40

#define VariablePacket 0x80

#define NONE_CODE 0x00

#define crcON 0x10

#define crc_autoclear_OFF 0x40
#define crc_autoclear_ON 0x00

#define addressFilter_NONE 0x00

#define RXRESTART 0x04
#define RXRESTARTDELAY_2BITS       0x10
#define AUTORXRESTART_ON           0x02  // Default
#define AES_OFF                    0x00  // Default

#define FIFOTHRESH_TXSTART_FIFONOTEMPTY    0x80  // Recommended default
#define FIFOTHRESH_VALUE                   0x0F  // Default



/*Escrever e ler no fifo*/
#define SPI_WRITE 0x80
#define SPI_READ 0x7F


/*estabelicimento do hardware da placa*/
#define slavePin 8

/*Estabelecer variaveis para estados*/
#define MODE_BITS 0xE3
#define M_SLP 0
#define M_STDBY 1
#define M_FS 2
#define M_TX 3
#define M_RX 4



#endif
