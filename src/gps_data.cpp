#include <Arduino.h>
#include "SkyTraqNmeaParser.h"
#include "gps_data.h"
//#include <SoftwareSerial.h>


void serialInterrupt(void);

//SoftwareSerial mySerial(0,1);

SkyTraqNmeaParser parser;
const GnssData* gdata;
uint8_t gpsQ = 0;

void initGPS() // confiuração para a leitura dos dados fornecido pelo módulo GPS
{
  Serial1.begin(115200,SERIAL_8N1);
  pinMode(2, INPUT);
  digitalWrite(2, LOW);
  attachInterrupt(digitalPinToInterrupt(2), serialInterrupt, CHANGE);

}


void getPosition(uint32_t &xt, uint32_t &yt, uint8_t &h, uint8_t &m, uint8_t &s, uint8_t &gpsst,uint8_t &gpspeed)
{
    gdata = parser.GetGnssData();
    const GnssData& gnss = *gdata;

        double lg2 =  gnss.GetLongitude(); // longitude
        double lat2 = gnss.GetLatitude(); // Latitude

        lg2 = ((lg2*(-1))-8)*100000000; // Obter só as casas décimais para enviar.
        xt = uint32_t(lg2);

        lat2 = (lat2-40)*100000000; // Obter só as casas décimais para enviar
        yt = uint32_t(lat2);

        h = gnss.GetHour(); // hora
        s = gnss.GetMinute(); // minuto
        m = gnss.GetSecond(); //Segundo

        gpsst = gnss.GetQualitMode(); // Estado do módulo GPS
        gpsQ = gpsst;

        gpspeed = gnss.GetSpeedInKmHr(); // Velocidade do estado GPS


}

  double gpsSpeed(void) // Velocidade do módulo GPS
{
  gdata = parser.GetGnssData();
  const GnssData& gnss = *gdata;

  return gnss.GetSpeedInKmHr();
}

void serialInterrupt() //interrupção para ler os dados provenientes do módulo GPS
{

        if(Serial1.available()>0)
        {
                parser.Encode(Serial1.read());
        }

}

uint8_t GPState(){ // ultimo estado registado no módulo GPS
   return gpsQ;
}
