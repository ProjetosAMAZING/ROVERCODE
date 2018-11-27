#include <Arduino.h>
#include "SkyTraqNmeaParser.h"
#include "gps_data.h"
//#include <SoftwareSerial.h>


void serialInterrupt(void);

//SoftwareSerial mySerial(0,1);

SkyTraqNmeaParser parser;
const GnssData* gdata;
uint8_t gpsQ = 0;

void initGPS()
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

        double lg2 =  gnss.GetLongitude();
//        Serial.println(lg2);
        double lat2 = gnss.GetLatitude();
        /*  Serial.print(" - ");
           Serial.print(lat2);
           Serial.print(" , ");
           Serial.print(lg2);
           Serial.println();*/
        lg2 = ((lg2*(-1))-8)*100000000;
        xt = uint32_t(lg2);
        //    Serial.println(xt);
          //Serial.print(xt);
        //Serial.print(",");

        lat2 = (lat2-40)*100000000;
        yt = uint32_t(lat2);
        //      Serial.println(yt);
        //Serial.println(yt);
        //  xt = ((x3-gnss.GetLongitude())/(x4-x3))*255;
        //yt = ((y2-gnss.GetLatitude())/(y2-y1))*255;



        h = gnss.GetHour();
        s = gnss.GetMinute();
        m = gnss.GetSecond();
        //  Serial.println();
        //  Serial.print(m);
        gpsst = gnss.GetQualitMode();
         gpsQ = gpsst;

        gpspeed = gnss.GetSpeedInKmHr();


}

  double gpsSpeed(void)
{
  gdata = parser.GetGnssData();
  const GnssData& gnss = *gdata;

  return gnss.GetSpeedInKmHr();
}

void serialInterrupt()
{

        if(Serial1.available()>0)
        {
                parser.Encode(Serial1.read());
        }

}

uint8_t GPState(){
   return gpsQ;
}
