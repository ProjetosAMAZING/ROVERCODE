#ifndef gps_data_H
#define gps_data_H

void getPosition(uint32_t &xt, uint32_t &yt, uint8_t &h, uint8_t &m, uint8_t &s, uint8_t &gpsst,uint8_t &gpspeed);
double gpsSpeed(void);
uint8_t GPState(void);
void initGPS(void);

#endif
