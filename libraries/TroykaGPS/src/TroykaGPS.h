#ifndef __AMPERKA_GPS_H__
#define __AMPERKA_GPS_H__

#include "./utility/uartDevice.h"
#include "Print.h"
#include "Stream.h"

#if defined (__SAM3X8E__) || defined(__SAM3A8C__) || defined(__SAM3A4C__) || defined(__SAMD21G18A__)
#include <avr/dtostrf.h>
#endif

#define GPS_OK              1
#define GPS_ERROR_DATA      2
#define GPS_ERROR_SAT       3

#define SIZE_GPS_BUFFER     96

#define KNOT_TO_KM          1.852

class GPS : public GPSNAME::UARTDevice
{
public:
    GPS(Stream &serial);
    int available();
    int available(int time);
    int read();
    int readParsing();
    int8_t getState() const { return _gpsState; }
    int8_t getSat() const { return _sat; }
    float getSpeedKn() const { return _speed; }
    float getSpeedKm() const { return _speed * KNOT_TO_KM; }
    float getAltitude() const { return _altitude; }
    float getLatitudeBase10() const { return _latitudeBase10; }
    float getLongitudeBase10() const { return _longitudeBase10; }
    void getLatitudeBase60(char* latitudeBase60, size_t maxLength) const;
    void getLongitudeBase60(char* longitudeBase60, size_t maxLength) const;
    void getTime(char* time, size_t maxLength) const;
    void getDate(char* date, size_t maxLength) const;
    int8_t getSecond() const { return _second; }
    int8_t getMinute() const { return _minute; }
    int8_t getHour() const { return _hour; }
    int8_t getDay() const { return _day; }
    int8_t getMonth() const { return _month; }
    uint16_t getYear() const { return _year; }

private:
    void setHeaderState(char c, int8_t* state, char* header);
    void parsingGNGGA(char* gpsBuffer, char* sat, char* altitude);
    void parsingGNRMC(char* gpsBuffer, char* connectSat, char* time, char* date, char* latitude, char* longitude, char* speed);
    bool _connectSat;
    int8_t _gpsState;
    int8_t _sat;
    int8_t _second;
    int8_t _minute;
    int8_t _hour;
    int8_t _day;
    int8_t _month;
    uint16_t _year;
    float _speed;
    float _altitude;
    float _latitudeBase10;
    float _longitudeBase10;
    char _latitudeBase60[16];
    char _longitudeBase60[16];
    char _time[16];
    char _date[16];
    int8_t _findGNGGA;
    int8_t _findGNRMC;
};

#endif
