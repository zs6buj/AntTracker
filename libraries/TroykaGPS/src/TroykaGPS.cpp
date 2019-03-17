#include <stdio.h>
#include "TroykaGPS.h"

using namespace GPSNAME;

enum { ZERO, ONE, TWO, THREE, FOUR, FIVE, FIND, COPY, SAVE };

GPS::GPS(Stream &serial) : UARTDevice(&serial) {

}

int GPS::available() {
    return uartDeviceAvailable();
}

int GPS::available(int time) {
    return uartDeviceAvailable(time);
}

int GPS::read() {
    return uartDeviceReadSerial();
}

int GPS::readParsing() {
    // $GNGGA,165708.000,5544.648951,N,03739.472758,E,2,5,4.62,143.098,M,14.426,M,,*49
    // $GNRMC,165708.000,A,5544.648951,N,03739.472758,E,4.00,63.20,070717,,,D*4F
    int i = 0;
    int j = 0;
    int t = 0;
    char gpsBuffer[SIZE_GPS_BUFFER];
    char connectSat[4];
    char speed[8];
    char altitude[8];
    char sat[8];
    char time[16];
    char date[16];
    char latitude[16];
    char longitude[16];
    char parsingLatitude[16];
    char parsingLongitude[16];    
    _findGNGGA = ZERO;
    _findGNRMC = ZERO;
    // очищаем буффер GPS-модуля
    uartDeviceFlushSerial();
    // пока не считали полностью две искомые строки данных
    while (_findGNGGA != SAVE || _findGNRMC != SAVE) {
        // если идут данные с gps-модуля
        if (uartDeviceAvailable()) {
            // считываем один символ
            char c = uartDeviceReadSerial();
            // если была найдена искомая строка «$GNGGA»
            // переходим в режим копирования
            if (_findGNGGA == FIND) {
                uartDeviceCleanBuffer(gpsBuffer, sizeof(gpsBuffer));
                strcat(gpsBuffer, "$GNGGA");
                i = strlen(gpsBuffer);
                _findGNGGA = COPY;
            }
            // если была найдена искомая строка «$GNRMC»
            // переходим в режим копирования
            if (_findGNRMC == FIND) {
                uartDeviceCleanBuffer(gpsBuffer, sizeof(gpsBuffer));
                strcat(gpsBuffer, "$GNRMC");
                i = strlen(gpsBuffer);
                _findGNRMC = COPY;
            }

            // если в режиме копирования строки «GNGGA»
            if (_findGNGGA == COPY) {
                if (c != '\n') {
                    gpsBuffer[i++] = c;
                } else {
                    gpsBuffer[i] = '\0';
                    parsingGNGGA(gpsBuffer, sat, altitude);
                    uartDeviceCleanBuffer(gpsBuffer, sizeof(gpsBuffer));
                    _findGNGGA = SAVE;
                }
            }

            // если в режиме копирования строки «GNRMC»
            if (_findGNRMC == COPY) {
                if (c != '\n') {
                    gpsBuffer[i++] = c;
                } else {
                    gpsBuffer[i] = '\0';
                    parsingGNRMC(gpsBuffer, connectSat, time, date, latitude, longitude, speed);
                    uartDeviceCleanBuffer(gpsBuffer, sizeof(gpsBuffer));
                    _findGNRMC = SAVE;
                }
            }

            setHeaderState(c, &_findGNGGA, "$GNGGA");
            setHeaderState(c, &_findGNRMC, "$GNRMC");
			//Serial.print(c);
        }
    }


    // есть ли связь со спутниками
    if (connectSat[0] != 'A') {
        _gpsState = GPS_ERROR_SAT;
        return _gpsState;
    }

    // запись данных времени в виде массива символов
    _time[0] = time[0];
    _time[1] = time[1];
    _time[2] = ':';
    _time[3] = time[2];
    _time[4] = time[3];
    _time[5] = ':';
    _time[6] = time[4];
    _time[7] = time[5];
    _time[8] = '\0';
    // запись данных времени в целочисленные переменные
    _hour = (time[0] - '0') * 10 + (time[1] - '0');  // changed to UTC
    _minute = (time[2] - '0') * 10 + (time[3] - '0');
    _second = (time[4] - '0') * 10 + (time[5] - '0');

    // запись данных даты в виде массива символов
    _date[0] = date[0];
    _date[1] = date[1];
    _date[2] = '.';
    _date[3] = date[2];
    _date[4] = date[3];
    _date[5] = '.';
    _date[6] = '2';
    _date[7] = '0';
    _date[8] = date[4];
    _date[9] = date[5];
    _date[10] = '\0';
    // запись данных времени в целочисленные переменные
    _day = (date[0] - '0') * 10 + (date[1] - '0');
    _month = (date[2] - '0') * 10 + (date[3] - '0');
    _year = 2000 + (date[4] - '0') * 10 + (date[5] - '0');

    // запись данных количества видимых спутников
    _sat = atoi(sat);
    // запись данных текущей скорости в узлах
    _speed = atof(speed);
    // запись данных высоты над уровнем моря
    _altitude = atof(altitude);

    // запись данных широты в ° градусах в виде десятичной дроби
    i = 2;
    j = 0;
    while (latitude[i] != '\0') {
        parsingLatitude[j++] = latitude[i++];
    }
    parsingLatitude[j] = '\0';
    _latitudeBase10 = atof(parsingLatitude) * 100 / 60 / 100;
    _latitudeBase10 = _latitudeBase10 + (latitude[0] - '0') * 10 + (latitude[1] - '0');
    if (latitude[strlen(latitude) - 1] == 'S') {
        _latitudeBase10 = -_latitudeBase10;
    }

    // запись данных долготы в ° градусах в виде десятичной дроби
    i = 3;
    j = 0;
    while (longitude[i] != '\0') {
        parsingLongitude[j++] = longitude[i++];
    }
    parsingLongitude[j] = '\0';
    _longitudeBase10 = atof(parsingLongitude) * 100 / 60 / 100;
    _longitudeBase10 = _longitudeBase10 + (longitude[0] - '0') * 100 + (longitude[1] - '0') * 10 + (longitude[2] - '0');
    if (longitude[strlen(longitude) - 1] == 'W') {
        _longitudeBase10 = -_longitudeBase10;
    }

    // запись данных широты в ° градусах, ′ минутах и ″ секундах с десятичной дробью
    float lalitudeSec;    
    char latitudeSecStr[12];
    i = 0;
    t = 0;
    _latitudeBase60[t++] = latitude[i++];
    _latitudeBase60[t++] = latitude[i++];
    _latitudeBase60[t++] = '*';
    _latitudeBase60[t++] = latitude[i++];
    _latitudeBase60[t++] = latitude[i++];
    _latitudeBase60[t++] = '\'';
    i++;
    j = 0;
    while (latitude[i] != '\0') {
        latitudeSecStr[j++] = latitude[i++];
    }
    latitudeSecStr[j] = '\0';
    lalitudeSec = atof(latitudeSecStr) * 60 / 100 / 10000;
    dtostrf(lalitudeSec, 6, 4, latitudeSecStr);
    i = 0;
    while (latitudeSecStr[i] != '\0') {
        _latitudeBase60[t++] = latitudeSecStr[i++];
    }
    _latitudeBase60[t++] = '"';
    _latitudeBase60[t++] = latitude[strlen(latitude) - 1];
    _latitudeBase60[t] = '\0';

    // запись данных долготы в ° градусах, ′ минутах и ″ секундах с десятичной дробью
    float longitudeSec;    
    char longitudeSecStr[12];
    i = 0;
    t = 0;
    if (longitude[0] == '1') {
        _longitudeBase60[t++] = longitude[i++];
    } else {
        i++;
    }
    _longitudeBase60[t++] = longitude[i++];
    _longitudeBase60[t++] = longitude[i++];
    _longitudeBase60[t++] = '*';
    _longitudeBase60[t++] = longitude[i++];
    _longitudeBase60[t++] = longitude[i++];
    _longitudeBase60[t++] = '\'';

    i++;
    j = 0;
    while (longitude[i] != '\0') {
        longitudeSecStr[j++] = longitude[i++];
    }
    longitudeSecStr[j] = '\0';
    longitudeSec = atof(longitudeSecStr) * 60 / 100 / 10000;
    dtostrf(longitudeSec, 6, 4, longitudeSecStr);
    
    i = 0;
    while (longitudeSecStr[i] != '\0') {
        _longitudeBase60[t++] = longitudeSecStr[i++];
    }
    _longitudeBase60[t++] = '"';
    _longitudeBase60[t++] = longitude[strlen(longitude) - 1];
    _longitudeBase60[t] = '\0';
    
    _gpsState = GPS_OK;
    return _gpsState;
}

void GPS::parsingGNGGA(char* gpsBuffer, char* sat, char* altitude) {
    int i = 0;
    int j = 0;
    // пропуск типа сообщения
    if (gpsBuffer[i] == '$') {
        i++;
        while (gpsBuffer[i] != ',') {
            i++;
        }
    }

    // пропуск записи данных о времени
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != '.') {
            i++;
        }
        i++;
        while (gpsBuffer[i] != ',') {
            i++;
        }
    }

    // пропуск данных широты
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            i++;
        }   
    }
    // пропуск буквы широты «N»
    i = i + 2;

    // пропуск данных долготы
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            i++;
        }
    }

    // пропуск буквы долготы «E»
    i = i + 2;
    // пропуск типа решения
    i = i + 2;

    // запись данных кол-ва спутников
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            sat[j++] = gpsBuffer[i++];
        }
        sat[j] = '\0';
    }

    // пропуск геометрического фактора
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            i++;
        }   
    }
    // запись данных высоты
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            altitude[j++] = gpsBuffer[i++];
        }
        altitude[j] = '\0';
    }
}

void GPS::parsingGNRMC(char* gpsBuffer, char* connectSat, char* time, char* date, char* latitude, char* longitude, char* speed) {

    int i = 0;
    int j = 0;

    if (gpsBuffer[i] == '$') {
        i++;
        while (gpsBuffer[i] != ',') {
            i++;
        }
    }

    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != '.') {
            time[j++] = gpsBuffer[i++];
        }
        i++;
        time[j] = '\0';
        while (gpsBuffer[i] != ',') {
            i++;
        }
    }
    // состояние GPS
    if (gpsBuffer[i] == ',') {
        i++;
        connectSat[0] = gpsBuffer[i];
        i++;
    }

    // запись данных широты
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            latitude[j++] = gpsBuffer[i++];
        }
        i++;
        while (gpsBuffer[i] != ',') {
            latitude[j++] = gpsBuffer[i++];
        }
        latitude[j] = '\0';     
    }

    // запись данных долготы
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            longitude[j++] = gpsBuffer[i++];
        }
        i++;
        while (gpsBuffer[i] != ',') {
            longitude[j++] = gpsBuffer[i++];
        }
        longitude[j] = '\0';
    }

    // запись данных скорости
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
        speed[j++] = gpsBuffer[i++];
        }
        speed[j] = '\0';
    }

    // пропуск данных высоты
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            i++;
        }
    }

    // запись даты
    if (gpsBuffer[i] == ',') {
        i++;
        j = 0;
        while (gpsBuffer[i] != ',') {
            date[j++] = gpsBuffer[i++];
        }
        date[j] = '\0';
    }
}

void GPS::setHeaderState(char c, int8_t* state, char* header) {
    if (*state != FIND && *state != COPY && *state != SAVE) {
        if (*state == ZERO && c == header[ZERO]) {
            *state = ONE;
        } else if (*state == ONE && c == header[ONE]) {
            *state = TWO;
        } else if (*state == TWO && c == header[TWO]) {
            *state = THREE;
        } else if (*state == THREE && c == header[THREE]) {
            *state = FOUR;
        } else if (*state == FOUR && c == header[FOUR]) {
            *state = FIVE;
        } else if (*state == FIVE && c == header[FIVE]) {
            *state = FIND;
        } else {
            *state = ZERO;
        }
    }
}

void GPS::getLatitudeBase60(char* latitudeBase60, size_t maxLength) const {
    strncpy(latitudeBase60, _latitudeBase60, maxLength);
}

void GPS::getLongitudeBase60(char* longitudeBase60, size_t maxLength) const {
    strncpy(longitudeBase60, _longitudeBase60, maxLength);
}

void GPS::getTime(char* time, size_t maxLength) const {
    strncpy(time, _time, maxLength);
}

void GPS::getDate(char* date, size_t maxLength) const {
    strncpy(date, _date, maxLength);
}
