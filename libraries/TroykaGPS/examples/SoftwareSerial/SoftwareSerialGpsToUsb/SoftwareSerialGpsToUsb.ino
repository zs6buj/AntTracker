// библиотека для работы программного Serial
#include <SoftwareSerial.h>

// создаём объект для работы с программным Serial
// и передаём ему пины TX и RX
SoftwareSerial mySerial(4, 5);

// serial-порт к которому подключён GPS-модуль
#define GPS_SERIAL    mySerial

void setup()
{
  // открываем последовательный порт для мониторинга действий в программе
  // и передаём скорость 9600 бод
  Serial.begin(9600);
  // ждём, пока не откроется монитор последовательного порта
  // для того, чтобы отследить все события в программе
  while (!Serial) {
  }
  Serial.print("Serial init OK\r\n");
  // открываем Serial-соединение с GPS-модулем на скорости 115200 бод
  GPS_SERIAL.begin(115200);
  // печатаем строку
  Serial.println("GPS init is OK on speed 115200");
  // изменяем скорость обещение GPS-модуля с управляющей платой на 9600 бод
  // используем NMEA-команду «$PMTK251,9600*17\r\n»
  GPS_SERIAL.write("$PMTK251,9600*17\r\n");
  // закрываем Serial-соединение с GPS-модулем
  GPS_SERIAL.end();
  // открываем Serial-соединение с GPS-модулем на скорости 9600 бод
  GPS_SERIAL.begin(9600);
  // печатаем строку
  Serial.print("GPS init is OK on speed 9600");
}

void loop()
{
  // если приходят данные из GPS-модуля - отправим их в порт компьютера
  if (GPS_SERIAL.available()) {
    Serial.write(GPS_SERIAL.read());
  }
  // если приходят данные из компьютера - отправим их в GPS-модульl
  if (Serial.available()) {
    GPS_SERIAL.write(Serial.read());
  }
}
