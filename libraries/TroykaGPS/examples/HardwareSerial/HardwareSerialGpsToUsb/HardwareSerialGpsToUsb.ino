// serial-порт к которому подключён GPS-модуль
#define GPS_SERIAL    Serial1

void setup()
{
  // открываем последовательный порт для мониторинга действий в программе
  // и передаём скорость 115200 бод
  Serial.begin(115200);
  while (!Serial) {
  }
  Serial.print("\r\nSerial init OK\r\n");
  // открываем Serial-соединение с GPS-модулем
  // и передаём скорсть 115200 бод
  GPS_SERIAL.begin(115200);
}

void loop()
{
  // если приходят данные из GPS-модуля - отправим их в порт компьютера
  if (GPS_SERIAL.available()) {
    Serial.write(GPS_SERIAL.read());
  }
  // если приходят данные из компьютера - отправим их в GPS-модуль
  if (Serial.available()) {
    GPS_SERIAL.write(Serial.read());
  }
}
