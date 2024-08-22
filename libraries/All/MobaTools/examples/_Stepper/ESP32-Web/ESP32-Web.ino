#include <WebServer.h>
#include <ArduinoOTA.h> // Programm hochladen Over The Air, siehe IDE Werkzeuge/Port
#include <MobaTools.h>  // ab 2.4.0: https://github.com/MicroBahner/MobaTools

const char *ssid = "Esp32AP";      // frei zu vergebener Name des Access Points, kann bis zu 32 Zeichen haben
const char *password = "12345678"; // frei zu vergebenes Passwort, mindestens 8 Zeichen jedoch nicht länger als 64 Zeichen

const byte dirPin       = 33; // verbinden mit DIR des Schrittmotortreibers
const byte stepPin      = 32; // verbinden mit STEP des Schrittmotortreibers
const byte enaPin       = 13; // verbinden mit ENA des Schrittmotortreibers

enum class Aktionen {STOP, LINKS, RECHTS, CONTL, CONTR};  // Aktionen des endlichen Automaten
const int STEPS_REVOLUTION = 3200;                         // 1/16 Microstep -> 3200 Schritte / Umdrehung
MoToStepper myStepper( STEPS_REVOLUTION, STEPDIR );       // Stepper einrichten
WebServer server(80);

#define DEBUGGING                                         // Einkommentieren für die Serielle Ausgabe
#ifdef DEBUGGING
#define DEBUG_B(...) Serial.begin(__VA_ARGS__)
#define DEBUG_P(...) Serial.println(__VA_ARGS__)
#define DEBUG_F(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_B(...)
#define DEBUG_P(...)
#define DEBUG_F(...)
#endif

#include "website.h"

void setup() {
  myStepper.attach( stepPin, dirPin );               // Step- und Dir-Pin aktivieren
  myStepper.attachEnable( enaPin, 10, LOW );         // Enable Pin aktivieren ( LOW=aktiv )
  myStepper.setSpeed( 2000 );                        // 200 U/min
  myStepper.setRampLen( 300 );                       // Rampenlänge 300 Steps

  DEBUG_B(115200);
  DEBUG_F("\nSketchname: %s\nBuild: %s\t\tIDE: %d.%d.%d\n\n", __FILE__, __TIMESTAMP__, ARDUINO / 10000, ARDUINO % 10000 / 100, ARDUINO % 100 / 10 ? ARDUINO % 100 : ARDUINO % 10);

  WiFi.mode(WIFI_AP);                                // Der ESP32 bildet als Access Point sein eigenes WLAN
  if (WiFi.softAP(ssid, password)) {
    DEBUG_F("Verbinde dich mit dem Netzwerk \"%s\"\nGib \"%s/stepper\" im Browser ein\n\n", ssid, WiFi.softAPIP().toString().c_str());
  } else {
    DEBUG_P("Fehler beim erstellen.");
  }

  server.on ( "/stepper", handleStepper );           // Im Browser "http://192.168.4.1/stepper" ruft diese Funktion auf.

  ArduinoOTA.begin();                                // OTA starten

  server.begin();                                    // Webserver starten
  DEBUG_P("HTTP Server gestartet\n\n");
}

void loop() {
  ArduinoOTA.handle();                               // OTA abhandeln
  server.handleClient();                             // Anfragen an den Server abhandeln
}

void handleStepper() {        // Reaktion auf Eingaben und Html-Seite als Antwort an Browser zurückschicken
  char htmlTemp[sizeof(HTMLTEXT)+15]; // Size of website+some extra bytes for 2 variable fields
  long htSpeed = 150000;
  long htRamp = 100;
  if (server.hasArg("links")) esp32Stepper(Aktionen::LINKS);   // http://192.168.4.1/stepper?links
  if (server.hasArg("rechts")) esp32Stepper(Aktionen::RECHTS); // http://192.168.4.1/stepper?rechts
  if (server.hasArg("contl")) esp32Stepper(Aktionen::CONTL);   // http://192.168.4.1/stepper?contl
  if (server.hasArg("contr")) esp32Stepper(Aktionen::CONTR);   // http://192.168.4.1/stepper?contr
  if (server.hasArg("stop"))  esp32Stepper(Aktionen::STOP);     // http://192.168.4.1/stepper?stop
  if (server.hasArg("speed")) htSpeed = 10*server.arg("speed").toInt(); 
  if (server.hasArg("ramp"))  htRamp = server.arg("ramp").toInt();

  if (server.hasArg("setSpeed")) {
    // seting speed and autoramp
    htRamp = myStepper.setSpeedSteps(htSpeed);
  }
  if (server.hasArg("setRamp")) {
    // seting ramplength
    htRamp = myStepper.setRampLen(htRamp);
  }

  int htmlSize = snprintf( htmlTemp, sizeof(htmlTemp), HTMLTEXT, htSpeed/10, htRamp );
  server.send(200, "text/html", htmlTemp);
  DEBUG_F("Html-Größe = %d Byte\n\r", htmlSize);
}

void esp32Stepper(const Aktionen aktion) {
  switch (aktion) {
    case Aktionen::LINKS:
      myStepper.doSteps(-STEPS_REVOLUTION); // Stepper dreht eine Umdrehung links
      DEBUG_P("Stepper dreht eine Umdrehung links.");
      break;
    case Aktionen::RECHTS:
      myStepper.doSteps(STEPS_REVOLUTION); // Stepper dreht eine Umdrehung rechts
      DEBUG_P("Stepper dreht eine Umdrehung rechts.");
      break;
    case Aktionen::CONTL:
      myStepper.rotate( -1 ); // Stepper dreht links
      DEBUG_P("Stepper dreht links.");
      break;
    case Aktionen::CONTR:
      myStepper.rotate( 1 ); // Stepper dreht rechts
      DEBUG_P("Stepper dreht rechts.");
      break;
    case Aktionen::STOP:
      myStepper.rotate( 0 ); // Stepper stoppt
      DEBUG_P("Stepper stoppt.");
      break;
  }
}
