/*
  WiFi Web Server to control a stepper ( for UNO R4 WiFi )

  A simple web server that lets control a stepper via the web.
  This sketch will print the IP address of your WiFi module (once connected)
  to the Serial Monitor. From there, you can open that address in a web browser
  to control a stepper.
  
  An extension of the AP_SimpleWebServer example for UNO R4 WIFI
*/

// One of the following 2 defines must be active:
//#define DEBUG_P( x, ... ) {char dbgBuf[80]; snprintf_P( dbgBuf, 80, PSTR( x ), ##__VA_ARGS__ ) ; Serial.println( dbgBuf ); }
#define DEBUG_P(...)  // no debug printing

#include "WiFiS3.h"
#include <MobaTools.h>  // min version 2.6.1: https://github.com/MicroBahner/MobaTools

#include "arduino_secrets.h"
/*/////please enter your sensitive data in the newly created tab 'arduino_secrets.h'
  e.g.:
  #define SECRET_SSID "your-ssid"
  #define SECRET_PASS "your-passwd"
*/
char ssid[] = SECRET_SSID;          // your network SSID (name)
char pass[] = SECRET_PASS;          // your network WPA  password

// change pin numers to your needs
const byte dirPin       = 5;
const byte stepPin      = 6;
const byte enaPin       = 7;

const int STEPS_REVOLUTION = 800;                         // 1/4 Microstep -> 800 steps/rev
MoToStepper myStepper( STEPS_REVOLUTION, STEPDIR );       // create stepper instance

uint32_t htSpeed = 8000;    // steps/10 sec
uint32_t htRamp = 100;      // ramp length in steps

//int status = WL_IDLE_STATUS;
WiFiServer server(80);
#include "website.h"

void setup() {
  Serial.begin(115200);                             // initialize serial communication
  // initialize stepper
  myStepper.attach( stepPin, dirPin );              // assign step/dir pins
  myStepper.attachEnable( enaPin, 10, LOW );        // attach enable in ( LOW=active )
  myStepper.setSpeedSteps( htSpeed );               // initial value of speed
  myStepper.setRampLen( htRamp );                   // initial ramp length


  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to Network named: ");
  Serial.println(ssid);                   // print the network name (SSID);
  WiFi.begin(ssid, pass);
  Serial.print("Waiting .");
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
}

// text buffer receiving from connected browser
constexpr uint8_t lineBufLen = 80;
char lineBuf[lineBufLen];
uint8_t bufIx = 0;

void loop() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    DEBUG_P("new client");                  // print a message out the serial port
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character
          lineBuf[bufIx] = '\0';          // terminate string

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (bufIx == 0) {
            char htmlTemp[sizeof(HTMLTEXT) + 15]; // Size of website+some extra bytes for 2 variable fields
            DEBUG_P("Send website, speed=%d, ramp=%d", htSpeed, htRamp)
            snprintf( htmlTemp, sizeof(htmlTemp), HTMLTEXT, htRamp, htSpeed / 10 );
            client.print( htmlTemp);
            DEBUG_P("---------------Send HTTP-Page --------------------");
            break;
          } else {
            // We got a complete line. Test if we got a GET line and process it if so
            handleStepper(  lineBuf );
            bufIx = 0;
          }

        } else if (c != '\r') {
          // If you got anything else but a carriage return character,
          // write it into the buffer, but pay attention to the buffer length!
          if ( bufIx < (lineBufLen - 1) ) {      // leave space for a terminating \0
            lineBuf[bufIx++] = c;
          }
        }

      }
    } // End 'while connected'
    // close the connection:
    client.stop();
    DEBUG_P("client disconnected");
  }
}

// Field commands from the browser
enum : uint8_t {SPEED, RAMP, SETSPEED_RAMP, SETSPEED, SETRAMP, LINKS, RECHTS, CONTL, CONTR, STOP}; // actions
char * keyWords[] = { "speed=", "ramp=", "setSpeedRamp=", "setSpeed=", "setRamp=", "links=", "rechts=", "contl=", "contr=", "stop=" };
constexpr uint8_t keyIxMax = sizeof(keyWords) / sizeof(keyWords[0]);
uint8_t keyIx;
char *keyP, *valP;

void handleStepper( char* GETcom ) {
  //check for GET command from client
  char* strGET = strstr( GETcom, "GET" );
  if ( strGET != NULL ) {
    // It's a GET line - process it
    uint32_t tmpSpeed = 8000, tmpRamp = 100;    // Default values for ramp and speed field - will usually been overwritten.
    Serial.println(GETcom);
    strGET = strstr( strGET, "stepper?" );
    if ( strGET != NULL ) {
      strGET += strlen("stepper?");
      strGET = strtok( strGET, " &" );
      DEBUG_P("vvvvvvvvvvvvvvvvvvvvvvv");
      while ( strGET != NULL ) {
        // DEBUG_P( strGET );
        // check for keywords - separate keyword from value
        valP = strchr(strGET, (int)'=');
        if ( valP ) valP++;
        for ( keyIx = 0; keyIx < keyIxMax; keyIx++ ) {
          keyP = strstr( strGET, keyWords[keyIx] );
          if ( keyP != NULL ) {
            DEBUG_P("Keyword: % s, Ix: % d", keyP, keyIx);
            // found keyword
            switch (keyIx) {
              case SPEED:
                tmpSpeed = 10 * atoi(valP);
                DEBUG_P("Speedval = %d", tmpSpeed);
                break;
              case RAMP:
                tmpRamp = atoi(valP);
                DEBUG_P("Rampval = % d", tmpRamp);
                break;
              case SETSPEED:
                htSpeed = tmpSpeed;
                htRamp = myStepper.setSpeedSteps(htSpeed, htRamp);
                DEBUG_P("set Speed = % d, actRamp = % d", htSpeed, htRamp);
                break;
              case SETSPEED_RAMP:
                htSpeed = tmpSpeed;
                htRamp = myStepper.setSpeedSteps(htSpeed);
                DEBUG_P("setSpeed + ramp = % d, actRamp = % d", htSpeed, htRamp);
                break;
              case SETRAMP:
                htRamp = tmpRamp;
                htRamp = myStepper.setRampLen(htRamp);
                break;
              case LINKS:
                myStepper.doSteps(-STEPS_REVOLUTION); 
                DEBUG_P("One rev. CCW");
                break;
              case RECHTS:
                myStepper.doSteps(STEPS_REVOLUTION); 
                DEBUG_P("one rev. CW");
                break;
              case CONTL:
                myStepper.rotate( -1 ); 
                DEBUG_P("rotate CCW");
                break;
              case CONTR:
                myStepper.rotate( 1 );
                DEBUG_P("rotate CW");
                break;
              case STOP:
                myStepper.rotate( 0 );
                DEBUG_P("Stop the stepper");
                break;
            }
            break; // Exit for loop, we found the keyword.
          }
        }
        strGET = strtok( NULL, " &" );
      }
      DEBUG_P("^^^^^^^^^^^^^^^^^^^^^^^^^");
    }
  }

}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
