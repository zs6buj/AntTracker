/************************************
  Nutzung der Button-Lib mit einer über I2C angeschlossenen 4x4 Tastermatrix
  Beim Drücken einer Taste wird eine entsprechende Meldung auf der seriellen
  Schnittstelle ausgegeben

  Use of the button lib with a 4x4 button matrix connected via I2C port expander (PCF8574).
  When a key is pressed, a corresponding message is output on the serial interface
***************************************************/

// if this define is commented out, you will get a shortpress AND a click event when the button is clicked
#define CLICK_NO_SHORT                        // click or double click supresses 'short press'

#include <MobaTools.h>                        // default is 16 buttons per instance
#include <Wire.h>

const byte pcfAdress = 0x38;                  // Adress of PCF8574A portexpander
const byte holdLed   = 8;                     // this led is lit as long as at least one button is held down
const byte changeLed = 9;                     // this led is on for 500ms if an arbitrary button changes its state

const char keyCode[] = "*0#D789C456B123A";    // may need to be adjusted

// callback function for MoToButtons
button_t getHW( void ) {
  // Einlesen der Tasterstates / scanning the keypad
  button_t tasterTemp = 0;
  for (byte i = 0; i < 4; i++) {              // scan 4 lines
    Wire.beginTransmission(pcfAdress);        // transmit to PCF
    Wire.write(0xff & ~(1 << (i+4) ) );        // select line ( LOW on one bit in high nibble )
    Wire.endTransmission();                   // stop transmitting
    Wire.requestFrom(pcfAdress, 1);           // read row ( a LOW in low nibble means pressed )
    while (!Wire.available());                // wait till byte is received
    byte answ = ~(Wire.read()) & 0xf;         // pressed buttons must be HIGH, so invert
    tasterTemp |= ( answ << (4 * i) );        // shift the bits to the correct position
  }
  return tasterTemp;                          // every bit represents the state of one button
}

MoToButtons buttonPad( getHW, 20, 500 );  // 'getHW' ist called every 20ms ( debouncing )
// >500ms for a long press
// 20ms Entprellzeit, 500ms für den Unterschied kurz/lang gedrückt.

MoToTimer changeTimer;                // timer for the change led

void setup()
{
  Serial.begin(115200);
  while (!Serial);                          // nur für Leonardo und Micro notwendig (Mega32u4)
  pinMode( holdLed, OUTPUT );
  pinMode( changeLed, OUTPUT );
  Wire.begin();
  Serial.println("Start loop");
}

void loop() {
  //--------------------------------------------------------
  buttonPad.processButtons();
  //--------------------------------------------------------

  digitalWrite( holdLed, (buttonPad.allStates()!=0));

  if ( buttonPad.changed() ) {
    // at least on led changed its state
    digitalWrite( changeLed, HIGH );
    changeTimer.setTime(500);
  }
  if ( changeTimer.expired() ) {
    digitalWrite( changeLed, LOW );
  }


  for (byte i = 0; i < 16; i++) {
    // check all 16 buttons
    if ( buttonPad.longPress(i) ) {
      Serial.print(" long press: "); Serial.println(keyCode[i]);
    }

    switch ( buttonPad.clicked(i) ) {
      case SINGLECLICK :
        Serial.print(" single click: "); Serial.println(keyCode[i]);
        break;
      case DOUBLECLICK :
        Serial.print(" double click: "); Serial.println(keyCode[i]);
        break;
      case NOCLICK:
        // a very short press is also a click!
        // usually you have to decide wether to check for a click or a short press
        if ( buttonPad.shortPress(i) ) {
          Serial.print(" short press: "); Serial.println(keyCode[i]);
        }
        break;
    }
  }
}
