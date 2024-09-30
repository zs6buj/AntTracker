/************************************
    Use of the button lib with a 4x5 button matrix connected via 16-bit I2C port expander (PCF8575).
    When a key is pressed, a corresponding message is output on the serial interface
    Can easily be extended to a matrix up to 4x8 (max) for 32 buttons
***************************************************/

// if this define is commented out, you will get a shortpress AND a click event when the button is clicked
#define CLICK_NO_SHORT                        // click or double click supresses 'short press'

#define MAX32BUTTONS                          // use extended version with > 16 buttons
#include <MobaTools.h>                        // default is 16 buttons per instance
#include <Wire.h>

const byte pcfAdress = 0x20;                  // Adress of PCF8575A portexpander ( may need to be adjusted )
const byte holdLed   = 8;                     // this led is lit as long as at least one button is held down
const byte changeLed = 9;                     // this led is on for 500ms if an arbitrary button changes its state

constexpr byte nbrOfCols = 4;                 // these are the PCF outputs to the matrix ( P00...P03 on PCF8575 )
constexpr byte nbrOfRows = 5;                 // these are the inputs from the matrix ( P10...P14 an PCF8575 )
                                              // this can be increased up to 8 for 32 buttons (P10..P17 )
                                              
const char keyCode[] = "<741A0852B>963#\nEv^*";   // key codes assigned to the buttons
                                                  //may need to be adjusted to your matrix
                                                  
constexpr byte NbrOfButtons = nbrOfCols * nbrOfRows; // Total number of buttons

// callback function for MoToButtons - raw reading of the matrix
button_t getHW( void ) {
    // Scanning the keypad
    button_t tasterTemp = 0;                      // every bit represents one button
    for (byte i = 0; i < nbrOfCols; i++) {              // scan 4 lines
        Wire.beginTransmission(pcfAdress);        // transmit to PCF
        Wire.write(0xff & ~(1 << (i)));           // select column ( LOW on one bit in low nibble (P0...P3 ) )
        Wire.write(0xff);                         //  receive pins must be high on output
        Wire.endTransmission();                   // stop transmitting
        
        Wire.requestFrom(pcfAdress, 2);           // read row ( a LOW means pressed )
        while (!Wire.available());                // wait till byte is received
        byte answ = Wire.read();                  // first byte is output ( row pins ) - discard
        while (!Wire.available());                // wait till byte is received
        answ = ~(Wire.read()) & 0x1f;             // pressed buttons must be HIGH for MoToButtons, so invert
        tasterTemp |= ( (uint32_t)answ << ( nbrOfRows * i ) );  // shift to correct position in tasterTemp
    }
    return tasterTemp;
}

MoToButtons buttonPad( getHW, 20, 500 );  // 'getHW' ist called every 20ms ( debouncing )
                                          // >500ms for a long press

MoToTimer changeTimer;                // timer for the change led ( it is switched off if this timer expires )

void setup()
{
    Serial.begin(115200);
    while (!Serial);                          // needed only for MCU with native USB
    pinMode( holdLed, OUTPUT );
    pinMode( changeLed, OUTPUT );
    Wire.begin();
    Serial.println("Start loop");
}

void loop() {
    //--------------------------------------------------------
    buttonPad.processButtons();
    //--------------------------------------------------------
    
    // The hold LED is lit as  long as any button is held down
    digitalWrite( holdLed, (buttonPad.allStates() != 0));

    //The changeLed is lit for 1sec if any button changes its state
    if ( buttonPad.changed() ) {
        // at least on button changed its state (pressed or released)
        digitalWrite( changeLed, HIGH );
        //Serial.print("Buttonstate changed: ");
        //Serial.println( buttonPad.allStates(),HEX );
        changeTimer.setTime(1000);
    }
    if ( changeTimer.expired() ) {
        digitalWrite( changeLed, LOW );
        //Serial.println("no change within 1sec");
    }


    for (byte i = 0; i < NbrOfButtons; i++) {
        // check all buttons
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
