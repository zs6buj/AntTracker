#define MAX8BUTTONS
#include <MobaTools.h>

const uint32_t BLENDZEIT = 50;
const byte taster = 2;
const byte ledPins[] = {3,4,5,6,7,8,9, 10};
const byte pinCount = sizeof(ledPins);
enum {LAUF, AUSSCHALTEN, AUS};
byte schritt = LAUF;

button_t getHW( void ) {    // User-Callback-Funktion für Tasterstatus
  return (button_t) !digitalRead(taster);
}

MoToButtons Taster( getHW, 20, 500 );
MoToSoftLed meineLeds[pinCount];
MoToTimer myTimer;

void setup() {
  //Serial.begin(115200);
  //while(!Serial);
  //Serial.println("Anfang");
  pinMode(taster, INPUT_PULLUP);
  
  for (byte led = 0; led < pinCount; led++) {
    meineLeds[led].attach(ledPins[led]);
    meineLeds[led].riseTime( BLENDZEIT*10 );    // Aufblendzeit in ms
  }
}

void loop() {
  Taster.processButtons();
  switch (schritt) {
    case LAUF: 
      lauflicht();
      if (Taster.pressed(0)) {
        schritt = AUSSCHALTEN;
      }
      break;
    case AUSSCHALTEN:
      for (byte led = 0; led < pinCount; led++) {
        meineLeds[led].off();
      }
      schritt = AUS;
      break;
    case AUS:
      if (Taster.pressed(0)) {
        schritt = LAUF;
      }
      break;
    default:
      schritt = LAUF;
  }
}
void lauflicht() {
  static byte led = 0;
  static bool ein = true;
  static bool hin = true;

  if (!myTimer.running()) {
    if (ein) {
      meineLeds[hin ? led : pinCount - 1 - led].on();
    } else {
      meineLeds[hin ? led : pinCount - 1 - led].off();
      led = (1 + led) % (pinCount - 1);
      if (!led) hin = !hin;
    }
    myTimer.setTime(BLENDZEIT);
    ein = !ein;
  }
}
