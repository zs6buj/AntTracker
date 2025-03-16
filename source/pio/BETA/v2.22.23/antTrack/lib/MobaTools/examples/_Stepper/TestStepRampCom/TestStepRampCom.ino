// Testsketch für Stepper-Rampe mit automatischer Befehlsfolge
// Steuerung der Stepper-Methoden per serieller Schnittstelle und per Abfolgen im EEPROM

/* Steuerung der Methodenaufrufe per serieller Schnittstelle
 *  dst nnn     -> doSteps( +/-nnnL )
 *  wra nnn     -> write( +/-angleL )
 *  wrs nnn     -> writeSteps ( +/-stepsL )
 *  rot d       -> rotate( +/-direction );
 *  ssp nn      -> setSpeed( rpm10
 *  sss nn rr   -> setSpeedSteps( speed10, ramp ) wenn rr fehlt wird es nicht ausgegeben
 *  srl nn      -> setRampLen( ramp )
 *  stp         -> stop
 *  ena n       -> enable on(n=1) or off (n=0) returns state of enable
 *  mov         -> print Restweg in % vom Gesamtweg
 *  std         -> print Restweg in Steps
 *  rda         -> print anglepos
 *  rds         -> print steppos
 *  szp         -> set zeropoint
 *  wrp         -> aktueller Punkt als Zielpunkt ( mit Rampe! )
 *  
 *  est nn      -> Kommandos in EEPROM ab Befehl nn abarbeiten
 *  esp         -> Abbarbeitung der EEPROM-Komandos stoppen
 *  eep ii c pp ccc nn rr Kommando im EEPROM an Stelle ii ablegen ( 0 <= ii < EEMAX )
 *              c= '-' Kommando sofort ausführen
 *              c= '<' Ausführen, wenn abs. Position pp unterschritten ( in Steps vom Ref.Punkt )
 *              c= '>' Ausführen, wenn abs, Position pp überschritten ( in Steps vom Ref. Punkt )
 *              c= 'm' Ausführen, wenn Restweg unter pp fällt ( in % )
 *              c= 's' Ausführen, wenn Restweg unter pp fällt ( in Steps )
 *              c= 't' nach pp Millisekunden ausführen
 *              c= 'p' Ausführen, wenn Pin pp HIGH, bzw -pp LOW ist
 *              c= '?' Abarbeitung stoppen
 *              ccc nn rr = eines der obigen Kommandos
 *  els         -> Kommandos aus EEPROM auflisten
 *  
*/

#include <MobaTools.h>
#include <EEPROM.h>

// Definition der Pins für verschiedene Platformen
//==========================================================================
#define stepMode  STEPDIR
const int stPerRev = 400;
#ifdef ESP8266  //===================== ESP8266 ========================
const byte A4988Step=14, A4988Dir=2 ;
#define enablePin 12
// SPI1 = Pins MOSI=PA7, MISO=PA6, SCK=PA5, NSS=PA4
// LA-Pins: TP1=PB12, TP2=PB13, TP3= PB14,  TP4=BP15
//............................................................................
#elif defined __STM32F1__  //===================== STM32F1 ========================
const byte A4988Step=PB8, A4988Dir=PB5 ;
#define enablePin PB9
// SPI1 = Pins MOSI=PA7, MISO=PA6, SCK=PA5, NSS=PA4
// LA-Pins: TP1=PB12, TP2=PB13, TP3= PB14,  TP4=BP15
//............................................................................
#elif defined __AVR_ATmega328P__ // ========- 328P ( Nano, Uno Mega ) ========--
const byte A4988Step=6, A4988Dir=5;
const byte stepPins[] = {16,18,17,19 };
#define enablePin 7
// SPI = Pins 10,11,12,13
// LA-Pins: TP1=A1, TP2=A2, TP3= A3,  TP4=A4
//............................................................................
#elif defined __AVR_ATmega32U4__ // ====- 32U4 ( Micro, Leonardo ) ================
const byte A4988Step=6, A4988Dir=5;
// SPI = Pins 14,15,16,17
// LA-Pins: TP1=A3, TP2=A2, TP3= D1,  TP4=D0
#endif
//====================================== Ende Pin-Definitionen ======================
#ifdef ESP8266
    MoToStepper  myStepper(stPerRev );
#else
    MoToStepper  myStepper(stPerRev, stepMode );
#endif

#define printf( x, ... ) { char txtbuf[100]; sprintf_P( txtbuf, PSTR( x ), ##__VA_ARGS__ ) ; Serial.print( txtbuf ); }

// Tokens der Befehle
enum comTok { dstT, wraT, wrsT, rotT, sspT, sssT, srlT, stpT, movT, rdaT, rdsT, szpT, wrpT, estT, espT, eepT, elsT,nopT,stdT,enaT };
const char comStr[] = "dst,wra,wrs,rot,ssp,sss,srl,stp,mov,rda,rds,szp,wrp,est,esp,eep,els,nop,std,ena";
// Befehlsstruktur im EEPROM
#define EEMAX   64 // Zahl der einträge im EEPROM
typedef struct {
    char bedingung = '?';     // Bedingung für die Ausführung des Befehls
    long bedParam;      // Paramter für die Bedingung
    byte command;       // Auszuführendes Kommando
    long comPar1;       // Kommandoparameter
    long comPar2;       // Kommandoparamter
    uint16_t dummy;     // für spätere Erweiterung
} eeBefehl_t;
const byte eeComLen = sizeof( eeBefehl_t );

eeBefehl_t autoCom, interCom;      // aktuell abzuarbeitende Kommandos ( automatisch, interaktiv )
#ifdef __STM32F1__
eeBefehl_t cmdStorage[EEMAX]; // Auf dem STM32 werdein die Befehle im Ram gespeichert
#endif
enum  { ASTOPPED, NEXTCOM, WAITGT, WAITLT, WAITMV, WAITTM, WAITST, WAITPN } autoZustand;
EggTimer waitTimer;
int comIx = -1;

//--------------------- Funktionen --------------------------------------
void printEeBefehl ( eeBefehl_t &comline, int eeIx = -1 ) {
    char _cmdStr[4];
    strncpy( _cmdStr, &comStr[comline.command*4], 3 );
    _cmdStr[3] = 0;
    if ( eeIx >= 0 ) {
        // im eep-Format ausgeben ( kann per Copy/paste wieder als
        // Kommando verwendet werden
        printf("eep %2d %2c %5ld  %s %5ld %5ld " ,
            eeIx,
            comline.bedingung,
            comline.bedParam,
            _cmdStr,
            comline.comPar1,
            comline.comPar2   );
        Serial.println();
    } else {
        printf("| %2c %5ld | Cmd: %s %5ld %5ld|" ,
            comline.bedingung,
            comline.bedParam,
            _cmdStr,
            comline.comPar1,
            comline.comPar2   );
        Serial.println();
    }
}


void storeCmd( byte eeIx, eeBefehl_t &cmdBuf ) {
    #ifdef __STM32F1__ // beim STM32 im Ram speichern
    if ( eeIx >= 0 && eeIx < EEMAX ) {
        memcpy( &cmdStorage[eeIx], &cmdBuf, sizeof( eeBefehl_t ) );
    }
    #else
    EEPROM.put( eeIx*eeComLen, cmdBuf );
    #ifdef ESP8266
        Serial.print("Commit ");
    if ( EEPROM.commit() ) {
      Serial.println("OK");
    } else {
      Serial.println("NOK");
    }
    #endif
    #endif
}

void readCmd( byte eeIx, eeBefehl_t &cmdBuf ) {
    #ifdef __STM32F1__ // hier gibt es keinen get-Befehl, eine Stelle kann aber uint16 aufnehmen
    if ( eeIx >= 0 && eeIx < EEMAX ) {
        memcpy( &cmdBuf, &cmdStorage[eeIx], eeComLen );
    }
    #else
    EEPROM.get( eeIx*eeComLen, cmdBuf );
    #endif
    
}

#include "readCommands.h"


void setup() {
  Serial.begin( 115200 );
  while( !Serial ); 
  Serial.println("Programmstart");
  #ifdef ESP8266
  EEPROM.begin( 2048 );
  #endif
  #if (stepMode == A4988 ) || ( stepMode == STEPDIR )
    if (myStepper.attach( A4988Step, A4988Dir )  ) Serial.println("Attach A4988 OK"); else Serial.println("Attach A4988 NOK");
  #else
    if (myStepper.attach( stepPins[0],stepPins[1],stepPins[2],stepPins[3] )  ) Serial.println("Attach 4Wire OK"); else Serial.println("Attach A4988 NOK");
  #endif
  #ifdef enablePin
    myStepper.attachEnable(enablePin, 30, LOW );
  #endif
  myStepper.setSpeedSteps( 6000, 100 );
  delay( 500 );
  Serial.println( "Starting loop" );
}

void loop() {
    if ( getCmd( interCom ) ) {
        //printEeBefehl( interCom );
        Serial.print("Manu:");
        execCmd( interCom ) ;
    }

    // Automatische Befehlsabarbeitung aus EEPROM
    switch( autoZustand ) {
      case ASTOPPED:
        comIx = -1;
        break;
      case NEXTCOM:
         if ( comIx == -1 ){
            // Ablauf stoppen
            Serial.println("Auto: Angehalten");
            autoZustand = ASTOPPED;
         } else {
             readCmd( comIx++, autoCom );
             printf("Auto: (%2d) ", comIx-1 );
             if ( autoCom.bedingung == '-' ) {
                // Befehl sofort ausführen
                execCmd( autoCom );
             } else if ( autoCom.bedingung == '>' ) {
                // verzögerte Ausführung
                printf(" warte bis Steppos > %ld ->", autoCom.bedParam );
                autoZustand = WAITGT;
             } else if ( autoCom.bedingung == '<' ) {
                printf(" warte bis Steppos < %ld ->", autoCom.bedParam );
                autoZustand = WAITLT;
             } else if ( autoCom.bedingung == 'm' ) {
                printf( " warte bis Restweg <= %ld%% ->", autoCom.bedParam );
                autoZustand = WAITMV;
             } else if ( autoCom.bedingung == 's' ) {
                printf( " warte bis Restweg <= %d Steps ->", autoCom.bedParam );
                autoZustand = WAITST;
             } else if ( autoCom.bedingung == 'p' ) {
                printf( " warte bis Pin %d  ", autoCom.bedParam );
                if ( autoCom.bedParam > 0 ) Serial.print("HIGH ->");
                else                        Serial.print("LOW ->");
                autoZustand = WAITPN;
             } else if ( autoCom.bedingung == 't' ) {
                printf(" warte %ld ms ->", autoCom.bedParam );
                autoZustand = WAITTM;
                waitTimer.setTime( autoCom.bedParam );
             } else {
                //kein gültiger Eintrag, Ablaufende
                Serial.println(" Ende");
                autoZustand = ASTOPPED;
             }
        }
        break;
      case WAITGT: // warten bis akt. Position > Param
        if ( myStepper.readSteps() > autoCom.bedParam ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITLT: // warten bis akt. Position < Param
        if ( myStepper.readSteps() < autoCom.bedParam ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITMV: // warten bis Restweg <= Param Prozent
        if ( myStepper.moving() <= autoCom.bedParam ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITST: // warten bis Restweg <= Param Steps
        if ( myStepper.stepsToDo() <= autoCom.bedParam ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITPN: // warten auf Pin Param
        if ( digitalRead( abs(autoCom.bedParam) ) == ( autoCom.bedParam > 0 ) ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
      case WAITTM: // warten auf Zeitablauf
        if ( !waitTimer.running() ) {
            execCmd( autoCom );
            autoZustand = NEXTCOM;
        }
        break;
    } //Ende Kommandoautomat
}
