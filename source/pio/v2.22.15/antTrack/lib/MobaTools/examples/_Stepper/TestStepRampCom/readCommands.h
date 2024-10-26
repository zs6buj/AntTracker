

bool getCmd ( eeBefehl_t &cmdBuf ) {
    // Wenn Daten verfügbar, diese in den receive-Buffer lesen. Endezeichen ist LF oder CR
    // Funktionswert ist true, wenn ein kompletter Befehl empfangen, und im cmdBuf eingetragen wurde
    const byte ETX = 03;
    bool fulCmd = false;
    int8_t eeIx = -1;         // default: kein EEPROM Eintrag
    static char rcvBuf[50];
    static byte rcvIx=0;       // Index im Empfangspuffer
    char *token, *tkPtr;
    const char trenner[] = " ,\n\r";
    byte dataAnz = Serial.available();
    if ( dataAnz > 0 ) {
        Serial.readBytes( &rcvBuf[rcvIx], dataAnz );
        rcvIx += dataAnz;
        if ( rcvBuf[rcvIx-1] == 10 || rcvBuf[rcvIx-1] == 13 ) {
            rcvBuf[rcvIx] = ETX; // Endekennung (strtok bei ESP erkennt kein Stringende-> Absturz, wenn über Ende gelesen wird)
            rcvBuf[rcvIx+1] = 0;
            // komplette Zeile empfangen -> auswerten und in cmdBuf eintragen
            fulCmd = true;
            cmdBuf.bedingung = '-';
            token = strtok( rcvBuf, "\n\r ,");
            tkPtr = strstr( comStr, token );
            if ( tkPtr == NULL ) {
                // unbekanntes Kommando
                Serial.println("Kommando unbekannt");
                fulCmd = false;
            } else { // === gültiges Kommando empfangen ===
                // token berechnen
                cmdBuf.command = (comTok)( (int)(tkPtr-comStr) / 4 );
                // Prüfen, ob der Befehl ins EEPROM soll
               if ( cmdBuf.command == eepT ) { 
                    // EEprom-Index lesen
                    eeIx = atoi( strtok( NULL, trenner ) );
                    // Ausführungsbedingung einlesen
                    if ( *token != ETX ) token = strtok( NULL, trenner );
                    cmdBuf.bedingung = *token;
                    cmdBuf.bedParam  = atol( strtok( NULL, trenner) );
                    if ( *token != ETX ) token = strtok( NULL, trenner ); // ab hier Befehlsauswertung
                    tkPtr = strstr( comStr, token );
                    if ( tkPtr == NULL ) {
                        // unbekanntes Kommando, wie nop behandeln
                        cmdBuf.command = nopT;
                    } else {
                         cmdBuf.command = (comTok)( (int)(tkPtr-comStr) / 4 ); 
                    }
                }
                // Parameter einlesen
                if ( *token != ETX ) token = strtok( NULL, trenner );
                cmdBuf.comPar1 = atol( token );
                if ( *token != ETX ) token = strtok( NULL, trenner );
                if ( *token == ETX ) {
                    cmdBuf.comPar2 = -1;
                } else {
                    cmdBuf.comPar2 = atoi( token );
                }

                switch ( cmdBuf.command )  {     
                  case estT: // ===============================  est nnn     -> Automat an Index nn starten ======
                    if ( eeIx < 0 ) {
                        // war kein eep-Befehl -> Ablauf starten
                        comIx = cmdBuf.comPar1;
                        if ( comIx >= 0 && comIx <EEMAX ) autoZustand = NEXTCOM;
                        printf( "Starte autom. Ablauf ab Index %d\n\r", comIx );
                        fulCmd = false;
                    }
                    break;
                  case espT: // ===============================  esp     -> Automat stoppen ======
                    if ( cmdBuf.comPar1 == 0 )
                        comIx = -1; // nach aaktuellem Befehl anhalten
                    else {
                        Serial.println(" Abgebrochen");
                        autoZustand = ASTOPPED; // direkt anhalten
                    }
                    fulCmd = false;
                    break;
                  case elsT: // ===============================  els         -> list Commands ======================
                    // Kommandos aus EEPROM ausgeben ( ab 0 bis ungültiger Eintrag )
                    Serial.println();
                    eeIx = 0;
                    readCmd( eeIx, cmdBuf  );
                    fulCmd = true;  // wird hier temporär als Merker für Lücken in der Liste genutzt
                    while (eeIx < EEMAX ) {
                        if ( (strchr( "-<>mt!?", cmdBuf.bedingung ) != NULL) && cmdBuf.bedingung != 0  ) {
                            // nur gültioge Einträge anzeigen
                            // printf( "%02d: ", eeIx );
                            printEeBefehl( cmdBuf , eeIx);
                            fulCmd = true;
                        } else if ( fulCmd) {
                            // Trenner ausgeben
                            Serial.println( "----" );
                            fulCmd = false;
                        }
                        readCmd( ++eeIx, cmdBuf );
                    }
                    fulCmd = false; // Kommando nicht weiterleiten
                    break;
                  default: // alle regulären Kommandos brauchen keine weitere Verarbeitung
                    ;
                }
                // ------------- war eep-Befehl -> Kommando im Speicher eintragen --------------
                if ( fulCmd && eeIx >=0 && eeIx < EEMAX ) {
                    // Kommando in EEPROM eintragen
                    printf( "EEPROM-Ix = %d, ", eeIx );
                    storeCmd( eeIx, cmdBuf );
                    readCmd( eeIx, cmdBuf );
                    printEeBefehl( cmdBuf );
                    fulCmd = false; // Kommando nicht direkt ausführen
                }
            } // Ende Kommandoauswertung
            // Empfangspuffer rücksetzen
            rcvIx = 0;
        } // Ende komplette Zeile empfagen und auswerten
    }
    return fulCmd;
}

void execCmd( eeBefehl_t &cmdBuf ) {
    long steps;
    uint16_t ramp;
    switch ( cmdBuf.command ) {
      case szpT:
        printf(" Pos %ld -> neuer Nullpunkt\n\r", myStepper.readSteps() );
        myStepper.setZero();
        break;
      case dstT: // ===============================  dst nnn     -> doSteps( +/-nnnL ) ==========
        printf(" Move: %ld\n\r",cmdBuf.comPar1 );
        myStepper.doSteps( cmdBuf.comPar1 );
        break;
      case wraT: // ===============================  wra nnn     -> write( +/-angleL ) ===========
        printf(" MoveTo: %ld\n\r",cmdBuf.comPar1 );
        myStepper.write( cmdBuf.comPar1 );
        break;
      case wrsT: // ===============================  wrs nnn     -> writeSteps ( +/-stepsL ) ======
        printf(" MoveTo: %ld\n\r",cmdBuf.comPar1 );
        myStepper.writeSteps( cmdBuf.comPar1 );
        break;
      case wrpT: // ===============================  wrp        -> writeSteps ( readStreps() ) ======
        steps = myStepper.readSteps();
        printf(" MoveTo: %ld\n\r",steps );
        myStepper.writeSteps( steps );
        break;
      case rotT: // ===============================  rot d       -> rotate( +/-direction ); =======
        printf(" rotate: %ld\n\r",cmdBuf.comPar1  );
        myStepper.rotate( cmdBuf.comPar1  );
        break;
      case sspT: // ===============================  ssp nn      -> setSpeed( rpm10 ) ===============================
        printf(" Rpm: %u,%u ", cmdBuf.comPar1 /10, cmdBuf.comPar1 %10 );
        myStepper.setSpeed( cmdBuf.comPar1  );
        break;
      case sssT: // ===============================  sss nn rr   -> setSpeedSteps( speed10, ramp ) wenn rr 0 ist oder fehlt wird es nicht ausgegeben
        if ( cmdBuf.comPar2 < 0 ) {
            printf(" sss: %u,%u",cmdBuf.comPar1 /10, cmdBuf.comPar1 %10);
            ramp = myStepper.setSpeedSteps( cmdBuf.comPar1  );
        } else {
            printf(" sss: %ld,%ld ramp: %ld ",cmdBuf.comPar1 /10, cmdBuf.comPar1 %10, cmdBuf.comPar2 );
            ramp = myStepper.setSpeedSteps( cmdBuf.comPar1 , cmdBuf.comPar2  );
        }
        printf( "->akt.Rampe=%u\n\r", ramp );
        break;
     case stpT: // ===============================  stp         -> stop ===============================
        Serial.println( " Stop!" );
        myStepper.stop();
        break;
     case srlT: // ===============================  srl nn      -> setRampLen( ramp ) ================
        ramp = myStepper.setRampLen( cmdBuf.comPar1  );
        printf( " akt.Rampe=%u\n\r", ramp );
        break;
     case movT: // ===============================  mov         -> print restweg in % =================
        printf( " Remaining: %d%%\n\r", myStepper.moving() );
        break;
     case stdT: // ===============================  std         -> print restweg in Steps =============
        printf( " Remaining: %ld Steps\n\r", myStepper.stepsToDo() );
        break;
     case rdaT: // ===============================  rda         -> print anglepos =====================
       printf( " Winkelposition: %ld\n\r", myStepper.read() );
       break;
     case rdsT: // ===============================  rds         -> print steppos ======================
       printf( " Stepposition: %ld\n\r", myStepper.readSteps() );
       break;
     case estT: // =============================== est -> Programm bei schritt nn fortführen ========
       if ( cmdBuf.comPar1 >= 0 && cmdBuf.comPar1 < EEMAX ) {
           printf( " Goto %ld\n\r", cmdBuf.comPar1 );
           if ( comIx != -1 ) comIx = cmdBuf.comPar1 ;
       }
       break;
     case nopT:
       Serial.println( " NOP" );
       break;
     default:
       Serial.println("Kommando unbekannt");
    }
    
}
