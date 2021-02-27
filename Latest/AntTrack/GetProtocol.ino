// 9600 NMEA
#if (Telemetry_In == 0)    //  Serial
uint16_t Lth=0;

uint8_t Mav1 = 0;
uint8_t Mav2 = 0;
uint8_t FSky = 0;
uint8_t LTM = 0;
uint8_t MSP = 0;
uint8_t NMEA_GPS = 0;

uint16_t  q = 0;
uint16_t  r = 0;



char ch[3]; // rolling input bytes

// **********************************************************
uint8_t GetProtocol() { 
  
    typedef enum polarity_set { idle_low = 0, idle_high = 1 } pol_t;

    #if (defined ESP32) || (defined ESP8266)
      pol_t pol = (pol_t)getPolarity();
      if (pol == idle_low) {
        rxInvert = true;
        Log.printf("Serial port rx pin %d is idle_low, inverting rx polarity\n", rxPin);
      } else {
        rxInvert= false;
        Log.printf("Serial port rx pin %d is idle_high, regular rx polarity retained\n", rxPin);        
      }
   #endif
   
   #if defined AutoBaud
      baud = getBaud(rxPin);
      Log.print("Baud rate detected is ");  Serial.print(baud); Serial.println(" b/s"); 
      String s_baud=String(baud);   // integer to string. "String" overloaded
      LogScreenPrintln("Telem found at "+ s_baud);   
   #else 
     baud = 57600;  // default if not autobaud
     Log.print("Default baud rate is ");  Serial.print(baud); Serial.println(" b/s"); 
     String s_baud=String(baud);   // integer to string. "String" overloaded
     LogScreenPrintln("Default b/s:"+ s_baud);  
   #endif  
   DetectProtocol(baud);

   // Balance of probabilities - first to get to 10 occurances
   if (Mav1>10) { 
     return 1; 
     }
   if (Mav2>10) {
     return 2;
     }
   if (FSky>10) {
     return 3;
     }
   if (LTM>10) {
     return 4;
     }
   if (MSP>10) {
     return 5;
   }
   if (NMEA_GPS>10) {
     return 6;
     }
   return 0;  // unsuccessful  
 
}

//***************************************************
uint16_t DetectProtocol(uint32_t baud) {

    #if ( (defined ESP8266) || (defined ESP32) ) 
      inSerial.begin(baud, SERIAL_8N1, rxPin, txPin, rxInvert); 
    #elif (defined TEENSY3X) 
      frSerial.begin(frBaud); // Teensy 3.x    tx pin hard wired
       if (rxInvert) {          // For S.Port not F.Port
         UART0_C3 = 0x10;       // Invert Serial1 Tx levels
         UART0_S2 = 0x10;       // Invert Serial1 Rx levels;       
       }
       #if (defined frOneWire )  // default
         UART0_C1 = 0xA0;        // Switch Serial1 to single wire (half-duplex) mode  
       #endif    
    #else
      inSerial.begin(baud);
    #endif     
  
    chr = NxtChar();
    while (Lth>0) {  //  ignore 0x00 for this routine
      
      if (chr==0xFE) {   
        int pl = NxtChar();   // Payload length
        if (pl>=8 && pl<=37) Mav1++;             // Found candidate Mavlink 1
        }
      if (chr==0xFD) Mav2++;                     // Found candidate Mavlink 2
      if (chr==0x7E) FSky++;                     // Found candidate FrSky
      
      /*
      if (chr==0x24) {    //  'S'
        chr = NxtChar(); // start2 should be 0x54 ('T')
        if (chr==0x54) LTM++;                     // Found candidate LTM
      }
     */
     
      if ((ch[0]=='$') && (ch[1]=='T') && (ch[2]=='<')) LTM++;       // Found candidate LTM
      
      if ((ch[0]=='$') && (ch[1]=='M') && (ch[2]=='<')) MSP++;       // Found candidate MSP 

      if ((ch[0]=='$') && (ch[1]=='G') && (ch[2]=='P')) NMEA_GPS++; // Found candidate GPS

     // Balance of probabilities - first to get to 10 occurances  
    if ((Mav1>10) || (Mav2>10) || (FSky>10) || (LTM>10) || (MSP>10) || (NMEA_GPS>10) ){
      inSerial.end(); 
      return baud; 
    }

    #if defined Debug_All || defined Debug_Protocol
      Log.print("Mav1=");  Log.println(Mav1);
      Log.print("Mav2=");  Log.println(Mav2);
      Log.print("FSky=");  Log.println(FSky);
      Log.print("LTM=");  Log.println(LTM);
      Log.print("MSP=");  Log.println(MSP);
      Log.print("NMEA_GPS=");  Log.println(NMEA_GPS);
    #endif  
  
    chr = NxtChar(); 
    ch[0] = ch[1];   // rolling input bytes
    ch[1] = ch[2];
    ch[2] = chr;
      
    }
   delay(5); 
}

//***************************************************
byte NxtChar() {
byte x;

  Lth=inSerial.available();     //   wait for more data

  while (Lth==0) {
 
    q++;      
    if (q>100) {
      #if defined Debug_All || defined Debug_Protocol
        Log.print("."); 
      #endif     
      LogScreenPrintln("No telemetry");
      q=0;
    }

    delay(100);
    Lth=inSerial.available();
  }
  q=0;
  // Data is available

  x = inSerial.read();
  
  r++;
  
  #if defined Debug_All || defined Debug_Protocol
   // Log.print((char)x);   
     PrintByte(x);
   
   if ((r % 50) == 0) {
      Log.println();
      r = 0;
    }
  #endif  
  
  return x;
}
// **********************************************************
uint32_t getBaud(uint8_t rxPin) {
  Log.print("AutoBaud - Sensing rxPin "); Log.println(rxPin);
 // Log.printf("AutoBaud - Sensing rxPin %2d \n", rxPin );
  uint8_t i = 0;
  uint8_t col = 0;
  pinMode(rxPin, INPUT);       
  digitalWrite (rxPin, HIGH); // pull up enabled for noise reduction ?

  uint32_t gb_baud = GetConsistent(rxPin);
  while (gb_baud == 0) {
    if(ftgetBaud) {
      ftgetBaud = false;
    }

    i++;
    if ((i % 5) == 0) {
      Log.print(".");
      col++; 
    }
    if (col > 60) {
      Log.println(); 
        Log.print("No telemetry found on pin "); Log.println(rxPin);
  //    Log.printf("No telemetry found on pin %2d\n", rxPin); 
      col = 0;
      i = 0;
    }
    gb_baud = GetConsistent(rxPin);
  } 
  if (!ftgetBaud) {
    Log.println();
  }

  Log.print("Telem found at "); Log.print(gb_baud);  Log.println(" b/s");
  LogScreenPrintln("Telem found at " + String(gb_baud));

  return(gb_baud);
}
//************************************************
uint32_t GetConsistent(uint8_t rxPin) {
  uint32_t t_baud[5];

  while (true) {  
    t_baud[0] = SenseUart(rxPin);
    delay(10);
    t_baud[1] = SenseUart(rxPin);
    delay(10);
    t_baud[2] = SenseUart(rxPin);
    delay(10);
    t_baud[3] = SenseUart(rxPin);
    delay(10);
    t_baud[4] = SenseUart(rxPin);
    #if defined Debug_All || defined Debug_Baud
      Log.print("  t_baud[0]="); Log.print(t_baud[0]);
      Log.print("  t_baud[1]="); Log.print(t_baud[1]);
      Log.print("  t_baud[2]="); Log.print(t_baud[2]);
      Log.print("  t_baud[3]="); Log.println(t_baud[3]);
    #endif  
    if (t_baud[0] == t_baud[1]) {
      if (t_baud[1] == t_baud[2]) {
        if (t_baud[2] == t_baud[3]) { 
          if (t_baud[3] == t_baud[4]) {   
            #if defined Debug_All || defined Debug_Baud    
              Log.print("Consistent baud found="); Log.println(t_baud[3]); 
            #endif   
            return t_baud[3]; 
          }          
        }
      }
    }
  }
}
//************************************************
uint32_t SenseUart(uint8_t  rxPin) {

uint32_t pw = 999999;  //  Pulse width in uS
uint32_t min_pw = 999999;
uint32_t su_baud = 0;
const uint32_t su_timeout = 5000; // uS !

#if defined Debug_All || defined Debug_Baud
  Log.print("rxPin ");  Log.println(rxPin);
#endif  

  while(digitalRead(rxPin) == 1){ }  // wait for low bit to start
  
  for (int i = 0; i < 10; i++) {
    pw = pulseIn(rxPin,LOW, su_timeout);     // default timeout 1000mS! Returns the length of the pulse in uS

    if (pw !=0) {
      min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
    } else {
       return 0;  // timeout - no telemetry
    }
  }
 
  #if defined Debug_All || defined Debug_Baud
    Log.print("pw="); Log.print(pw); Log.print("  min_pw="); Log.println(min_pw);
  #endif

  switch(min_pw) {   
    case 1:     
     su_baud = 921600;
      break;
    case 2:     
     su_baud = 460800;
      break;     
    case 4 ... 11:     
     su_baud = 115200;
      break;
    case 12 ... 19:  
     su_baud = 57600;
      break;
     case 20 ... 28:  
     su_baud = 38400;
      break; 
    case 29 ... 39:  
     su_baud = 28800;
      break;
    case 40 ... 59:  
     su_baud = 19200;
      break;
    case 60 ... 79:  
     su_baud = 14400;
      break;
    case 80 ... 149:  
     su_baud = 9600;
      break;
    case 150 ... 299:  
     su_baud = 4800;
      break;
     case 300 ... 599:  
     su_baud = 2400;
      break;
     case 600 ... 1199:  
     su_baud = 1200;  
      break;                        
    default:  
     su_baud = 0;    // no signal        
 }

 return su_baud;
} 
#endif
//*************************************************************
bool getPolarity() { 
  uint16_t hi = 0;
  uint16_t lo = 0;  
  
  while ((lo <= 500) && (hi <= 500)) {

      if (digitalRead(rxPin) == 0) { 
       lo++;
      } else 
      if (digitalRead(rxPin) == 1) {  
       hi++;
      }

      if (lo > 500) return 0;
      if (hi > 500) return 1;
      
       //Log.printf("%d\t\t%d\n",lo, hi);  
   } 
}
