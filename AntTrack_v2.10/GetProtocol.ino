// 9600 NMEA
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

  baud = GetBaud(rxPin); 
  Debug.print("Baud rate detected is ");  Serial.print(baud); Serial.println(" b/s"); 
  String s_baud=String(baud);   // integer to string. "String" overloaded
  OledDisplayln("Telem found at "+ s_baud); 

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
  inSerial.begin(baud);

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
      Debug.print("Mav1=");  Debug.println(Mav1);
      Debug.print("Mav2=");  Debug.println(Mav2);
      Debug.print("FSky=");  Debug.println(FSky);
      Debug.print("LTM=");  Debug.println(LTM);
      Debug.print("MSP=");  Debug.println(MSP);
      Debug.print("NMEA_GPS=");  Debug.println(NMEA_GPS);
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
      #if defined Debug_All || defined Debug_Telemetry
        Debug.print("."); 
      #endif     
      OledDisplayln("Waiting for telemetry");
      q=0;
    }

    delay(100);
    Lth=inSerial.available();
  }
  q=0;
  // Data is available

  x = inSerial.read();
  
  r++;
  
  #if defined Debug_All || defined Debug_Telemetry
   // Debug.print((char)x);   
     DisplayByte(x);
   
   if ((r % 50) == 0) {
      Debug.println();
      r = 0;
    }
  #endif  
  
  return x;
}
// **********************************************************
uint32_t GetBaud(uint8_t rxPin) {
  
  pinMode(rxPin, INPUT);       
  digitalWrite (rxPin, HIGH); // pull up enabled for noise reduction ?
  
  uint32_t gb_baud = GetConsistent(rxPin);
  while (gb_baud == 0) {
    if(ftGetBaud) {
      ftGetBaud = false;
    }
    Debug.print("."); 
    gb_baud = GetConsistent(rxPin);
  } 
  if (!ftGetBaud) {
    Debug.println();
  }

  return(gb_baud);
}

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
      Debug.print("  t_baud[0]="); Debug.print(t_baud[0]);
      Debug.print("  t_baud[1]="); Debug.print(t_baud[1]);
      Debug.print("  t_baud[2]="); Debug.print(t_baud[2]);
      Debug.print("  t_baud[3]="); Debug.println(t_baud[3]);
    #endif  
    if (t_baud[0] == t_baud[1]) {
      if (t_baud[1] == t_baud[2]) {
        if (t_baud[2] == t_baud[3]) { 
          if (t_baud[3] == t_baud[4]) {   
            #if defined Debug_All || defined Debug_Baud    
              Debug.print("Consistent baud found="); Debug.println(t_baud[3]); 
            #endif   
            return t_baud[3]; 
          }          
        }
      }
    }
  }
}

uint32_t SenseUart(uint8_t  rxPin) {

uint32_t pw = 999999;  //  Pulse width in uS
uint32_t min_pw = 999999;
uint32_t su_baud = 0;



#if defined Debug_All || defined Debug_Baud
//  Debug.print("rxPin ");  Debug.println(rxPin);
#endif  

 while(digitalRead(rxPin) == 0) {
  if (rxFT) {
   rxFT = false;
   Debug.println("Waiting for telemetry"); 
   OledDisplayln("Waiting for telemetry");
   delay(50);
   }
 }
 
 while(digitalRead(rxPin) == 1){} // wait for low bit to start
 
  for (int i = 0; i < 10; i++) {
    pw = pulseIn(rxPin,LOW);               // Returns the length of the pulse in microseconds
    if (pw !=0) {
      min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
    } else {
       return 0;  // timeout - no telemetry
    }
  }
 
  #if defined Debug_All || defined Debug_Baud
 //   Debug.print("pw="); Debug.print(pw); Debug.print("  min_pw="); Debug.println(min_pw);
  #endif

  switch(min_pw) {   
    case 3 ... 11:     
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
     su_baud = 0;            
 }

 return su_baud;
} 
