// 9600 NMEA
uint8_t Lth=0;

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

  pinMode(rxPin, INPUT);      
  digitalWrite (rxPin, HIGH); // pull up enabled for noise reduction ?
    
  baud = GetBaud(rxPin); 
    
  String s_baud=String(baud);   // integer to string. "String" overloaded
  OledDisplayln("Found telem "+ s_baud+"b/s"); 
  
  #if defined Debug_All || defined Debug_Telemetry
     Debug.print("Telem found at ");  Debug.println(baud);
  #endif   

  LookForProtocol(baud);
 
  // Balance of probabilities - first to get to 10 occurances
  if (Mav1>10) return 1; 
  if (Mav2>10) return 2;
  if (FSky>10) return 3;
  if (LTM>10) return 4;
  if (MSP>10) return 5;
  if (NMEA_GPS>10) return 6;
  return 0;  // unsuccessful  
}

//***************************************************
uint16_t LookForProtocol(uint32_t baud) {
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
        Debug.println("."); 
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
uint32_t GetBaud(uint8_t  rxPin) {

uint32_t pw = 999999;  //  Pulse width in uS
uint32_t min_pw;
Debug.print("getBaud");
 for (int i = 0; i < 10; i++) {
   pw = pulseIn(rxPin,LOW);              // Returns the length of the pulse in microseconds
   min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
 }

  #if defined Debug_All || defined Debug_Baud
    Debug.print("pw="); Debug.print(pw); Debug.print("  min_pw="); Debug.println(min_pw);
  #endif
  switch(min_pw) {   
    case 0 ... 11:     
      return 115200;
    case 12 ... 19:  
      return 57600;
     case 20 ... 28:  
      return 38400; 
    case 29 ... 39:  
      return 28800;
    case 40 ... 59:  
      return 19200;
    case 60 ... 79:  
      return 14400;
    case 80 ... 149:  
      return 9600;
    case 150 ... 299:  
      return 4800;
     case 300 ... 599:  
      return 2400;
     case 600 ... 1199:  
      return 1200;                          
    default:  
      return 0;            
 }
} 
