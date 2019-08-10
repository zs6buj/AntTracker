
uint8_t Lth=0;

uint8_t Mav1 = 0;
uint8_t Mav2 = 0;
uint8_t FSky = 0;
uint8_t LTM = 0;
uint8_t MSP = 0;

uint16_t  q = 0;
uint16_t  r = 0;
uint8_t rxPin = 3;  // rx pin on Serial 1 on Blue Pill

char ch[3];  // my private end token

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
      if (chr==0x24) {
        chr = NxtChar(); // start2 should be 0x54 
        if (chr==0x54) LTM++;                     // Found candidate LTM
      }
      if (chr=='$') {     // '$' also = 0x24 
        chr = NxtChar(); // next should be 'M'
        if (chr=='M') {
          chr = NxtChar(); // next should be '<'   incoming
          if (chr=='<') MSP++;                     // Found candidate MSP
        }
      }
    
     // Balance of probabilities - first to get to 10 occurances  
    if ((Mav1>10) || (Mav2>10) || (FSky>10) || (LTM>10) || (MSP>10)){
      inSerial.end(); 
      return baud; 
    }
  
    chr = NxtChar();  
      
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
 // Debug.print("r="); Debug.print(r);     Debug.print("  Lth="); Debug.println(Lth);
    DisplayByte(x);
    if ((r % 50) == 0) {
      Debug.println();
      r = 0;
    }
  #endif  
  
  return x;
}

uint32_t GetBaud(uint8_t  rxPin) {

uint32_t pw = 999999;  //  Pulse width in uS
uint32_t min_pw;

 for (int i = 0; i < 10; i++) {
   pw = pulseIn(rxPin,LOW);              // Returns the length of the pulse in microseconds
   min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
 }

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
