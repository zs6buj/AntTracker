#include <Arduino.h>

#define crsf_invert     false
#define crsf_rxPin      27      //16 YELLOW rx from GREEN FC tx
#define crsf_txPin      17      // GREEN tx to YELLOW FC rx    


#define log   Serial

#define crsf_uart            1              // Serial1
#define crsf_baud           420000
//#define crsf_baud           115200        // EdgeTX output from RadioMaster TX16S AUX2
HardwareSerial crsfSerial(crsf_uart);       // instantiate Serial object

CRSF crsf;                                  // instantiate CRSF object

//===================================================
uint32_t SenseUart(uint8_t  pin) {

uint32_t pw = 999999;  //  Pulse width in uS
uint32_t min_pw = 999999;
uint32_t su_baud = 0;
const uint32_t su_timeout = 5000; // uS !  Default timeout 1000mS!

  #if defined DEBUG_All || defined DEBUG_BAUD
    log.printf("pin:%d  in_invert:%d\n", pin, in_invert);        
  #endif  

  if (crsf_invert) {
    while(digitalRead(pin) == 0){ };  // idle_low, wait for high bit (low pulse) to start
  } else {
    while(digitalRead(pin) == 1){ };  // idle_high, wait for low bit (high pulse) to start  
  }
  for (int i = 0; i < 10; i++) {

    if (crsf_invert) {               
      pw = pulseIn(pin,HIGH, su_timeout);     //  Returns the length of the pulse in uS
    } else {
      pw = pulseIn(pin,LOW, su_timeout);    
    }    

    if (pw !=0) {
      min_pw = (pw < min_pw) ? pw : min_pw;  // Choose the lowest
      //log.printf("i:%d  pw:%d  min_pw:%d\n", i, pw, min_pw);    
    } 
  } 
  #if defined DEBUG_All || defined DEBUG_BAUD
    log.printf("pw:%d  min_pw:%d\n", pw, min_pw);         
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

//======================================
uint32_t getConsistent(uint8_t pin) {
  uint32_t t_baud[5];

  while (true) {  
    t_baud[0] = SenseUart(pin);
    delay(10);
    t_baud[1] = SenseUart(pin);
    delay(10);
    t_baud[2] = SenseUart(pin);
    delay(10);
    t_baud[3] = SenseUart(pin);
    delay(10);
    t_baud[4] = SenseUart(pin);
    #define DEBUG_BAUD
    #if defined DEBUG_BAUD
      log.print("  t_baud[0]="); log.print(t_baud[0]);
      log.print("  t_baud[1]="); log.print(t_baud[1]);
      log.print("  t_baud[2]="); log.print(t_baud[2]);
      log.print("  t_baud[3]="); log.println(t_baud[3]);
    #endif  
    if (t_baud[0] == t_baud[1]) {
      if (t_baud[1] == t_baud[2]) {
        if (t_baud[2] == t_baud[3]) { 
          if (t_baud[3] == t_baud[4]) {   
            #if defined DEBUG_All || defined DEBUG_BAUD    
              log.print("Consistent baud found="); log.println(t_baud[3]); 
            #endif   
            return t_baud[3]; 
          }          
        }
      }
    }
  }
}

void printLoop1(bool newline)
{
  static uint32_t prev_lp1_millis = 0;
  uint32_t now_millis = millis();
  uint32_t period = now_millis - prev_lp1_millis;
  log.printf("Loop1 %3dmS ", period);
  if (newline)
    log.println();
  prev_lp1_millis = now_millis;
}

void setup() 
{
  log.begin(115200);
  delay(2000);

  log.printf("Sensing on rxPin:%u invert:%u\n", crsf_rxPin, crsf_invert);
  crsf.initialise(crsfSerial);

  getConsistent(crsf_rxPin);
}


void loop() 
{
    delay(1000);
}