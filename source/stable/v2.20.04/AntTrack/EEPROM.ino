#include <EEPROM.h>

void EEPROM_Setup() {
  
  #if (defined ESP32) || (defined ESP8266)  
    #define EEPROM_SIZE 64
    if (!EEPROM.begin(EEPROM_SIZE)) { // We use only 5
      log.println("EEPROM failed to initialise"); 
      LogScreenPrintln("EEPROM init failed!");
      while (true) delay(1000);
    }

    #if defined DEBUG_EEPROM
      DisplayEEPROM(); 
    #endif 
  #endif   
}

//***************************************************

void EEPROMWritelong(uint16_t idx, int32_t value) {
uint16_t addr=idx*4;
  //One = Most significant , Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(addr, four);
  EEPROM.write(addr + 1, three);
  EEPROM.write(addr + 2, two);
  EEPROM.write(addr + 3, one);
  #if (defined ESP32) || (defined ESP8266) 
    EEPROM.commit();
  #endif     
  #if defined DEBUG_All || defined DEBUG_EEPROM
     log.print("EEPROMWriteLong():"); 
     log.print("  composit="); log.print(value);     
     log.print("  one="); log.print(one, HEX);
     log.print("  two="); log.print(two, HEX);
     log.print("  three="); log.print(three, HEX);
     log.print("  four="); log.println(four, HEX);    
  #endif        
} 

//*****************************************************  

int32_t EEPROMReadlong(uint16_t idx) {
uint16_t addr=idx*4;     
      //One = Most significant , Four = Least significant byte
  uint32_t four = EEPROM.read(addr);
  uint32_t three = EEPROM.read(addr + 1);
  uint32_t two = EEPROM.read(addr + 2);
  uint32_t one = EEPROM.read(addr + 3);
  uint32_t composit = ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
  
  #if defined DEBUG_All || defined DEBUG_EEPROM
     log.print("EEPROMReadLong():"); 
     log.print("  one="); log.print(one, HEX);
     log.print("  two="); log.print(two, HEX);
     log.print("  three="); log.print(three, HEX);
     log.print("  four="); log.print(four, HEX);    
     log.print("  composit="); log.println(composit);      
  #endif     

  return composit;
}

//***************************************************
#if (defined ESP32) || (defined ESP8266) 
  void DisplayEEPROM() {
    log.println("EEPROM:");

    for (int i = 0; i < EEPROM_SIZE; i++) {
      printByte(EEPROM.read(i)); log.print(" ");
    }
    log.println();
  
    for (int i = 0; i < EEPROM_SIZE; i++) {
      log.print(byte(EEPROM.read(i))); log.print(" ");
    }
    log.println();
  }
#endif
