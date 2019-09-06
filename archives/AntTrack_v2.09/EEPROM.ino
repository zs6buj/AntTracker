
//********************************** E S P 3 2 *********************************

#if (Target_Board == 3) 

#include <EEPROM.h>

#define EEPROM_SIZE 64

//***************************************************
void EEPROM_Setup() {
    if (!EEPROM.begin(EEPROM_SIZE))   // We use only 5
  {
    Debug.println("EEPROM failed to initialise"); 
    OledDisplayln("EEPROM init failed!");
    while (true) delay(1000);
  }
 #if defined Debug_EEPROM
   DisplayEEPROM(); 
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
  EEPROM.commit();   
  #if defined Debug_All || defined Debug_EEPROM
     Serial.print("EEPROMWriteLong():"); 
     Serial.print("  composit="); Serial.print(value);     
     Serial.print("  one="); Serial.print(one, HEX);
     Serial.print("  two="); Serial.print(two, HEX);
     Serial.print("  three="); Serial.print(three, HEX);
     Serial.print("  four="); Serial.println(four, HEX);    
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
  
  #if defined Debug_All || defined Debug_EEPROM
     Serial.print("EEPROMReadLong():"); 
     Serial.print("  one="); Serial.print(one, HEX);
     Serial.print("  two="); Serial.print(two, HEX);
     Serial.print("  three="); Serial.print(three, HEX);
     Serial.print("  four="); Serial.print(four, HEX);    
     Serial.print("  composit="); Serial.println(composit);      
  #endif     

  return composit;
}

//***************************************************

void DisplayEEPROM() {
  Debug.println("EEPROM:");

  for (int i = 0; i < EEPROM_SIZE; i++) {
    DisplayByte(EEPROM.read(i)); Serial.print(" ");
  }
  Debug.println();
  
  for (int i = 0; i < EEPROM_SIZE; i++) {
    Serial.print(byte(EEPROM.read(i))); Serial.print(" ");
  }
  Debug.println();
}

#endif


//********************************** S T M 3 2 *********************************
#if (Target_Board == 0) || (Target_Board == 1)|| (Target_Board == 2)

#include <EEPROM.h>

/*******************************************************************************
 * STP32 Virtual EEPROM
 * 
 * The STM32F103C microcontrollers do not have EEPROM per se. This library maps 
 * virtual EEPROM into the last 2 kB of the flash memory. Note that the firmware 
 * flashing process will erase All flash memory, including the V EEPROM.
 * Library found here: https://github.com/rogerclarkmelbourne/Arduino_STM32
 * The library name EEPROM is ambiguous and could confuse references to Arduino
 * and ESP8266/ESP32 libraries of the same name.
 * 
 * Support for 32bit integers added by Eric Stockenstrom 
 * Read and write 32bit integers into and from virtual EEPROM on the STM32F103C
 * Signed integers are supported but will naturally have 31 bit resolution
 *
 ********************************************************************************
*/ 

void EEPROM_Setup() {

}

//***************************************************

void EEPROMWritelong(uint16_t idx, int32_t value)  {  
uint16_t addr=idx*2;

      //One = Most significant -> Two = Least significant word
      uint16_t two = (value & 0xFFFF);
      uint16_t one = ((value >> 16) & 0xFFFF);

      //Write the 2 words into the eeprom memory.
      EEPROM.write(addr, two);
      EEPROM.write(addr+1, one);
}

//*****************************************************  
    
int32 EEPROMReadlong(uint16_t idx)  {
uint16_t addr=idx*2;
uint16_t two;
uint16_t one;
  
      //Read the 2 words from the eeprom memory.

    EEPROM.read(addr, &two);
    EEPROM.read(addr+1, &one);
  #if defined Debug_All || defined Debug_EEPROM
     Serial.print("EEPROMReadLong():"); 
     Serial.print("  one="); Serial.print(one, HEX);
     Serial.print("  two="); Serial.println(two, HEX);
  #endif
      //Return the recomposed long by using bitshift.
      return two  + (one << 16);
  }
     


#endif
