
//********************************** E S P 3 2 *********************************

#if (defined ESP32) || (defined ESP8266)

#include <EEPROM.h>

#define EEPROM_SIZE 64

//***************************************************
void EEPROM_Setup() {
    if (!EEPROM.begin(EEPROM_SIZE))   // We use only 5
  {
    Log.println("EEPROM failed to initialise"); 
    LogScreenPrintln("EEPROM init failed!");
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
     Log.print("EEPROMWriteLong():"); 
     Log.print("  composit="); Log.print(value);     
     Log.print("  one="); Log.print(one, HEX);
     Log.print("  two="); Log.print(two, HEX);
     Log.print("  three="); Log.print(three, HEX);
     Log.print("  four="); Log.println(four, HEX);    
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
     Log.print("EEPROMReadLong():"); 
     Log.print("  one="); Log.print(one, HEX);
     Log.print("  two="); Log.print(two, HEX);
     Log.print("  three="); Log.print(three, HEX);
     Log.print("  four="); Log.print(four, HEX);    
     Log.print("  composit="); Log.println(composit);      
  #endif     

  return composit;
}

//***************************************************

void DisplayEEPROM() {
  Log.println("EEPROM:");

  for (int i = 0; i < EEPROM_SIZE; i++) {
    PrintByte(EEPROM.read(i)); Log.print(" ");
  }
  Log.println();
  
  for (int i = 0; i < EEPROM_SIZE; i++) {
    Log.print(byte(EEPROM.read(i))); Log.print(" ");
  }
  Log.println();
}

#endif


//*************************  S T M 3 2  &&  T e e n s y   3 . x ****************
#if (Target_Board == 0) || (Target_Board == 1)|| (Target_Board == 2)

#include <EEPROM.h>

/*******************************************************************************
 * STM32 Virtual EEPROM
 * 
 * The STM32F103C microcontrollers do not have EEPROM per se. This library maps 
 * virtual EEPROM into the last 2 kB of the flash memory. Note that the firmware 
 * flashing process will erase All flash memory, including the V EEPROM.
 * Library found here: https://github.com/rogerclarkmelbourne/Arduino_STM32
 * The library name EEPROM is ambiguous and could confuse references to Arduino
 * and ESP8266/ESP32 libraries of the same name.
 * 
 * Support for 32bit integers added by Eric Stockenstrom 
 * Read and write 32bit integers into and from virtual EEPROM on the STM32F1xx
 * Signed integers are supported but will naturally have 31 bit resolution
 *
 ********************************************************************************
*/ 

void EEPROM_Setup() {

}

//***************************************************

void EEPROMWritelong(uint16_t idx, int32_t val)  {  

  #if defined TEENSY3X || defined STM32F1xx
    byte b4 = (val & 0xFF);
    byte b3 = ((val >> 8) & 0xFF);
    byte b2 = ((val >> 16) & 0xFF);
    byte b1 = ((val >> 24) & 0xFF);
    EEPROM.write(idx, b1);
    EEPROM.write(idx+1, b2);  
    EEPROM.write(idx+2, b3);
    EEPROM.write(idx+3, b4);       
  //#else  // Maple? deprecated
  //  uint16_t addr=idx*2;
  //  uint16_t w2 = (val & 0xFFFF);
  //  uint16_t w1 = ((val >> 16) & 0xFFFF);
  //  EEPROM.write(addr, w2);
  //  EEPROM.write(addr+1, w1);
  #endif
  
  #if defined Debug_All || defined Debug_EEPROM
     Log.print("EEPROMWriteLong():"); 
     Log.print("  idx="); Log.print(idx);
     Log.print("  b1="); Log.print(b1, HEX);
     Log.print("  b2="); Log.print(b2, HEX);
     Log.print("  b3="); Log.print(b3, HEX);
     Log.print("  b4="); Log.print(b4, HEX);
     Log.print("  val="); Log.println(val);
  #endif
}

//*****************************************************  
    
int32_t EEPROMReadlong(uint16_t idx)  {
  
  uint32_t val = 0;
 
  #if defined TEENSY3X || defined STM32F1xx
    byte b1 = EEPROM.read(idx);
    byte b2 = EEPROM.read(idx+1);
    byte b3 = EEPROM.read(idx+2);
    byte b4 = EEPROM.read(idx+3);  
    val = b4  + (b3 << 8) + (b2 << 16) + (b1 << 24);  
  //#else
  //  uint16_t addr=idx*2;
  //  uint16_t two = 0;
  //  uint16_t one = 0;
  //  EEPROM.read(addr, &two);
  //  EEPROM.read(addr+1, &one);
  //  val = two  + (one << 16);
  #endif
    
  #if defined Debug_All || defined Debug_EEPROM
     Log.print("EEPROMReadLong():"); 
     Log.print("  idx="); Log.print(idx);     
     Log.print("  b1="); Log.print(b1, HEX);
     Log.print("  b2="); Log.print(b2, HEX);
     Log.print("  b3="); Log.print(b3, HEX);
     Log.print("  b4="); Log.print(b4, HEX);
     Log.print("  val="); Log.println(val);
  #endif
  
  return val;
}
     
#endif
