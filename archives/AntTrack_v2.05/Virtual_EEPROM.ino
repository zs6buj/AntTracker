#include <EEPROM.h>
/*******************************************************************************
 * Virtual EEPROM
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
void prepareEEPROM() {
  /*
// Otherwise defaults to page 0 = 0x801F800 and page 1 = 0x801FC00 = 128KB
// So page 1 is then outside the 128 KB. This is Ok if you don't use page 1.
// Each page is 0x400 or 1024 Bytes
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;  // 1024

  EEPROM.init();
  */
}

//***************************************************
void LostPowerCheckAndRestore(uint32_t epochNow) {
  epochBase = epochNow;
  eeprom_millis = millis();

  #if defined Debug_All || defined Debug_Time 
    Debug.print("Checking for RestoreHomeEEPRO conditions:"); 
    Debug.print("  homeInitialised="); Debug.print(homeInitialised);
    Debug.print("  timeGood="); Debug.print(timeGood);
    Debug.print("  epochNow="); Debug.print(TimeString(epochNow));
    uint32_t s_epoch= StoredEpoch();
    Debug.print("  StoredEpoch="); Debug.println(TimeString(s_epoch));
  #endif 

  if (!timeGood) {
    timeGood = true; 
    if ((homeInitialised==0)&& eepromGood && ((epochNow -  StoredEpoch()) < 3600)) {  //  within 3 minutes
      RestoreHomeFromEEPROM();
      homeInitialised=1;   
    }        
  } 
}
//***************************************************
void SaveHomeInEEPROM() {
  EEPROMWritelong(0, true);         // eepromGood erased to 0xFFFFFFFF during firmware flashing
  EEPROMWritelong(1, hom.lon*1E6);  // float to long
  EEPROMWritelong(2, hom.lat*1E6);
  EEPROMWritelong(3, hom.alt*10);
  EEPROMWritelong(4, hom.hdg*10);  
#if defined Debug_All || defined Debug_EEPROM
  Debug.print("home.lon="); Debug.print(hom.lon, 6);
  Debug.print("home.lat="); Debug.print(hom.lat, 6);
  Debug.print("home.alt="); Debug.print(hom.alt, 1);
  Debug.print("home.hdg="); Debug.print(hom.hdg, 1);
  DisplayPages(32);
#endif   
}

//***************************************************
void SaveEpochNow() {

  uint32_t epochNow = epochBase + (millis() - eeprom_millis) / 1E3;
  EEPROMWritelong(5, epochNow); // Seconds
  eeprom_millis = millis();
  
  #if defined Debug_All || defined Debug_EEPROM
  Debug.print("Epoch saved EpochNow="); Debug.println(TimeString(epochNow));
  DisplayPages(32);
#endif  
}

//***************************************************
uint32_t StoredEpoch() {

uint32_t stored_epoch = EEPROMReadlong(5);

 #if defined Debug_All || defined Debug_EEPROM
//   Debug.print("Stored epoch="); Debug.println(TimeString(epochBase));
//   DisplayPages(32);
 #endif
 return stored_epoch;    
}

//***************************************************
void RestoreHomeFromEEPROM() {

  eepromGood = EEPROMReadlong(0);
  hom.lon = EEPROMReadlong(1) / 1E6; //long back to float
  hom.lat = EEPROMReadlong(2) / 1E6;
  hom.alt = EEPROMReadlong(3) / 1E6;
  hom.hdg = EEPROMReadlong(4) / 1E6;
  OledDisplayln("Home co-ords restored");
  OledDisplayln("from EEPROM");
  #if defined Debug_All || defined Debug_EEPROM
  Debug.print("Home co-ords restored from EEPROM"); 
  Debug.print(  "home.lon="); Debug.print(hom.lon, 6);
  Debug.print("  home.lat="); Debug.print(hom.lat, 6);
  Debug.print("  home.alt="); Debug.print(hom.alt, 1);
  Debug.print("  home.hdg="); Debug.println(hom.hdg, 1);
  DisplayPages(32);
#endif  
}

//***************************************************

void EEPROMWritelong(uint16_t idx, uint32_t value)  {  
uint16_t addr=idx*2;

      //One = Most significant -> Two = Least significant word
      uint16_t two = (value & 0xFFFF);
      uint16_t one = ((value >> 16) & 0xFFFF);

      //Write the 2 words into the eeprom memory.
      EEPROM.write(addr, two);
      EEPROM.write(addr+1, one);
}

//*****************************************************  
    
uint32 EEPROMReadlong(uint16_t idx)  {
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
     
//***************************************************
#if defined Debug_All || defined Debug_EEPROM
 
void DisplayPages(uint32 endIndex) {
 
  Serial.println("Page 0     Top         Page 1");

  for (uint32 idx = 0; idx < endIndex; idx += 4)
  {
    Serial.print  (EEPROM.PageBase0 + idx, HEX);
    Serial.print  (" : ");
    DisplayHex(*(uint16*)(EEPROM.PageBase0 + idx));
    Serial.print  (" ");
    DisplayHex(*(uint16*)(EEPROM.PageBase0 + idx + 2));
    Serial.print  ("    ");
    Serial.print  (EEPROM.PageBase1 + idx, HEX);
    Serial.print  (" : ");
    DisplayHex(*(uint16*)(EEPROM.PageBase1 + idx));
    Serial.print  (" ");
    DisplayHex(*(uint16*)(EEPROM.PageBase1 + idx + 2));
    Serial.println();
  }
}
void DisplayHex(uint16 value)  {

  if (value <= 0xF)
    Serial.print("000");
  else if (value <= 0xFF)
    Serial.print("00");
  else if (value <= 0xFFF)
    Serial.print("0");
  Serial.print(value, HEX);
}
#endif
