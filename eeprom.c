#include "eeprom.h"
#include "DEE_Emulation_16-bit.h"

unsigned short EEPROM_values[EEPROM_SIZE];
static unsigned char EEPROM_write_enable = 0;

// _______________________________________________________
void forceEEPROMDefaultValues()
{
 EEPROM_values[EEPADDR_MAGIC_NUMBER]            = EEPROM_MAGIC_NUMBER;

 // TODO : mettre les autres 
 //EEPROM_values[EEPADDR_I2C_ADDRESS_8bits]       = 
}

// _______________________________________________________
void Init_EEPROM()
{
 DataEEInit();
 dataEEFlags.val = 0;

 readEEPROM(); 
}

// ___________________________________________
void resetFactoryEEPROM()
{
  unsigned char tmp_write_enable = EEPROM_write_enable;
    
  forceEEPROMDefaultValues();
  EEPROM_write_enable = EEPROM_WRITE_ENABLED_CODE;  
  saveEEPROM();
  EEPROM_write_enable = tmp_write_enable;  
}

// ___________________________________________
void readEEPROM()
{
 int i; 
 for (i=0; i<EEPROM_SIZE; i++) {
   EEPROM_values[i] = DataEERead(i);
 }
 // EEPROM corrupted or never initialized
 if (EEPROM_values[EEPADDR_MAGIC_NUMBER] != EEPROM_MAGIC_NUMBER) {
    resetFactoryEEPROM();
 }
}

// ___________________________________________
void saveEEPROM()
{
 // Vérifie si l'EEPROM est bien dévérouillée avant d'écrire
 if (EEPROM_write_enable != EEPROM_WRITE_ENABLED_CODE) return;
 int i; 
 for (i=0; i<EEPROM_SIZE; i++) {
   DataEEWrite(EEPROM_values[i], i);
 }
}

// _______________________________________________________
void enableWriteEEPROM(unsigned char enable)
{
   EEPROM_write_enable = enable; 
}