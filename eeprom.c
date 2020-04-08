#include "eeprom.h"
#include "DEE_Emulation_16-bit.h"
#include "General.h"
#include "lidar_const.h"

unsigned short EEPROM_values[EEPROM_SIZE];
static unsigned char EEPROM_write_enable = 0;

// _______________________________________________________
void forceEEPROMDefaultValues()
{
 EEPROM_values[EEPADDR_MAGIC_NUMBER]                    = EEPROM_MAGIC_NUMBER;
 EEPROM_values[EEPADDR_I2C_ADDRESS_8bits]               = I2C_DEFAULT_ADDRESS_8bits;
 EEPROM_values[EEPADDR_ENABLED_TELEMETER_BITFIELD]      = 0xFFFF;  // Tous les télémètres actifs
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_1]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_2]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_3]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_4]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_5]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_6]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_7]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_8]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_9]     = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_10]    = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_12]    = INFINITE_DISTANCE;
 EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_12]    = INFINITE_DISTANCE;
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
 unsigned short checksum=0; 
 for (i=0; i<EEPROM_SIZE; i++) {
   EEPROM_values[i] = DataEERead(i);
   checksum += EEPROM_values[i];
 }
 // EEPROM corrupted or never initialized
 if ((EEPROM_values[EEPADDR_MAGIC_NUMBER] != EEPROM_MAGIC_NUMBER) || 
     (checksum != DataEERead(EEPADDR_CHECKSUM)) )
 {
    resetFactoryEEPROM();
 }
}

// ___________________________________________
void saveEEPROM()
{
 // Vérifie si l'EEPROM est bien dévérouillée avant d'écrire
 if (EEPROM_write_enable != EEPROM_WRITE_ENABLED_CODE) return;
 unsigned short checksum=0; 
 int i; 
 for (i=0; i<EEPROM_SIZE; i++) {
   DataEEWrite(EEPROM_values[i], i);
   checksum += EEPROM_values[i];
 }
 DataEEWrite(checksum, EEPADDR_CHECKSUM);
}

// _______________________________________________________
void enableWriteEEPROM(unsigned char enable)
{
   EEPROM_write_enable = enable; 
}