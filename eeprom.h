#ifndef _EEPROM_H_
#define _EEPROM_H_

// Structures and defines
typedef enum {
    EEPADDR_I2C_ADDRESS_8bits = 0,
    
    // _________
    EEPADDR_MAGIC_NUMBER,
    EEPROM_SIZE
}tEEPROM_addresses;

#define EEPROM_MAGIC_NUMBER         (0x5A96)
#define EEPROM_WRITE_ENABLED_CODE   (0x5A)
#define EEPROM_RESET_FACTORY_CODE   (0x69)

// Functions prototypes
void Init_EEPROM(); 
void readEEPROM();
void saveEEPROM();
void forceEEPROMDefaultValues();
void resetFactoryEEPROM();

// External data
extern unsigned short EEPROM_values[];




#endif // _EEPROM_H_
