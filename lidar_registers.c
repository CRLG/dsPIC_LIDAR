#include <xc.h>
#include "lidar_const.h"
#include "lidar_registers.h"
#include "eeprom.h"
#include "General.h"

 T_dsPIC_REGISTER dsPIC_reg[MAX_REGISTRES_NUMBER];

// ___________________________________________________________
void lidar_registers_init(void)
{
  unsigned short i=0;
  
  // Initialise de manière massive tous les registres
  for (i=0; i<MAX_REGISTRES_NUMBER; i++) {
     dsPIC_reg[i].val               = 0;
     dsPIC_reg[i].new_data          = 0;
     dsPIC_reg[i].type_read_write   = READ_ONLY; 
  }
  // Les registres Read/write
  for (i=REG_ENABLE_TELEMETER_1; i<MAX_REGISTRES_NUMBER; i++) {
    dsPIC_reg[i].type_read_write = READ_WRITE;     
  }

  // 
  for (i=0; i<NUMBER_MAX_OF_TELEMETERS; i++) {
        dsPIC_reg[REG_FORCE_LED1+i].val             = LED_CONTROLED_BY_LIDAR;
        dsPIC_reg[REG_DISTANCE_TELEMETER_1+i].val   = INFINITE_DISTANCE;
  }
  
  // Initialise les valeurs par défaut
  dsPIC_reg[REG_VERSION_SOFT_MAJ].val               = VERSION_SOFT_MAJ;
  dsPIC_reg[REG_VERSION_SOFT_MIN].val               = VERSION_SOFT_MIN;
  dsPIC_reg[REG_PTR_REG_LECTURE_I2C].val            = REG_DISTANCE_ALARM_TELEMETERS_H;
  dsPIC_reg[REG_NBRE_REGISTRES_LECTURE_I2C].val     = 12; // Nombre de registres lus par le MBED lors d'une opération de lecture
  dsPIC_reg[REG_EEPROM_WRITE_UNPROTECT].val         = 0;  // EEPROM protégée en écriture
  dsPIC_reg[REG_I2C_8BITS_ADDRESS].val              = EEPROM_values[EEPADDR_I2C_ADDRESS_8bits]; // restitution de la valeur configurée en eeprom
}   
 
 
void lidar_registers_management()
{
    unsigned char ucval;
    //unsigned short usval;
    // _____________________________________________________
    // Configuration adresse I2C
    if (dsPIC_reg[REG_I2C_8BITS_ADDRESS].new_data) {
ENTER_CRITICAL_SECTION_I2C()            
        ucval = dsPIC_reg[REG_I2C_8BITS_ADDRESS].val;
        dsPIC_reg[REG_I2C_8BITS_ADDRESS].new_data = 0;
LEAVE_CRITICAL_SECTION_I2C()
        EEPROM_values[EEPADDR_I2C_ADDRESS_8bits] = ucval;  // un reboot de la carte sera nécessaire. Pas de prise en compte immédiat pour ce changement
        saveEEPROM();  // l'EEPROM devra avoir été dévérouillée en écriture préalablement
     }    

    
    
}
