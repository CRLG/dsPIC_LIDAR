#ifndef _LIDAR_CONST_H_
#define	_LIDAR_CONST_H_

#define NUMBER_OF_TELEMETERS 12

typedef enum {
    STATUS_OK = 0,
    STATUS_DEGRADED,
    STATUS_ERROR
}T_SYSTEM_STATUS;

typedef enum {
   TELEM_OK = 0,
   TELEM_INHIBED,
   TELEM_AUTOTEST_ERROR
}T_TELEMETER_STATUS;

#define INFINITE_DISTANCE   (0xFF)

typedef enum {
    // ============================================
    // Read Only registers
    // ============================================
    REG_VERSION_SOFT_MAJ = 0,
    REG_VERSION_SOFT_MIN,
    // Status général 
    REG_SYSTEM_STATUS,                  // voir enum T_SYSTEM_STATUS
    // Status pour chaque télémètre
    REG_STATUS_TELEMETER_1,             // voir enum T_TELEMETER_STATUS
    REG_STATUS_TELEMETER_2,
    REG_STATUS_TELEMETER_3,
    REG_STATUS_TELEMETER_4,
    REG_STATUS_TELEMETER_5,
    REG_STATUS_TELEMETER_6,
    REG_STATUS_TELEMETER_7,
    REG_STATUS_TELEMETER_8,
    REG_STATUS_TELEMETER_9,
    REG_STATUS_TELEMETER_10,
    REG_STATUS_TELEMETER_11,
    REG_STATUS_TELEMETER_12,
    // Mesure de distance
    REG_DISTANCE_TELEMETER_1,
    REG_DISTANCE_TELEMETER_2,
    REG_DISTANCE_TELEMETER_3,
    REG_DISTANCE_TELEMETER_4,
    REG_DISTANCE_TELEMETER_5,
    REG_DISTANCE_TELEMETER_6,
    REG_DISTANCE_TELEMETER_7,
    REG_DISTANCE_TELEMETER_8,
    REG_DISTANCE_TELEMETER_9,
    REG_DISTANCE_TELEMETER_10,
    REG_DISTANCE_TELEMETER_11,
    REG_DISTANCE_TELEMETER_12,
    // Alerte distance seuile détectée
    REG_DISTANCE_ALERT_TELEMETERS_H,    // Alerte groupée : télémètres 1 à 8        
    REG_DISTANCE_ALERT_TELEMETERS_L,    // Alerte groupée : télémètres 9 à 12    

            // ============================================
    // Read write registers
    // ============================================
    // Pour les échanges optimisés/sécurisés (avec checksum) lors des requêtes de lecture
    REG_PTR_REG_LECTURE_I2C,            // Indique quel registre sera renvoyé en 1er lors de la prochaine requête de lecture
    REG_NBRE_REGISTRES_LECTURE_I2C,     // Indique le nombre de registres qui vont être demandés par le master lors de la prochaine requête de lecture
    //  AUTORISATION OU NON DE FONCTIONNER POUR CHAQUE TELEMETRE
    REG_ENABLE_TELEMETER_1,   // Télémètre par télémètre
    REG_ENABLE_TELEMETER_2,
    REG_ENABLE_TELEMETER_3,
    REG_ENABLE_TELEMETER_4,
    REG_ENABLE_TELEMETER_5,
    REG_ENABLE_TELEMETER_6,
    REG_ENABLE_TELEMETER_7,
    REG_ENABLE_TELEMETER_8,
    REG_ENABLE_TELEMETER_9,
    REG_ENABLE_TELEMETER_10,
    REG_ENABLE_TELEMETER_11,
    REG_ENABLE_TELEMETER_12,
    REG_ENABLE_TELEMETERS_H,    // Autorisation groupée : 8 bits : télémètre 1 à 8
    REG_ENABLE_TELEMETERS_L,    // Autorisation groupée : 4 bits : télémètres 9 à 12
    //  ADRESSE I2C DE CHAQUE TELEMETRE
    REG_I2CADDR_TELEMETER_1,
    REG_I2CADDR_TELEMETER_2,
    REG_I2CADDR_TELEMETER_3,
    REG_I2CADDR_TELEMETER_4,
    REG_I2CADDR_TELEMETER_5,
    REG_I2CADDR_TELEMETER_6,
    REG_I2CADDR_TELEMETER_7,
    REG_I2CADDR_TELEMETER_8,
    REG_I2CADDR_TELEMETER_9,
    REG_I2CADDR_TELEMETER_10,
    REG_I2CADDR_TELEMETER_11,
    REG_I2CADDR_TELEMETER_12,

    REG_LAUNCH_AUTOTEST,        
    REG_STOR_PGED,
    REG_STOR_PGEC,
            
    REG_I2C_8BITS_ADDRESS,
    REG_EEPROM_WRITE_UNPROTECT,
    REG_EEPROM_RESET_FACTORY,
    // _____________________________
    MAX_REGISTRES_NUMBER
    
}T_REG_ADDRESS; 

#endif	// _LIDAR_CONST_H_
