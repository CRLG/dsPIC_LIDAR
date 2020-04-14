#include <xc.h>
#include "lidar_const.h"
#include "lidar_registers.h"
#include "lidar.h"
#include "i2c_masterdrv.h"
#include "General.h"

// ___________________________________________________________
// Initialise le LIDAR
void lidar_init()
{
    // TODO
}

// ___________________________________________________________
unsigned char lidar_autotest(tTelemeterIndex index)
{
    // TODO  
    unsigned char result = 1;
    return result;
}

// ___________________________________________________________
unsigned char lidar_read_distance(tTelemeterIndex index)
{
    // TODO : appeler la bonne fonction dans le driver STMicro
    return 0xFF;
}

// ___________________________________________________________
void lidar_change_i2c_addr(tTelemeterIndex index, unsigned char new_addr)
{
    // TODO    
}

// ___________________________________________________________
void lidar_enable(tTelemeterIndex index, unsigned char enable)
{
    // TODO  : certainement à revoir en fonction de la facilité du routage
    switch(index) {
        case TELEMETER_1 : 
            LATAbits.LATA0 = enable;
            break;
        case TELEMETER_2 : 
            LATAbits.LATA1 = enable;
            break;
        case TELEMETER_3 : 
            LATBbits.LATB2 = enable;
            break;
        case TELEMETER_4 : 
            LATBbits.LATB3 = enable;
            break;
        case TELEMETER_5 : 
            LATAbits.LATA3 = enable;
            break;
        case TELEMETER_6 : 
            LATBbits.LATB4 = enable;
            break;
        case TELEMETER_7 : 
            LATBbits.LATB7 = enable;
            break;
        case TELEMETER_8 : 
            LATBbits.LATB12 = enable;
            break;
        case TELEMETER_9 : 
            LATBbits.LATB13 = enable;
            break;
        case TELEMETER_10 : 
            LATBbits.LATB14 = enable;
            break;
        case TELEMETER_11 : 
            LATBbits.LATB15 = enable;
            break;
        case TELEMETER_12 : 
            LATAbits.LATA4 = enable;
            break;
        case ALL_TELEMETERS :
            LATAbits.LATA0 = enable;
            LATAbits.LATA1 = enable;
            LATBbits.LATB2 = enable;
            LATBbits.LATB3 = enable;
            LATAbits.LATA3 = enable;
            LATBbits.LATB4 = enable;
            LATBbits.LATB7 = enable;
            LATBbits.LATB12 = enable;
            LATBbits.LATB13 = enable;
            LATBbits.LATB14 = enable;
            LATBbits.LATB15 = enable;
            LATAbits.LATA4 = enable;
            break;
        default : 
            // ne rien faire
            break;
    }
}

// ___________________________________________________________
unsigned char lidar_get_status(tTelemeterIndex index)
{
    // TODO    
    unsigned char result = 0;
    return result;
}

// ___________________________________________________________
unsigned char lidar_ping(tTelemeterIndex index)
{
    unsigned char index_reg = (index - TELEMETER_1) + REG_I2CADDR_TELEMETER_1;
    return i2c_master_ping(dsPIC_reg[index_reg].val);
}

// ___________________________________________________________
void lidar_periodic_call()
{
    // TODO
    //   Lecture des distances brutes de chaque télémètre actif
    //   Filtrage des données brutes
    //   Seuil et conclusion sur les alertes
}
