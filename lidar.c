#include <xc.h>
#include "lidar_const.h"
#include "lidar_registers.h"
#include "lidar.h"
#include "i2c_masterdrv.h"
#include "General.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_def.h"
#include "vl53l0x_api.h"

static VL53L0X_Dev_t m_sensor_handle[NUMBER_OF_TELEMETERS];

// ___________________________________________________________
// Initialise le LIDAR
void lidar_init()
{
    uint8_t i=0;
       
    for(i=0;i<NUMBER_OF_TELEMETERS;i++)
    {
        //adresse et mode par défaut
        m_sensor_handle[i].I2cDevAddr      = 0x52;
        m_sensor_handle[i].comms_type      =  1;
        m_sensor_handle[i].comms_speed_khz =  400;
    }
    
    lidar_change_i2c_addr(TELEMETER_1,0x54);
    lidar_calibration(TELEMETER_1);
    lidar_settings(TELEMETER_1);
}

// ___________________________________________________________
unsigned char lidar_calibration(tTelemeterIndex index)
{
    unsigned char errorCalibration=1;
    if(index>=NUMBER_OF_TELEMETERS)
        return errorCalibration;
    
    int8_t int_status=VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t * m_telemeter_handle=m_sensor_handle+index-1;
    
    VL53L0X_DeviceInfo_t m_capteur_01_info; //structure pour stocker les infos du capteur
    uint32_t refComptageSPAD; //nombre de photons comptés
    uint8_t isOuvertureSPAD; //type d'ouverture du SPAD
    uint8_t VhvCal; //Tension de diode inverse minimale requise pour produire une impulsion de sortie
    uint8_t PhaseCal; //phase des photons

    //__________________________________________________
    //	init et calibration capteur
    //	(environ 90ms)
    //__________________________________________________

    //Initialisation des structures de données capteur
    int_status=VL53L0X_DataInit(m_telemeter_handle);

    if(int_status == VL53L0X_ERROR_NONE)
    {
      //optionnel: on vérifie les infos capteurs
      int_status = VL53L0X_GetDeviceInfo(m_telemeter_handle, &m_capteur_01_info);
      /*if(int_status == VL53L0X_ERROR_NONE)
      {
          printf("Capteur Nom : %s\n", m_capteur_01_info.Name);
          printf("Capteur Type : %s\n", m_capteur_01_info.Type);
          printf("Capteur ID : %s\n", m_capteur_01_info.ProductId);
          printf("Indice Revision Majeur : %d\n", m_capteur_01_info.ProductRevisionMajor);
          printf("Indice Revision Mineur : : %d\n", m_capteur_01_info.ProductRevisionMinor);
      }*/
    }

    //environ 40ms en comptant le datainit précédent
    //Permet à l'utilisateur de modifier les paramètres capteur
    if(int_status == VL53L0X_ERROR_NONE)
      int_status = VL53L0X_StaticInit(m_telemeter_handle);

    //environ 10ms
    //Paramétrage de la diode d'émission (SPAD) à conserver et d'autant plus si on veut mettre plus tard une protection en plexi
    if(int_status == VL53L0X_ERROR_NONE)
      int_status = VL53L0X_PerformRefSpadManagement(m_telemeter_handle, &refComptageSPAD, &isOuvertureSPAD);

    //environ 40ms
    //Paramétrage d'une mesure simple - nécessite absolument la calibration du SPAD - sensible aux changements de température, à refaire si changement de plus de 8°C (ne nous concerne pas a priori)
    if(int_status == VL53L0X_ERROR_NONE)
      int_status = VL53L0X_PerformRefCalibration(m_telemeter_handle, &VhvCal, &PhaseCal);

    if(int_status == VL53L0X_ERROR_NONE)
      errorCalibration=0;

    return errorCalibration;
}

// ___________________________________________________________
unsigned char lidar_settings(tTelemeterIndex index)
{
    unsigned char errorSetting=1;
    if(index>=NUMBER_OF_TELEMETERS)
        return errorSetting;
    
    int8_t int_status=VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t * m_telemeter_handle=m_sensor_handle+index-1;
    
    //Inutile normalement si on veut une mesure simple (paramétrage par défaut)
    if(int_status == VL53L0X_ERROR_NONE)
        //mesure simple
        int_status = VL53L0X_SetDeviceMode(m_telemeter_handle, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    //Activation/désactivation des vérifications du sigma et du signal (tiré d'un exemple d'utilisation - à potarder plus tard)
    if (int_status == VL53L0X_ERROR_NONE)
        int_status = VL53L0X_SetLimitCheckEnable(m_telemeter_handle, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (int_status == VL53L0X_ERROR_NONE)
        int_status = VL53L0X_SetLimitCheckEnable(m_telemeter_handle, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (int_status == VL53L0X_ERROR_NONE)
        int_status = VL53L0X_SetLimitCheckEnable(m_telemeter_handle, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    if (int_status == VL53L0X_ERROR_NONE)
        int_status = VL53L0X_SetLimitCheckValue(m_telemeter_handle, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));

    if(int_status == VL53L0X_ERROR_NONE)
        errorSetting=0;

    return errorSetting;
}

// ___________________________________________________________
unsigned char lidar_autotest(tTelemeterIndex index)
{
    // TODO  
    unsigned char result = 1;
    return result;
}

// ___________________________________________________________
uint16_t lidar_read_distance(tTelemeterIndex index)
{
    uint16_t rangeValue=0;
    if(index>=NUMBER_OF_TELEMETERS)
        return rangeValue;
    
    rangeValue=0xFFFF;
    
    int8_t int_status=VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t * m_telemeter_handle=m_sensor_handle+index-1;
    
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;

    //mesure de distance
    int_status = VL53L0X_PerformSingleRangingMeasurement(m_telemeter_handle, &RangingMeasurementData);

    //si la mesure s'est mal passée, il faut renvoyer valeur infinie
    if (int_status != VL53L0X_ERROR_NONE)
        return rangeValue;

    //la donnée de mesure est valide, si elle n'est pas valide on garde la valeur infinie
    if(RangingMeasurementData.RangeStatus==0)
    {
        rangeValue=RangingMeasurementData.RangeMilliMeter;
        //printf("Distance: %d\n", RangingMeasurementData.RangeMilliMeter);
    }

    return rangeValue;
}

// ___________________________________________________________
unsigned char lidar_change_i2c_addr(tTelemeterIndex index, unsigned char new_addr)
{
    unsigned char int_status=1;
    if(index>=NUMBER_OF_TELEMETERS)
        return int_status;
    
    VL53L0X_Dev_t * m_telemeter_handle=m_sensor_handle+index-1;
    
    if(VL53L0X_SetDeviceAddress(m_telemeter_handle,new_addr)==VL53L0X_ERROR_NONE)
    {
        m_telemeter_handle->I2cDevAddr=new_addr;
        int_status=I2CM_OK;
    }
    
    return int_status;
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
