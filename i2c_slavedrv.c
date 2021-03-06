#include <xc.h>
#include <stdint.h>
#include "i2c_slavedrv.h"
#include "lidar_registers.h"

/*****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************/
#define SIZE_RAM_BUFFER 200
static uint8_t ucRAMBuffer[SIZE_RAM_BUFFER]; //RAM area which will work as EEPROM for Master I2C device
unsigned short cptPerteComMaster=0;

// Prototype des fonctions locales
static void PrepareBufferToSend(void);
static void GestionReceptionI2C(unsigned char first, unsigned char data);
static void FinReceptionTrameValideI2C(void);


/******************************************************************************
 * Overview:        Initializes I2C1 peripheral.
 * i2c_addr : 7 bit address
 *****************************************************************************/
void i2c_slave_init(unsigned char i2c_addr)
{
	#if !defined(USE_I2C_Clock_Stretch)
		I2C1CON = 0x8000;	//Enable I2C1 module
	#else
		I2C1CON = 0x9040;	//Enable I2C1 module, enable clock stretching
	#endif
	

	I2C1ADD = i2c_addr>>1; // 7-bit I2C slave address must be initialised in register.
	
	IFS1=0;
	_SI2C1IE = 1;
}



/******************************************************************************
 * Overview:        This is the ISR for I2C1 Slave interrupt.
 *****************************************************************************/
void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void)
{
	unsigned char Temp;	//used for dummy read
    static unsigned int indexRAM=0;
    // Requ�te d'�criture: juste apr�s l'adresse I2C reconnue
	if((I2C1STATbits.R_W == 0)&&(I2C1STATbits.D_A == 0))	//Address matched
	{
    	GestionReceptionI2C(1, I2C1RCV);  // Initialise la machine d'�tat + dummy read (rien � lire, il faut attendre la r�ception des octets suivants)
	}
	else if((I2C1STATbits.R_W == 0)&&(I2C1STATbits.D_A == 1))	//check for data	
	{
		GestionReceptionI2C(0, I2C1RCV);
		#if defined(USE_I2C_Clock_Stretch)
			I2C1CONbits.SCLREL = 1;	//Release SCL1 line
		#endif
	}
	// Requ�te de lecture : juste apr�s l'adresse I2C reconnue	
	else if((I2C1STATbits.R_W == 1)&&(I2C1STATbits.D_A == 0))
	{
		Temp = I2C1RCV;
		PrepareBufferToSend();  // Pr�pare un buffer � envoyer
        indexRAM = 0;
		I2C1TRN = ucRAMBuffer[indexRAM++];  // Renvoie le 1er octet du buffer
        I2C1CONbits.SCLREL = 1;	//Release SCL1 line
		while(I2C1STATbits.TBF);//Wait till all
        cptPerteComMaster = 0; // requ�te lecture -> le master i2c est bien pr�sent 
	}
	// Requ�te de lecture: la suite des octets
	else
	{
		I2C1TRN = ucRAMBuffer[indexRAM++];
        I2C1CONbits.SCLREL = 1;	//Release SCL1 line
		while(I2C1STATbits.TBF);//Wait till all
	}
	_SI2C1IF = 0;	//clear I2C1 Slave interrupt flag
}	

// ________________________________________________________
// Fonction appel�e sur une requ�te de lecture
// Juste apr�s que l'adresse I2C ai �t� reconnue
// Pr�pare le buffer � renvoyer au maitre
// ucRAMBuffer est le buffer
// Certaines valeurs comme les entr�s analogiques sont d�j� renseign�e dans dsPIC_reg[] par la tache de fond
//  
static void PrepareBufferToSend(void)
{
 unsigned short i=0;  
 unsigned char checksum=0; 
 unsigned short indexBuffer = 0;
 unsigned char indexRegistre;
 
 indexRegistre = dsPIC_reg[REG_PTR_REG_LECTURE_I2C].val;
 for (i=0; i<dsPIC_reg[REG_NBRE_REGISTRES_LECTURE_I2C].val; i++) {    
    // Ajouter ici les autres cas particulier s'il y en a
    
    ucRAMBuffer[indexBuffer] = dsPIC_reg[indexRegistre].val;
    checksum += ucRAMBuffer[indexBuffer];
    indexBuffer++;
    indexRegistre++;
 }
 ucRAMBuffer[indexBuffer] = checksum; // Le dernier octet � transf�rer est le checksum 
 LATAbits.LATA4 = ~LATAbits.LATA4;
}  

// ____________________________________________________________  
typedef enum {
    RX_NOMBRE_OCTETS_TRANSFERES = 0,
    RX_ADRESSE_REGSTRE,
    RX_VALEURS_REGISTRES,
    RX_CHECKSUM,
    RX_ERREUR   
}T_ETAT_RX_I2C;   

// _______________________________________________________________
static void GestionReceptionI2C(unsigned char first, unsigned char data)
{
  static unsigned char etat = RX_NOMBRE_OCTETS_TRANSFERES;
  static unsigned char checksum  = 0;
  static unsigned char nbre_octets_recus = 0;
  static unsigned char nbre_octets_a_recevoir = 0;  
  // RAZ de la machine d'�tat en d�but de communication
  if (first) { 
     etat = RX_NOMBRE_OCTETS_TRANSFERES; 
     checksum = 0;
     nbre_octets_recus = 0;
     return;
  }
  
  switch(etat) {
    // _________________________________
    case RX_NOMBRE_OCTETS_TRANSFERES : 
      ucRAMBuffer[0] = data;
      nbre_octets_a_recevoir = data;
      nbre_octets_recus = 0;
      etat = RX_ADRESSE_REGSTRE; 
      checksum = data;
      // Cas anormal, on ne peut pas recevoir plus de donn�es que la taille du buffer de r�ception.
      if (nbre_octets_a_recevoir > SIZE_RAM_BUFFER) { etat = RX_ERREUR; }
      // Cas anormal, il faut  recevoir au moins 3 octets (l'adresse du registre + valeur + checksum)
      if (nbre_octets_a_recevoir < 3) { etat = RX_ERREUR; }
    break;   
    // _________________________________
    case RX_ADRESSE_REGSTRE : 
      ucRAMBuffer[nbre_octets_recus + 1] = data;
      nbre_octets_recus++;
      etat = RX_VALEURS_REGISTRES; 
      checksum += data;
    break;   
   // _________________________________
    case RX_VALEURS_REGISTRES : // TODO : y'a un truc � revoir sur la position du nbre_octets_recus++; pour optimiser les op�rations r�alis�es
      ucRAMBuffer[nbre_octets_recus + 1] = data;
      nbre_octets_recus++;
      checksum += data;  
      if (nbre_octets_recus >= (nbre_octets_a_recevoir-1)) {
         etat = RX_CHECKSUM; 
      }           
    break;   
    // _________________________________
    case RX_CHECKSUM : 
      if (data == checksum) {
        FinReceptionTrameValideI2C();
        cptPerteComMaster = 0; // requ�te d'�criture valide -> le master i2c est bien pr�sent 
      } 
      else {
        etat = RX_ERREUR;
      }   
    break;  
    // _________________________________
    case RX_ERREUR:
    default :
        // Ne rien faire, attendre le d�but de la prochaine communication 
    break;  
      
  }      
}    
  
// ____________________________________________________________ 
// Si on arrive ici : 
//  - ucRAMBuffer[0] contient le nombre d'octets qui suit le transfert
//  - ucRAMBuffer[1] contient l'adresse du premier registre reg affect� par l'�criture
//  - ucRAMBuffer[2] contient la valeur du registre reg
//  - ucRAMBuffer[3] contient la valeur du registre reg+1
static void FinReceptionTrameValideI2C(void)
{
 unsigned short i=0;
 unsigned char indexReg;
 
 for (i=0; i<(ucRAMBuffer[0]-2); i++) { // -2 car il faut supprimer le checksum et le num�ro de registre
    indexReg = ucRAMBuffer[1] + i;
    if (dsPIC_reg[indexReg].type_read_write == READ_WRITE) {
       dsPIC_reg[indexReg].val = ucRAMBuffer[i+2];
       dsPIC_reg[indexReg].new_data = 1; // indique � la t�che de fond que le registe a �t� modifi�     
    }
    // else : le registre est read only, pas d'�criture     
 }
}   


