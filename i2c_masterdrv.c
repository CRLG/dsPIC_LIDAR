#include <xc.h>
#include "General.h"
#include "i2c_masterdrv.h"

#define SLAVE_ACK_OK   0
#define SLAVE_ACK_NOK  1

/*********************************************************************
* Generates an I2C Start Condition
* Voir note d'application Microchip AN70000195
* I2CxBRG = ( (1/Fscl - Delay) * Fcy) - 2
* Fcsl = 400 kHz
* Delay = 120nsec 
* Fcy = 23.03125 MHz
* => I2CBRG =  
********************************************************************/
#define Fi2cslave       (400000)
#define I2C_SLAVE_BRG   ( (((1.f/Fi2cslave) - 120e-9) * F_FCY) - 2)

void i2c_master_init(void)
{	
	I2C2BRG = I2C_SLAVE_BRG;

	I2C2RCV = 0x0000;
	I2C2TRN = 0x0000;
	
    // Enable I2C
    I2C2CON = 0x9000;
}

/*********************************************************************
* Generates an I2C Start Condition
********************************************************************/
static void StartI2C(void)
{
	I2C2CONbits.SEN = 1;		//Generate Start COndition
	while(I2C2CONbits.SEN);     //Wait for Start COndition
}

/*********************************************************************
* Generates a restart condition
********************************************************************/
static void RestartI2C(void)
{
	I2C2CONbits.RSEN = 1;		//Generate Restart		
	while(I2C2CONbits.RSEN);	//Wait for restart	
}

/*********************************************************************
* Overview:		Generates a bus stop condition
********************************************************************/
static void StopI2C(void)
{
	I2C2CONbits.PEN = 1;		//Generate Stop Condition
	while(I2C2CONbits.PEN);     //Wait for Stop
}

/*********************************************************************
* Overview:		Writes a byte out to the bus
********************************************************************/
static void WriteI2C(unsigned char byte)
{
	//This function transmits the byte passed to the function
	I2C2TRN = byte;					//Load byte to I2C2 Transmit buffer
	while (I2C2STATbits.TBF);		//wait for data transmission
}

/*********************************************************************
* Overview:		Read a single byte from Bus
* Return :		Contents of I2C2 receive buffer.
********************************************************************/
static unsigned int getI2C(void)
{
	I2C2CONbits.RCEN = 1;			//Enable Master receive
	Nop();  // TODO : à vérifier si nécessaire ?
	while(!I2C2STATbits.RBF);		//Wait for receive buffer to be full
	return(I2C2RCV);				//Return data in buffer
}

/*********************************************************************
* Overview:		Waits for bus to become Idle
********************************************************************/
static void IdleI2C(void)
{
	while (I2C2STATbits.TRSTAT);		//Wait for bus Idle
}

/*********************************************************************
* Overview:		Return the Acknowledge status on the bus
* Return : 
*   SLAVE_ACK_OK    = slave ack last transmit byte 
*   SLAVE_ACK_NOK   = slave did not ack last transmit byte
********************************************************************/
static unsigned int ACKStatus(void)
{
	return (I2C2STATbits.ACKSTAT);
}

/*********************************************************************
* Overview:		Generates a NO Acknowledge on the Bus
********************************************************************/
static void NotAckI2C(void)
{
	I2C2CONbits.ACKDT = 1;			//Set for NotACk
	I2C2CONbits.ACKEN = 1;
	while(I2C2CONbits.ACKEN);		//wait for ACK to complete
	I2C2CONbits.ACKDT = 0;			//Set for ACk
}

/*********************************************************************
* Overview:		Generates an Acknowledge.
********************************************************************/
static void AckI2C(void)
{
	I2C2CONbits.ACKDT = 0;			//Set for ACk
	I2C2CONbits.ACKEN = 1;
	while(I2C2CONbits.ACKEN);		//wait for ACK to complete
}

/*********************************************************************
* Overview:		Complete write sequence
* Input : 
*   address : 7 bits slave destination address
*   data : buffer of data bytes
*   len : number of bytes to write from the buffer to I2C
* Return : 
*    0 : OK
*   -1 : an arror occurs during transmission
********************************************************************/
tI2CMasterError i2c_master_write(char address, unsigned char *data, unsigned char len)
{
    tI2CMasterError error_code = I2CM_OK;

    IdleI2C();                  //Ensure Module is Idle
	StartI2C();                 //Generate Start COndition
	WriteI2C(address);          //Write Control byte
	IdleI2C();
    
    if (ACKStatus() == SLAVE_ACK_NOK) {
        error_code = I2CM_SLAVE_NACK_TRANSMISSION;
        goto i2c_master_write_end;
    }
    
    while (len--) {
        WriteI2C(*data++);
        IdleI2C();
        if (ACKStatus() == SLAVE_ACK_NOK) {
            error_code = I2CM_SLAVE_NACK_TRANSMISSION;
            goto i2c_master_write_end;
        }
    }
i2c_master_write_end :
    IdleI2C();
	StopI2C();
    return error_code;
}

/*********************************************************************
* Overview:		Complete write sequence
* Input : 
*   address : 7 bits slave destination address
*   reg : register address 
*   data : buffer of data bytes
*   len : number of bytes to write from the buffer to I2C
* Return : 
*    0 : OK
*   -1 : an arror occurs during transmission
********************************************************************/
tI2CMasterError i2c_master_write_register(char address, unsigned char reg, unsigned char *data, unsigned char len)
{
    tI2CMasterError error_code = I2CM_OK;

    IdleI2C();                  //Ensure Module is Idle
	StartI2C();                 //Generate Start COndition
	WriteI2C(address);          //Write Control byte
	IdleI2C();
    if (ACKStatus() == SLAVE_ACK_NOK) {
        error_code = I2CM_SLAVE_NACK_TRANSMISSION;
        goto i2c_master_write_end;
    }

	WriteI2C(reg);          //Write reg address
	IdleI2C();
    if (ACKStatus() == SLAVE_ACK_NOK) {
        error_code = I2CM_SLAVE_NACK_TRANSMISSION;
        goto i2c_master_write_end;
    }
    
    while (len--) {
        WriteI2C(*data++);
        IdleI2C();
        if (ACKStatus() == SLAVE_ACK_NOK) {
            error_code = I2CM_SLAVE_NACK_TRANSMISSION;
            goto i2c_master_write_end;
        }
    }
i2c_master_write_end :
    IdleI2C();
	StopI2C();
    return error_code;
}

/*********************************************************************
* Overview:		Complete read sequence
* Input : 
*   address : 7 bits slave destination address
*   len : number of bytes to read from slave
* Input/Output : 
*   data : buffer of allocated data bytes
* Return : 
*    0 : OK
*   -1 : an arror occurs during transmission
********************************************************************/
tI2CMasterError i2c_master_read(char address, unsigned char *data, unsigned char len)
{
    tI2CMasterError error_code = I2CM_OK;
     
	IdleI2C();                      //wait for bus Idle
	StartI2C();                     //Generate Start Condition
    WriteI2C(address | 0x01);       //Write control byte for read
	IdleI2C();                      //wait for bus Idle
	
    if (ACKStatus() == SLAVE_ACK_NOK) {
        error_code = I2CM_SLAVE_NACK_TRANSMISSION;
        goto i2c_master_read_end;
    }   

    while (len--) {
        *data++ = getI2C();
        if (len)    AckI2C();       // Send Ack
        else        NotAckI2C();    // Send Not Ack for last byte
    }

i2c_master_read_end : 
	StopI2C();                      //Generate Stop 
    return error_code;
}

/*********************************************************************
* Overview:		read data from register index
* Input : 
*   address : 7 bits slave destination address
*   reg : register address 
*   len : number of bytes to read 
* Input/Output : 
*   data : buffer of allocated data bytes
* Return : 
*    0 : OK
*   -1 : an arror occurs during transmission
********************************************************************/
tI2CMasterError i2c_master_read_register(char address, unsigned char reg, unsigned char *data, unsigned char len)
{
    tI2CMasterError error_code = I2CM_OK;
     
    IdleI2C();                  //Ensure Module is Idle
	StartI2C();                 //Generate Start COndition
	WriteI2C(address);          //Write Control byte
	IdleI2C();
    if (ACKStatus() == SLAVE_ACK_NOK) {
        error_code = I2CM_SLAVE_NACK_TRANSMISSION;
        goto i2c_master_read_end;
    }

  	WriteI2C(reg);              //Write reg address
	IdleI2C();
    if (ACKStatus() == SLAVE_ACK_NOK) {
        error_code = I2CM_SLAVE_NACK_TRANSMISSION;
        goto i2c_master_read_end;
    }
      
	RestartI2C();                   //Generate Start Condition
    WriteI2C(address | 0x01);       //Write control byte for read
	IdleI2C();                      //wait for bus Idle
    if (ACKStatus() == SLAVE_ACK_NOK) {
        error_code = I2CM_SLAVE_NACK_TRANSMISSION;
        goto i2c_master_read_end;
    }   
    while (len--) {
        *data++ = getI2C();
        if (len)    AckI2C();       // Send Ack
        else        NotAckI2C();    // Send Not Ack for last byte
    }

i2c_master_read_end : 
	StopI2C();                      //Generate Stop 
    return error_code;
}

/*********************************************************************
* Overview:		Ping a slave
* Input : 
*  address : 7 bits slave address
* Return : 
*   1 if slave is present
*   0 if slave is not present 
********************************************************************/
int i2c_master_ping(char address)
{
    int present = 0;
    IdleI2C();                  //Ensure Module is Idle
    StartI2C();                 //Generate Start COndition
    WriteI2C(address);          //Write Control byte
    IdleI2C();
    if (ACKStatus() == SLAVE_ACK_OK) present = 1;
    IdleI2C();
    StopI2C();
    return present;
}
