#ifndef _I2C_MASTER_DRIVER_H_
#define _I2C_MASTER_DRIVER_H_


typedef enum {
    I2CM_OK = 0,
    I2CM_SLAVE_NACK_TRANSMISSION,
    I2CM_TIMEOUT,
    I2CM_BUS_COLLISION
}tI2CMasterError;

void i2c_master_init(void);
tI2CMasterError i2c_master_write(char address, unsigned char *data, unsigned char len);
tI2CMasterError i2c_master_write_register(char address, unsigned char reg, unsigned char *data, unsigned char len);
tI2CMasterError i2c_master_read(char address, unsigned char *data, unsigned char len);
tI2CMasterError i2c_master_read_register(char address, unsigned char reg, unsigned char *data, unsigned char len);
int i2c_master_ping(char address);

#endif  // _I2C_MASTER_DRIVER_H_
