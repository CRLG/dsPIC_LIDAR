/* 
 * File:   lidar_registers.h
 * Author: crlg
 *
 * Created on 29 mars 2020, 15:19
 */

#ifndef LIDAR_REGISTERS_H
#define	LIDAR_REGISTERS_H

#include "lidar_const.h"

typedef  enum {
    READ_WRITE = 0,
    READ_ONLY
}T_TYPE_REGISTRE;    


typedef struct {
    unsigned char   val;
    unsigned char   new_data;
    T_TYPE_REGISTRE type_read_write;
}T_dsPIC_REGISTER;    


extern T_dsPIC_REGISTER dsPIC_reg[];

void lidar_registers_management(void);
void lidar_registers_init(void);

#endif	/* LIDAR_REGISTERS_H */

