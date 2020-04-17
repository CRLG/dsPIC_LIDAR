#ifndef _LIDAR_H_
#define _LIDAR_H_

#include "vl53l0x_types.h"

typedef enum {
    TELEMETER_1 = 1,
    TELEMETER_2,
    TELEMETER_3,
    TELEMETER_4,
    TELEMETER_5,
    TELEMETER_6,
    TELEMETER_7,
    TELEMETER_8,
    TELEMETER_9,
    TELEMETER_10,
    TELEMETER_11,
    TELEMETER_12,
    ALL_TELEMETERS = 0xFF
}tTelemeterIndex;

void lidar_init();
unsigned char lidar_calibration(tTelemeterIndex index);
unsigned char lidar_settings(tTelemeterIndex index);
unsigned char lidar_autotest(tTelemeterIndex index);
unsigned char lidar_read_distance(tTelemeterIndex index);
unsigned char lidar_change_i2c_addr(tTelemeterIndex index, unsigned char new_addr);
void lidar_enable(tTelemeterIndex index, unsigned char enable);
unsigned char lidar_get_status(tTelemeterIndex index);
unsigned char lidar_ping(tTelemeterIndex index);
void lidar_periodic_call();

#endif	/* _LIDAR_H_ */

