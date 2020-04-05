
#ifndef _GENERAL_H_
#define _GENERAL_H_

#define VERSION_SOFT_MAJ    (1)
#define VERSION_SOFT_MIN    (0)

#define I2C_DEFAULT_ADDRESS_8bits (0x64)
void i2c1_init(unsigned char i2c_addr);

#define ENTER_CRITICAL_SECTION_I2C() { _SI2C1IE = 0; }
#define LEAVE_CRITICAL_SECTION_I2C() { _SI2C1IE = 1; }

// Pour le séquenceur de tâche
#define PERIODE_TICK    (10)
#define TEMPO_10msec    (10/PERIODE_TICK)
#define TEMPO_20msec    (20/PERIODE_TICK)
#define TEMPO_50msec    (50/PERIODE_TICK)
#define TEMPO_100msec   (100/PERIODE_TICK)
#define TEMPO_200msec   (200/PERIODE_TICK)
#define TEMPO_500msec   (500/PERIODE_TICK)
#define TEMPO_1sec      (1000/PERIODE_TICK)
#define TEMPO_2sec      (2000/PERIODE_TICK)
#define TEMPO_5sec      (5000/PERIODE_TICK)
#define TEMPO_10sec     (10000/PERIODE_TICK)
#define TEMPO_15sec     (15000/PERIODE_TICK)

extern unsigned short cptPerteComMaster;

#endif
// End


