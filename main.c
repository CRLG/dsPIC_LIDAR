// DSPIC33EP512GP502 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS      = PGD3      // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN   = OFF       // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1  = ON        // Alternate I2C1 pins (I2C1 mapped to ASDA1/ASCL1 pins)    
#pragma config ALTI2C2  = ON        // Alternate I2C2 pins (I2C2 mapped to ASDA2/ASCL2 pins)

// FWDT
#pragma config WDTWIN   = WIN25     // Watchdog Window Select bits (WDT Window is 25% of WDT period)
#pragma config WDTPOST  = PS256     // Watchdog Timer Postscaler bits (1:256)
#pragma config WDTPRE   = PR128     // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN   = ON        // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS   = OFF       // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN   = OFF       // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD   = NONE      // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON       // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY  = ON        // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM    = CSDCMD    // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC    = FRCPLL    // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL))
#pragma config IESO     = OFF       // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP     = OFF       // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP      = OFF       // General Segment Code-Protect bit (General Segment Code protect is Disabled)
// #pragma config statements should precede project file includes.
// =================================================================
// Horloge sur le dsPIC : voir note d'application Microchip AN70227E
//      - FRC = 7.37 MHz        -> Fréquence générée en interne de base 
// Avec activation de la PLL, on a :
//      - Fosc = FRC * (M / (N1 * N2))
//          - M = 330 (voir PLLDIV)
//          - N1 = 2 (voir PLLPRE)
//          - N2 = 19 (void PLLPOST)
//        => Fosc = 7.37 * (330/(2*19)) = 64.00263158 MHz
//      - Fcy = (Fosc / 2) = 32.001316 MHz (Tcy = 31.24nsec)    -> Fréquence pour les instructions CPU
//      - Fp = Fcy                                              -> Fréquence de base pour les prériphériques (Timer, I2C, UART, ...)
// =================================================================
// Watchdog : voir note d'application Microchip DS70615C
// Treset_watchdog = 1/Frc * N1 *N2   => Période du watchdog avant reset
//      Avec Frc = Fréquence l'oscillateur interne = 32kHz
//      N1 = 32 (WDTPRE=config 5 bits) ou 128 (WDTPRE = config 7 bits))
//      N2 = WDTPOST 
//      On choisit un période watchdog à 1sec environ
//      => WDTPRE = 128 et WDTPOST = 256
//      => Treset_watchdog = (1/32e3) * 128 * 256 = 1.024sec
// =================================================================

#include "General.h"    // A mettre en 1er car contient FCY
#include <xc.h>
#include <libpic30.h>
#include <stdio.h>
#include "eeprom.h"
#include "lidar_registers.h"
#include "lidar.h"
#include "i2c_slavedrv.h"
#include "i2c_masterdrv.h"
#include "mcp23017.h"
#include "uartdrv.h"
#include "uart2drv.h"
#include "xbeedriver.h"
#include "leds_ws2812b.h"


// Constates

// Variables globales
volatile unsigned char tick_timer=0;

// Variables globales externes
extern unsigned short cptPerteComMaster;

// Prototypes des fonctions locales
void Sequenceur(void);
void Init_PLL_Oscillator();
void Init_Timer1(void);
void Init_Ports(void);
void Init_Watchdog();
void Init_IRQ_Priority();
void Refresh_Watchdog();
void led_management();
void distance_alarm_management();


// ____________________________________
//	main entry point
int main (void)
{
    // Hardware init
    Init_PLL_Oscillator();
    Init_EEPROM(); 
    Init_Ports();
    Init_IRQ_Priority();
    lidar_registers_init();
    i2c_slave_init(dsPIC_reg[REG_I2C_8BITS_ADDRESS].val);  // restitution de la valeur configurée en EEPROM
    i2c_master_init();
    mcp23017_init(I2C_MCP23017_I2C_ADDRESS);
    mcp23017_configDirections(0, 0);
    lidar_init();
    uart_init();
    uart2_init();
    ws2812b_init();
    Init_Timer1();
    Init_Watchdog();
 
    printf("\n\r Starting #...\n\r");
    
	while(1)
	{
        //lidar_registers_management();
        if (tick_timer) {
			tick_timer = 0;
			Sequenceur();
		}	
	}
}

//---------------------------------------------------------------------
void Sequenceur(void)
{
  static unsigned int cpt10msec = 0;
  static unsigned int cpt20msec = 0;
  static unsigned int cpt50msec = 0;
  static unsigned int cpt100msec = 0;
  static unsigned int cpt200msec = 0;
  static unsigned int cpt500msec = 0;
  static unsigned int cpt1sec = 0;
  static unsigned int cpt2sec = 0;
  static unsigned int cpt5sec = 0;
  static unsigned char start_led=0;
 
  // ______________________________
  cpt10msec++;
  if (cpt10msec >= TEMPO_10msec) {
  	cpt10msec = 0;

  }

  // ______________________________
  cpt20msec++;
  if (cpt20msec >= TEMPO_20msec) {
    cpt20msec = 0;

    Refresh_Watchdog();
   }


  // ______________________________
  cpt50msec++;
  if (cpt50msec >= TEMPO_50msec) {
    cpt50msec = 0;
    int distance;
 
    //lidar_periodic_call();
 
    dsPIC_reg[REG_DISTANCE_TELEMETER_1].val = lidar_read_distance(TELEMETER_1);

    LATAbits.LATA4 = ~LATAbits.LATA4;    

    int i;
    for (i=0; i<NUMBER_MAX_OF_TELEMETERS; i++) {
        dsPIC_reg[REG_FORCE_LED1+i].val = LED_FORCE_OFF;
    }
    //
    dsPIC_reg[REG_FORCE_LED1].val   = LED_CONTROLED_BY_LIDAR;
    EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_1] = 30;

        
    dsPIC_reg[REG_FORCE_LED1+3].val = LED_CONTROLED_BY_LIDAR;
    //dsPIC_reg[REG_FORCE_LED1+4].val = LED_FORCE_GREEN;
    //dsPIC_reg[REG_FORCE_LED1+5].val = LED_FORCE_BLUE;
    dsPIC_reg[REG_DISTANCE_TELEMETER_1+3].val = 30;
    EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_1+3] = 40;
     
    distance_alarm_management();
    led_management();
    ws2812b_periodicTask();
  }

  // ______________________________
  cpt100msec++;
  if (cpt100msec >= TEMPO_100msec) {
    cpt100msec = 0;

    //printf("Hello %d\n\r", data[0]++);
  }

  // ______________________________
  cpt200msec++;
  if (cpt200msec >= TEMPO_200msec) {
    cpt200msec = 0;

  }
  // ______________________________
  cpt500msec++;
  if (cpt500msec >= TEMPO_500msec) {
    cpt500msec = 0;

  }

  // ______________________________
  cpt1sec++;
  if (cpt1sec >= TEMPO_1sec) {
    cpt1sec = 0;

  }
  // ______________________________
  cpt2sec++;
  if (cpt2sec >= TEMPO_2sec) {
    cpt2sec = 0;

  }
  // ______________________________
  cpt5sec++;
  if (cpt5sec >= TEMPO_5sec) {
    cpt5sec = 0;

  }
}

/*---------------------------------------------------------------------
 Réglage PLL pour obtenir FOSC=64MHz => FCY = 32MHz
 Voir explication au début du fichier
-----------------------------------------------------------------------*/
void Init_PLL_Oscillator()
{
    CLKDIVbits.PLLPRE = 17;     // N1 = 19;
    CLKDIVbits.PLLPOST = 0;     // N2 = 2
    PLLFBDbits.PLLDIV = 328;    // M = 330
}


/*---------------------------------------------------------------------
  Function Name: Init_Timer1
  Description:   Initialize Timer1
  Inputs:        None
  Returns:       None
 F_FP = 32 MHz
 Tp = 1/F_FP
 On veut 1 IRQ toutes les 10msec
 Tirq = Tp* Prescaler * (PR1+1)
 => PR1 = Tirq/(Tp*Prescaler) - 1
 Avec Prescaler=64 => PR1 = 4999
-----------------------------------------------------------------------*/
void Init_Timer1(void)
{
	T1CON = 0;                  /* ensure Timer 1 is in reset state */
 	IFS0bits.T1IF   = 0;        /* reset Timer 1 interrupt flag */
	IPC0bits.T1IP   = 4;        /* set Timer1 interrupt priority level to 4 */
 	IEC0bits.T1IE   = 1;        /* enable Timer 1 interrupt */
	PR1             = 4999;     /* set Timer 1 period register */
	T1CONbits.TCKPS = 2;        /* Prescaler = 64 */
	T1CONbits.TCS   = 0;        /* select internal timer clock */
	T1CONbits.TON   = 1;        /* enable Timer 1 and start the count */ 
}


/*---------------------------------------------------------------------
  Function Name: _T1Interrupt
  Description:   Timer1 Interrupt Handler
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void __attribute__((interrupt, auto_psv)) _T1Interrupt( void )
{
	tick_timer = 1;				/* flag */
 	IFS0bits.T1IF = 0;			/* reset timer interrupt flag	*/
}	
	

// ___________________________________________________________
void Init_Ports(void)
{
    /* 	Initialize ports */
    LATA  = 0x0000;             // set latch levels
    TRISA = 0xFFFF;             // set IO as inputs

    TRISAbits.TRISA3 = 0;       // Sortie
    TRISAbits.TRISA4 = 0;       // Sortie : LED d'activité du SW
}

// ___________________________________________________________
void Init_IRQ_Priority()
{
    IPC0bits.T1IP       = 1;    // IRQ Timer1 (le plus prioritaire)
    IPC4bits.SI2C1IP    = 2;    // IRQ I2C esclave
    IPC2bits.U1RXIP     = 3;    // IRQ sur réception UART
}


// ___________________________________________________________
// Conclu pour chaque télémètre à un obstacle détecté
// 
void distance_alarm_management()
{
    int i;
    unsigned short _16bits_alarm = 0;
    for (i=0; i<NUMBER_MAX_OF_TELEMETERS; i++) 
    {
        if (dsPIC_reg[REG_DISTANCE_TELEMETER_1+i].val < EEPROM_values[EEPADDR_THRESHOLD_ALERT_TELEMETER_1+i]) {
            _16bits_alarm |= (1<<i);
        }
    }
    dsPIC_reg[REG_DISTANCE_ALARM_TELEMETERS_H].val =  _16bits_alarm>>8;
    dsPIC_reg[REG_DISTANCE_ALARM_TELEMETERS_L].val = _16bits_alarm&0xFF;
}

// ___________________________________________________________
// Défini l'état des LED
// 2 modes de fonctionnement pour les LED : 
//      - Pilotées par la fonction LIDAR : LED n°<i> allumée en fonction de la distance du télémètre n°<i>
//      - Forcé à une certaine couleur
void led_management()
{
    int i;
    int obstacles_detected = (dsPIC_reg[REG_DISTANCE_ALARM_TELEMETERS_H].val <<8) | dsPIC_reg[REG_DISTANCE_ALARM_TELEMETERS_L].val;
    for (i=0; i<NUMBER_MAX_OF_TELEMETERS; i++) 
    {
        unsigned char led_behavior = dsPIC_reg[REG_FORCE_LED1+i].val;
        switch(led_behavior) {
            case LED_FORCE_OFF : 
                ws2812b_setState(i, 0);
                break;
            case LED_FORCE_RED : 
                ws2812b_setColor(i, RED);
                break;
            case LED_FORCE_GREEN : 
                ws2812b_setColor(i, GREEN);
                break;
            case LED_FORCE_BLUE : 
                ws2812b_setColor(i, BLUE);
                break;
            case LED_CONTROLED_BY_LIDAR : 
            default : 
                ws2812b_setColor(i, (obstacles_detected&(1<<i))==0?OFF_BLACK:RED);
                break;
        }
    }
}

// ============================================================
//                          WATCHDOG
// ============================================================
void Init_Watchdog()
{
   RCONbits.SWDTEN = 1; 
}
void Refresh_Watchdog()
{
   ClrWdt();    
}

// ___________________________________________________________
// TODO : à mettre côté XBEE
void uart_irq_rx_callback(unsigned char data)
{
    LATAbits.LATA4 = ~LATAbits.LATA4;
    // pour les essais uniquement
    //uart_send_byte(data);
    xbee_decode(data);
    
}

// ___________________________________________________________
void uart2_irq_rx_callback(unsigned char data)
{
}


// ============================================================
//      FONCTIONS COMMUNES MISES A DISPO PAR L'APPLICATIF
// Ré-implémentation des fonctions en lien avec le hardware
// ============================================================

// ___________________________________________________________
// this method is called by drivers to request a delay on specific hardware
void _app_delay_us(unsigned long delay)
{
    __delay_us(delay);
}

// ___________________________________________________________
// this method is called by drivers to request a delay on specific hardware
void _app_delay_ms(unsigned long delay)
{
    __delay_ms(delay);
}


// ============================================================
//                          XBEE
// Ré-implémentation des fonctions XBEE en lien avec le hardware
// ============================================================
// ___________________________________________________________
// callback : useful data receveid
void xbee_readyBytes_callback(unsigned char *buff_data, unsigned char buff_size, unsigned short source_id)
{
    // TODO : 
    // Utiliser les données
}
// ___________________________________________________________
// this method is called by driver to write a buffer to physical serial port on specific hardware
void xbee_write(unsigned char *buff_data, unsigned char buff_size)
{
    uart_send(buff_data, buff_size);
}

