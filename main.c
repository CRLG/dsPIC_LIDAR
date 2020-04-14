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
#pragma config WDTPOST  = PS256   // Watchdog Timer Postscaler bits (1:256)
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

// Horloge sur le dsPIC : 
//      - FRC = 7.37 MHz                                    -> Fréquence générée en interne de base 
// Avec activation de la PLL, on a :
//      - Fosc = FRC * (M / (N1 * N2))  (voir note d'application Microchip AN70227E)
//          - M = 50 (voir PLLDIV)
//          - N1 = 2 (voir PLLPRE)
//          - N2 = 4 (void PLLPOST)
//        => Fosc = 7.37 * (50/(2*4)) = 46.0625MHz
//      - Fcy = (Fosc / 2) = 23.03125 MHz (Tcy = 43.41nsec) -> Fréquence pour les instructions CPU
//      - Fp = Fcy                                          -> Fréquence de base pour les prériphériques (Timer, I2C, UART, ...)


// Watchdog : voir note d'application Microchip DS70615C
// Treset_watchdog = 1/Frc * N1 *N2   => Période du watchdog avant reset
//      Avec Frc = Fréquence l'oscillateur interne = 32kHz
//      N1 = 32 (WDTPRE=config 5 bits) ou 128 (WDTPRE = config 7 bits))
//      N2 = WDTPOST 
//      On choisit un période watchdog à 1sec environ
//      => WDTPRE = 128 et WDTPOST = 256
//      => Treset_watchdog = (1/32e3) * 128 * 256 = 1.024sec

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
#include "General.h"

// Constates

// Variables globales
volatile unsigned char tick_timer=0;

// Variables globales externes
extern unsigned short cptPerteComMaster;

// Prototypes des fonctions locales
void Sequenceur(void);
void Init_Timer1(void);
void Init_Ports(void);
void Init_Watchdog();
void Init_IRQ_Priority();
void Refresh_Watchdog();


// ____________________________________
//	main entry point
int main ( void )
{
    // Hardware init
    Init_EEPROM(); 
    Init_Ports();
    Init_IRQ_Priority();
    lidar_registers_init();
    lidar_init();
    i2c_slave_init(dsPIC_reg[REG_I2C_8BITS_ADDRESS].val);  // restitution de la valeur configurée en EEPROM
    i2c_master_init();
    mcp23017_init(I2C_MCP23017_I2C_ADDRESS);
    mcp23017_configDirections(0, 0);
    uart_init();
    Init_Timer1();
    Init_Watchdog();
 
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
  unsigned char data[10] = { 3, 12, 1, 16};
  static unsigned char cptErrorCom = 0;
 
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
 
    lidar_periodic_call();
}

  // ______________________________
  cpt100msec++;
  if (cpt100msec >= TEMPO_100msec) {
    cpt100msec = 0;

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
    
    printf("Hello %c", data[0]);
  }
}


/*---------------------------------------------------------------------
  Function Name: Init_Timer1
  Description:   Initialize Timer1
  Inputs:        None
  Returns:       None
 F_FP = 23.03125 MHz
 Tp = 1/F_FP
 On veut 1 IRQ toutes les 10msec
 Tirq = Tp* Prescaler * (PR1+1)
 => PR1 = Tirq/(Tp*Prescaler) - 1
 Avec Prescaler=64 => PR1 = 3598
-----------------------------------------------------------------------*/
void Init_Timer1(void)
{
	T1CON = 0;						/* ensure Timer 1 is in reset state */
 	IFS0bits.T1IF = 0;				/* reset Timer 1 interrupt flag */
	IPC0bits.T1IP = 4;				/* set Timer1 interrupt priority level to 4 */
 	IEC0bits.T1IE = 1;				/* enable Timer 1 interrupt */
	PR1 = 3598;                     /* set Timer 1 period register */
	T1CONbits.TCKPS = 2;			/* Prescaler = 64 */
	T1CONbits.TCS = 0;			 	/* select internal timer clock */
	T1CONbits.TON = 1;			 	/* enable Timer 1 and start the count */ 
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
    // Configuration de la LED sur la carte MICTOSTICK
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
    
}

