// DSPIC33EP512GP502 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS      = PGD3      // ICD Communication Channel Select bits (Communicate on PGEC3 and PGED3)
#pragma config JTAGEN   = OFF       // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1  = OFF       // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2  = OFF       // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)

// FWDT
#pragma config WDTWIN   = WIN25     // Watchdog Window Select bits (WDT Window is 25% of WDT period)
#pragma config WDTPOST  = PS32768   // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE   = PR128     // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN   = ON        // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS   = OFF       // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN   = OFF       // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD   = NONE      // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
// Pour gagner une pin GPIO sur la sortie OSC2, changer la config en :
//#pragma config OSCIOFNC = ON       // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY  = ON        // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM    = CSDCMD    // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC    = FRCPLL    // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL))
#pragma config IESO     = OFF       // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP     = OFF       // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP      = OFF       // General Segment Code-Protect bit (General Segment Code protect is Disabled)
// #pragma config statements should precede project file includes.

#include <xc.h>
#include <libpic30.h>
#include "eeprom.h"
#include "lidar_registers.h"
#include "General.h"

// Constates
#define TIMER_PERIOD  0x3FFF

// Variables globales
volatile unsigned char tick_timer=0;


// Prototypes des fonctions locales
void Sequenceur(void);
void Init_Timer1(void);

// Prototypes des fonctions externes 


// ____________________________________
//	main entry point
int main ( void )
{
 
 Init_EEPROM(); 
 lidar_registers_init();
 //Init_Ports();
 //Init_Registers();
 Init_Timer1();
 //i2c1_init(dsPIC_reg[REG_I2C_8BITS_ADDRESS].val);  // restitution de la valeur configurée en EEPROM
 
	while(1)
	{
        lidar_registers_management();
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

  // ______________________________
  cpt10msec++;
  if (cpt10msec >= TEMPO_10msec) {
  	cpt10msec = 0;

  }

  // ______________________________
  cpt20msec++;
  if (cpt20msec >= TEMPO_20msec) {
  	cpt20msec = 0;

  }


  // ______________________________
  cpt50msec++;
  if (cpt50msec >= TEMPO_50msec) {
  	cpt50msec = 0;
							
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

  }    
}


/*---------------------------------------------------------------------
  Function Name: Init_Timer1
  Description:   Initialize Timer1
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void Init_Timer1(void)
{
	T1CON = 0;						/* ensure Timer 1 is in reset state */
 	IFS0bits.T1IF = 0;				/* reset Timer 1 interrupt flag */
	IPC0bits.T1IP = 4;				/* set Timer1 interrupt priority level to 4 */
 	IEC0bits.T1IE = 1;				/* enable Timer 1 interrupt */
	PR1 = TIMER_PERIOD;				/* set Timer 1 period register */
	T1CONbits.TCKPS = 2;			/* select Timer1 Input Clock Prescale */
	T1CONbits.TCS = 0;			 	/* select external timer clock */
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
 	IFS0bits.T1IF = 0;				/* reset timer interrupt flag	*/
}	
	

// ___________________________________________________________
void Init_Ports(void)
{
    /* 	Initialize ports */
    LATA  = 0x0000;             // set latch levels
    TRISA = 0xFFFF;             // set IO as inputs
    // Configuration de la LED sur la carte MICTOSTICK
    TRISAbits.TRISA4 = 0;       // set IO as outputs

    // STOR1
    LATBbits.LATB6  = 0;       // set latch levels
    TRISBbits.TRISB6 = 0;      // set IO as outputs

    // STOR2
    LATBbits.LATB7  = 0;       // set latch levels
    TRISBbits.TRISB7 = 0;      // set IO as outputs

    // STOR3
    LATBbits.LATB10  = 0;       // set latch levels
    TRISBbits.TRISB10 = 0;      // set IO as outputs

    // STOR4
    LATBbits.LATB11  = 0;       // set latch levels
    TRISBbits.TRISB11 = 0;      // set IO as outputs

    // STOR5
    LATBbits.LATB12  = 0;       // set latch levels
    TRISBbits.TRISB12 = 0;      // set IO as outputs

    // STOR6
    LATBbits.LATB13  = 0;       // set latch levels
    TRISBbits.TRISB13 = 0;      // set IO as outputs

    // STOR7
    LATBbits.LATB14  = 0;       // set latch levels
    TRISBbits.TRISB14 = 0;      // set IO as outputs

    // STOR8
    LATBbits.LATB15  = 0;       // set latch levels
    TRISBbits.TRISB15 = 0;      // set IO as outputs
}




