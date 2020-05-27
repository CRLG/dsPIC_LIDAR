#include <xc.h>
#include "General.h"
#include "uart2drv.h"

#define BRGVAL      0
// _______________________________________________
// Initialisation de l'UART pour piloter des LEDs WS2812B
// ! Attention : signal TX inversé
void uart2_init(void)
{	
    // Configuration du baudrate
    U2BRG = BRGVAL;
    
    // Configuration des pins
    //  Fonction U2TX mappée sur pin RP42 (pin 21)
    RPOR4bits.RP42R     = 0x03;     // (0x03 = U2TX)

    U2MODEbits.UARTEN   = 1;        // Enable UART
    U2MODEbits.BRGH     = 1;        // Double la fréquence d'horloge
    U2STAbits.UTXEN     = 1;        // Enable UART transmit
    
    U2STAbits.URXISEL   = 0;        // Interrupt after one RX character is received;
    U2STAbits.TXINV     = 1;        // Inverse le signal
    // Mode loopback pour les tests uniquement 
    //U2MODEbits.LPBACK   = 1;
    
    IEC1bits.U2RXIE     = 1;        // Enable interrupt
}

// _______________________________________________
void uart2_send_byte(unsigned char data)
{
    while(U2STAbits.UTXBF);
    U2TXREG = data; 
}

// _______________________________________________
void uart2_send(unsigned char *buff, unsigned short len)
{
    unsigned short i;
    for (i=0; i<len; i++) {
       uart2_send_byte(buff[len]);
    } 
}

// _______________________________________________
void __attribute__ ( (interrupt, no_auto_psv) ) _U2RXInterrupt( void )
{
    IFS1bits.U2RXIF = 0; // Clear the RX Interrupt Flag
    uart2_irq_rx_callback(U2RXREG);
}
// _______________________________________________
void __attribute__ ( (interrupt, no_auto_psv) ) _U2ErrInterrupt( void )
{
    IFS4bits.U2EIF = 0;  // Clear the Error Interrupt Flag
}

