#include <xc.h>
#include "General.h"
#include "uartdrv.h"

#define BAUDRATE    9600  // Utilisation avec XBEE
#define BRGVAL      ( (F_FCY / BAUDRATE) / 16 ) - 1

// _______________________________________________
void uart_init(void)
{	
    // Configuration du baudrate
    U1BRG = BRGVAL;
    
    // Configuration des pins
    //  Fonction U1RX mappée sur pin RP42 (pin 21)
    //  Fonction U1TX mappée sur pin RP43 (pin 22)
    RPINR18bits.U1RXR   = 0x2A;     // (0x2A = RP42)
    RPOR4bits.RP43R     = 0x01;     // (0x01 = U1TX)

    U1MODEbits.UARTEN   = 1;        // Enable UART
    U1STAbits.UTXEN     = 1;        // Enable UART transmit
    
    U1STAbits.URXISEL   = 0;        // Interrupt after one RX character is received;
    
    // Mode loopback pour les tests uniquement 
    //U1MODEbits.LPBACK   = 1;
    
    IEC0bits.U1RXIE     = 1;        // Enable interrupt
}

// _______________________________________________
void uart_send_byte(unsigned char data)
{
    while(U1STAbits.UTXBF);
    U1TXREG = data; 
}

// _______________________________________________
void uart_send(unsigned char *buff, unsigned short len)
{
    unsigned short i;
    for (i=0; i<len; i++) {
       uart_send_byte(buff[len]);
    } 
}

// _______________________________________________
void __attribute__ ( (interrupt, no_auto_psv) ) _U1RXInterrupt( void )
{
    IFS0bits.U1RXIF = 0; // Clear the RX Interrupt Flag
    uart_irq_rx_callback(U1RXREG);
}
// _______________________________________________
void __attribute__ ( (interrupt, no_auto_psv) ) _U1ErrInterrupt( void )
{
    IFS4bits.U1EIF = 0;  // Clear the Error Interrupt Flag
}

// _______________________________________________
// Réimplémente la fonction write pour bénéficier de la fonction printf
// en redirigeant le flux de sortie vers l'UART
// Pour optimiser, tous les flux de sortie sont redirigés vers l'UART : fwrite, printf, ... (donc pas de test sur handle)
// Voir doc Microchip DS50001456
int __attribute__((__section__(".libc.write")))
  write(int handle, void *buffer, unsigned int len) 
{
    (void)handle;
    int i;
    for (i=0; i<len; i++) {
        uart_send_byte(*(((unsigned char*)buffer)+i));
    }
    return(len);
}
