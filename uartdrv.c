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
    //  Fonction U1RX mappée sur pin RPI44 (pin 23)
    //  Fonction U1TX mappée sur pin RP43 (pin 22)
    RPINR18bits.U1RXR   = 0x2C;     // (0x2C = RPI44)
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


// ==============================================
// DMA 0
// ==============================================
#define UART1_DMA_BUFFER_SIZE (256)

volatile static unsigned char u1_dma_transfer_in_progress=0;
unsigned int UART1_DMA_BUFFER[UART1_DMA_BUFFER_SIZE];

void uart_send_buffer_dma(unsigned short size)
{
    // Vérifie si un transfert n'est pas déjà en cours
    while (u1_dma_transfer_in_progress);
    while(U1STAbits.UTXBF);
    
    DMA0CON = 0x2001; // One-Shot, Post-Increment, RAM-to-Peripheral
    DMA0CNT = size-1;
    DMA0REQ = 0x000C; // Select UART1 transmitter
    DMA0PAD = (volatile unsigned int) &U1TXREG;
    DMA0STAL = __builtin_dmaoffset(UART1_DMA_BUFFER);
    DMA0STAH = 0x0000;
    IFS0bits.DMA0IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1; // Enable DMA interrupt

    DMA0CONbits.CHEN = 1; // Enable DMA0 channel
    DMA0REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
    u1_dma_transfer_in_progress = 1;
}

void __attribute__((__interrupt__,no_auto_psv)) _DMA0Interrupt(void)
{
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag
    u1_dma_transfer_in_progress = 0;
}
