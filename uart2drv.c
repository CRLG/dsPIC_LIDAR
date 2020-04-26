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

// ==============================================
// DMA 1
// ==============================================
#define NB_OF_LEDS   16     // 16 LED à contrôler
#define BITS_PER_LED 24     // 1 LED = 24 bits (1 octet UART utilisé pour coder 1 bit)
#define U2_DMA_BUFFER_SIZE (NB_OF_LEDS*BITS_PER_LED)
#define WS2812B_CODE_0 0xFC
#define WS2812B_CODE_1 0xE0

volatile static unsigned char u2_dma_transfer_in_progress=0;
unsigned int UART2_DMA_BUFFER[U2_DMA_BUFFER_SIZE];

void uart2_send_buffer_dma(unsigned short size)
{
    int i;
    
    // Vérifie si un transfert n'est pas déjà en cours
    while (u2_dma_transfer_in_progress);
    while(U2STAbits.UTXBF);
    
    for (i=0; i<size; i++) {
        if (i%2)    UART2_DMA_BUFFER[i] = WS2812B_CODE_1;
        else        UART2_DMA_BUFFER[i] = WS2812B_CODE_0;
    }
    
    DMA1CON = 0x2001; // One-Shot, Post-Increment, RAM-to-Peripheral
    DMA1CNT = size-1;
    DMA1REQ = 0x001F; // Select UART2 transmitter
    DMA1PAD = (volatile unsigned int) &U2TXREG;
    DMA1STAL = __builtin_dmaoffset(UART2_DMA_BUFFER);
    DMA1STAH = 0x0000;
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt

    DMA1CONbits.CHEN = 1; // Enable DMA0 channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
    u2_dma_transfer_in_progress = 1;
}


void __attribute__((__interrupt__,no_auto_psv)) _DMA1Interrupt(void)
{
 IFS0bits.DMA1IF = 0; // Clear the DMA0 Interrupt Flag
 u2_dma_transfer_in_progress = 0;
}