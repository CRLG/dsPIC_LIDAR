#include <xc.h>
#include "leds_ws2812b.h"
#include "uart2drv.h"

#define NB_OF_LEDS   16     // 16 LED à contrôler
#define BITS_PER_LED 24     // 1 LED = 24 bits (1 octet UART utilisé pour coder 1 bit)
#define LED_DMA_BUFFER_SIZE (NB_OF_LEDS*BITS_PER_LED)
#define WS2812B_CODE_0 0xFC
#define WS2812B_CODE_1 0xE0

static unsigned int LED_WS2812B_DMA_BUFFER[LED_DMA_BUFFER_SIZE];
static unsigned char ws2812b_dma_transfer_in_progress = 0;


// _______________________________________________
void ws2812b_init()
{
    int i;
    for (i=0; i<LED_DMA_BUFFER_SIZE; i++) {
       LED_WS2812B_DMA_BUFFER[i] = WS2812B_CODE_0; 
    }
}


// _______________________________________________
//                 LED0                |                LED1 ...
// [     G;         R;        B]       |[     G;         R;        B]       
// [ [G7...G0] [R7....R0] [B7...B0] ]  |[ [G7...G0] [R7....R0] [B7...B0] ]
//     8bits     8bits      8bits      |    8bits     8bits      8bits
void ws2812b_setLED(unsigned short index, unsigned char R, unsigned char G, unsigned char B)
{
    unsigned short i = index * 24;
    int j;
    
    if (index >= NB_OF_LEDS) return;
    // Green
    for (j=7; j>=0; j--) {
        LED_WS2812B_DMA_BUFFER[i++] = ((G>>j)&0x01)?WS2812B_CODE_1:WS2812B_CODE_0;
    }
    // Red
    for (j=7; j>=0; j--) {
        LED_WS2812B_DMA_BUFFER[i++] = ((R>>j)&0x01)?WS2812B_CODE_1:WS2812B_CODE_0;
    }
    // Blue
    for (j=7; j>=0; j--) {
        LED_WS2812B_DMA_BUFFER[i++] = ((B>>j)&0x01)?WS2812B_CODE_1:WS2812B_CODE_0;
    }
}

// _______________________________________________
static void ws2812b_send_buffer_dma()
{
    // Vérifie si un transfert n'est pas déjà en cours
    while (ws2812b_dma_transfer_in_progress);
    while(U2STAbits.UTXBF);

    DMA1CON = 0x2001; // One-Shot, Post-Increment, RAM-to-Peripheral
    DMA1CNT = LED_DMA_BUFFER_SIZE-1;
    DMA1REQ = 0x001F; // Select UART2 transmitter
    DMA1PAD = (volatile unsigned int) &U2TXREG;
    DMA1STAL = __builtin_dmaoffset(LED_WS2812B_DMA_BUFFER);
    DMA1STAH = 0x0000;
    IFS0bits.DMA1IF = 0; // Clear DMA Interrupt Flag
    IEC0bits.DMA1IE = 1; // Enable DMA interrupt

    DMA1CONbits.CHEN = 1; // Enable DMA0 channel
    DMA1REQbits.FORCE = 1; // Manual mode: Kick-start the 1st transfer
    ws2812b_dma_transfer_in_progress = 1;
}

// _______________________________________________
void __attribute__((__interrupt__,no_auto_psv)) _DMA1Interrupt(void)
{
 IFS0bits.DMA1IF = 0; // Clear the DMA0 Interrupt Flag
 ws2812b_dma_transfer_in_progress = 0;
}

// _______________________________________________
void ws2812b_periodicTask()
{
   ws2812b_send_buffer_dma(LED_WS2812B_DMA_BUFFER, LED_DMA_BUFFER_SIZE);
}