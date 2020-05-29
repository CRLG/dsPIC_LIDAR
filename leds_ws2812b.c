#include <xc.h>
#include "leds_ws2812b.h"
#include "uart2drv.h"

typedef struct 
{
    unsigned char Toff;             // Durée à l'état OFF
    unsigned char Ton;              // Durée à l'état ON
    unsigned long ColorOff;         // Couleur de l'état OFF
    unsigned long ColorOn;          // Couleur de l'état ON
    unsigned char Timer;            // Compteur de temps
    unsigned char CurrentState;     // Mémorise l'état courant OFF ou ON
}tWS2812BPattern;


#define NB_OF_LEDS   16     // 16 LED à contrôler
#define BITS_PER_LED 24     // 1 LED = 24 bits (1 octet UART utilisé pour coder 1 bit)
#define LED_DMA_BUFFER_SIZE (NB_OF_LEDS*BITS_PER_LED)
#define WS2812B_CODE_0 0xFC
#define WS2812B_CODE_1 0xE0

static unsigned int LED_WS2812B_DMA_BUFFER[LED_DMA_BUFFER_SIZE];
static unsigned char ws2812b_dma_transfer_in_progress = 0;
static tWS2812BPattern LED_WS2812B[NB_OF_LEDS];

void ws2812b_setLED_dma(unsigned short index, unsigned char R, unsigned char G, unsigned char B);
void ws2812b_setLED2_dma(unsigned short index, unsigned long rgb);


// _______________________________________________
void ws2812b_init()
{
    int i;
    for (i=0; i<NB_OF_LEDS; i++) {
        LED_WS2812B[i].Ton          = 0;
        LED_WS2812B[i].Toff         = 0;
        LED_WS2812B[i].Timer        = 0;
        LED_WS2812B[i].ColorOn      = BLUE;
        LED_WS2812B[i].ColorOff     = OFF_BLACK;
        
    }
    ws2812b_periodicTask();
    
    /*
    for (i=0; i<LED_DMA_BUFFER_SIZE; i++) {
       LED_WS2812B_DMA_BUFFER[i] = WS2812B_CODE_0; 
    }
    */
}

// =======================================================
//                      API
// =======================================================
void ws2812b_setState(unsigned short index, unsigned char state)
{
    if (index >= NB_OF_LEDS) return;
    
    if (state == 0) {   // Force l'état OFF
        LED_WS2812B[index].Ton  = 0;
        LED_WS2812B[index].Toff = 1;
    }
    else {              // Force l'état ON
        LED_WS2812B[index].Ton  = 1;
        LED_WS2812B[index].Toff = 0;
    }
}

// _______________________________________________
void ws2812b_setColor(unsigned short index, unsigned long rgb)
{
    if (index >= NB_OF_LEDS) return;
    
    LED_WS2812B[index].ColorOn = rgb;
    ws2812b_setState(index, 1);
}

// _______________________________________________
void ws2812b_configOnOffColor(unsigned short index, unsigned long on_rgb, unsigned long off_rgb)
{
    if (index >= NB_OF_LEDS) return;

    LED_WS2812B[index].ColorOn  = on_rgb;
    LED_WS2812B[index].ColorOff = off_rgb;
}

// _______________________________________________
void ws2812b_setPattern(unsigned short index, unsigned char ton, unsigned char toff)
{
    if (index >= NB_OF_LEDS) return;
    
    if ( (LED_WS2812B[index].Ton == ton) && (LED_WS2812B[index].Toff== toff) ) {
        // Ne rien faire
        return;
    }
    LED_WS2812B[index].Ton  = ton;
    LED_WS2812B[index].Toff = toff;
    LED_WS2812B[index].Timer = 0;

}

// =======================================================
//                     DMA BUFFER
// =======================================================
// _______________________________________________
//                 LED0                |                LED1 ...
// [     G;         R;        B]       |[     G;         R;        B]       
// [ [G7...G0] [R7....R0] [B7...B0] ]  |[ [G7...G0] [R7....R0] [B7...B0] ]
//     8bits     8bits      8bits      |    8bits     8bits      8bits
void ws2812b_setLED_dma(unsigned short index, unsigned char R, unsigned char G, unsigned char B)
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
void ws2812b_setLED_dma2(unsigned short index, unsigned long rgb)
{
    unsigned char r = (rgb>>16)&0xFF;
    unsigned char g = (rgb>> 8)&0xFF;
    unsigned char b = (rgb>> 0)&0xFF;
    ws2812b_setLED_dma(index, r, g, b);
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
static void ws2812b_compute_led_state(unsigned short index)
{
    if (LED_WS2812B[index].Ton == 0) {
        LED_WS2812B[index].CurrentState = 0;
        return;
    }
    if (LED_WS2812B[index].Toff == 0) {
        LED_WS2812B[index].CurrentState = 1;
        return;
    }
    
    LED_WS2812B[index].CurrentState = (LED_WS2812B[index].Timer < LED_WS2812B[index].Ton);

    unsigned short period = LED_WS2812B[index].Ton + LED_WS2812B[index].Toff;
    if (++LED_WS2812B[index].Timer > period) {
        LED_WS2812B[index].Timer = 0;
    }
}

// _______________________________________________
void ws2812b_periodicTask()
{
    unsigned short i;
    for (i=0; i<NB_OF_LEDS; i++) {
       ws2812b_compute_led_state(i);
       unsigned long _color = LED_WS2812B[i].CurrentState==0?LED_WS2812B[i].ColorOff:LED_WS2812B[i].ColorOn;
       // met à jour le buffer DMA avant le transfert
       ws2812b_setLED_dma2(i, _color);
    }
    ws2812b_send_buffer_dma(LED_WS2812B_DMA_BUFFER, LED_DMA_BUFFER_SIZE);
}

