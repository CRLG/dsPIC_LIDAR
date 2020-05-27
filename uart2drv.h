#ifndef _UART2_DRIVER_H_
#define _UART2_DRIVER_H_

extern unsigned int UART2_DMA_BUFFER[];

void uart2_init(void);
void uart2_send_byte(unsigned char data);
void uart2_send(unsigned char *buff, unsigned short len);

// Fonction à ré-implémenter dans l'applicatif.
// Fonction appelée lorsqu'un octet est reçu sur l'UART.
// Le contenu de la fonction doit être le plus court possible car toujours sous interruption
void uart2_irq_rx_callback(unsigned char data);

#endif  // _UART2_DRIVER_H_
