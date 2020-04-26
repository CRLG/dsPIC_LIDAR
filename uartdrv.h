#ifndef _UART_DRIVER_H_
#define _UART_DRIVER_H_

extern unsigned int UART1_DMA_BUFFER[];

void uart_init(void);
void uart_send_byte(unsigned char data);
void uart_send(unsigned char *buff, unsigned short len);

// Fonction � r�-impl�menter dans l'applicatif.
// Fonction appel�e lorsqu'un octet est re�u sur l'UART.
// Le contenu de la fonction doit �tre le plus court possible car toujours sous interruption
void uart_irq_rx_callback(unsigned char data);

void uart_send_buffer_dma(unsigned short size);

#endif  // _UART_DRIVER_H_
