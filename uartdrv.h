#ifndef _UART_DRIVER_H_
#define _UART_DRIVER_H_

void uart_init(void);
void uart_send_byte(unsigned char data);
void uart_send(unsigned char *buff, unsigned short len);

// Fonction à ré-implémenter dans l'applicatif
// Le contenu de la fonction doit être le plus court possible car toujours sous interruption
void uart_irq_rx_callback(unsigned char data);

#endif  // _UART_DRIVER_H_
