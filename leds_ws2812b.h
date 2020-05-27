#ifndef _LEDS_WS2812B_H_
#define	_LEDS_WS2812B_H_

void ws2812b_init();
void ws2812b_setLED(unsigned short index, unsigned char R, unsigned char G, unsigned char B);
void ws2812b_periodicTask();

#endif	/* _LEDS_WS2812B_H_ */

