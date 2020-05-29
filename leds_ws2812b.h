#ifndef _LEDS_WS2812B_H_
#define	_LEDS_WS2812B_H_

// RGB
#define OFF_BLACK   (0x000000)
#define RED         (0xFF0000)
#define GREEN       (0x00FF00)
#define BLUE        (0x0000FF)
#define PURPLE      (0x990066)
#define YELLOW      (0xFF9900)
#define WHITE       (0xFFFFFF)
#define CHARTREUSE  (0x7FFF00)
#define TURQUOISE   (0x40E0D0)
#define OLIVE       (0x808000)

void ws2812b_init();

void ws2812b_setState(unsigned short index, unsigned char state);
void ws2812b_setColor(unsigned short index, unsigned long rgb);
void ws2812b_configOnOffColor(unsigned short index, unsigned long on_rgb, unsigned long off_rgb);
void ws2812b_setPattern(unsigned short index, unsigned char ton, unsigned char toff);

void ws2812b_periodicTask();

#endif	/* _LEDS_WS2812B_H_ */

