#ifndef _LEDS_H_
#define _LEDS_H_

/*
 * Initialise the LED GPIOs
 */
void leds_init(void);

/*
 * red: bitmask of LEDs to set red
 * green:  bitmask of LEDs to set green
 * mask: bitmask of LEDs to set values for
 */
void leds_set(uint8_t red, uint8_t green, uint8_t mask);

#endif // _LEDS_H_
