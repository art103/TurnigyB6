#ifndef _BALANCER_H_
#define _BALANCER_H_

/*
 * Initialise the balancer GPIOs
 */
void balancer_init(void);

/*
 * channels: bitmask of channel values (1 = on, 0 = off)
 * mask: bitmask of channels to set values for
 */
void balancer_set(uint8_t channels, uint8_t mask);

/*
 * Turn off the balancer
 */
void balancer_off(void);

#endif // _BALANCER_H_
