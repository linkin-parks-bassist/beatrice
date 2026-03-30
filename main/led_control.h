#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

#define LED_PIN_1 1
#define LED_PIN_2 2

int init_leds();
void swap_leds();
void update_leds();

extern int active;

#endif
