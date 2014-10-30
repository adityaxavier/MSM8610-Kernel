#ifndef _SWITCH_KEY_H_
#define _SWITCH_KEY_H_

struct switch_key_platform_data {
	int irq_gpio;
	int irq;
};

#define KEY_PRESS 0x01
#define KEY_RELEASE 0x00

#endif
