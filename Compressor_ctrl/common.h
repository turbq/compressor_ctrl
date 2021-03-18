/*
 * common.h
 *
 * Created: 13.12.2019 14:51:29
 *  Author: Legkiy
 */ 


#ifndef COMMON_H_
#define COMMON_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/*
 *	Pin config
 */
#define START_BUTTON	PD0
#define PRESSURE1		PD1
#define PRESSURE2_1		PD3
#define PRESSURE2_2		PD4

#define HIGH 1
#define LOW  0

/* timer */
typedef struct {
	uint32_t glob_sec;       // uptime in seconds
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint16_t pwr_dn;
} daytime;

typedef union
{
	uint8_t pind;
	struct  
	{
		unsigned pd0:1;
		unsigned pd1:1;
		unsigned :1;
		unsigned pd3:1;
		unsigned pd4:1;
		//reserved
	};
}InputStates;

/*
 *	hw.c prototypes
 */
void init_port(void);
void init_tim(void);

void pin_handle(uint8_t);
void pin_routine(void);

void hw_init(void);

#endif /* COMMON_H_ */