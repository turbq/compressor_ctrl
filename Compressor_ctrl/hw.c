/*
 * hw.c
 *
 * Created: 13.12.2019 14:55:55
 *  Author: Legkiy
 */

#include <stdbool.h>
#include "common.h"

 

volatile daytime uptime;
InputStates input;

//sleep flag shows if nothing happens until last pwr_dn increment
volatile bool f_sleep = true;
static volatile uint32_t t_start, t_work, t_stop;

ISR(TIMER0_OVF_vect)
{
	static uint16_t ovf_cnt = 0;
	ovf_cnt++;
	if (!f_sleep){//reset pwr_dn counter when some actions executed
		f_sleep = true;
		uptime.pwr_dn = 0;
	}
	if (ovf_cnt == 1000){
		uptime.glob_sec++;
		uptime.sec++;
		uptime.pwr_dn++;
		ovf_cnt = 0;
		//PINB = 1<<PB5;
	}
	if (uptime.sec == 60){
		uptime.sec = 0;
		uptime.min++;
		//PINB = 1<<PB4;
	}
	if (uptime.min == 60){
		uptime.min = 0;
		uptime.hour++;
	}
	if (uptime.hour == 24){
		uptime.hour = 0;
		uptime.day++;
	}
	if (uptime.pwr_dn == 300){ //power down after ~5 mins
		uptime.pwr_dn = 0;
		//set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	}
	if ((ovf_cnt%20)){
		return;
	}
	if ((t_stop)&&(t_stop+2 == uptime.glob_sec)){
		if (input.pd3 == LOW){
			PORTB &= ~_BV(PB5);
			t_start = 0;
			t_work = 0;
		}
	} else
	if ((!t_stop)&&(input.pd4 == HIGH)){
		t_stop = uptime.glob_sec;
	} else
	
 	if (t_work){
 		if (input.pd0 == LOW){ //pressed start when working => stop
 			PORTB &= ~_BV(PB4);
 			//PORTB |= _BV(PB5);
 			//t_work = 0;
			//t_start = uptime.glob_sec;
 		}
 		if ((uptime.glob_sec - t_work) == 10){
 			PORTB &= ~_BV(PB4);
 			//t_work = 0;
 		}
		if (input.pd1 == LOW){
			PORTB &= ~_BV(PB4);
			PORTB |= _BV(PB5);
			t_work = 0;
		}
 		f_sleep = false;
 	} else
 	
 	//if start pressed this should exist
 	if ((t_start) && ((t_start+2)==uptime.glob_sec)){
 		if (input.pd1 == LOW){
 			PORTB &= ~_BV(PB4);
			//PORTB |= _BV(PB5);
 		} else {
			t_work = uptime.glob_sec;
		}
 		t_start = 0;
 	} else
 	
 	if ((input.pd0 == LOW) && (!t_start)){
 		t_start = uptime.glob_sec;
		t_stop = 0;
 		PORTB |= _BV(PB4);
 	}
}

void init_port()
{
	//MCUCR |= (1<<PUD);
 	DDRA = 0x07;PORTA = 0x00;
 	DDRB = 0xff;PORTB = 0x00;
 	DDRD = 0x7f;PORTD = 0x00;
	/* Configure PB4 PB5 as output */
	DDRB |= _BV(PB4)|_BV(PB5);
	/* Turn on input pins */
 	DDRD &= (~(_BV(START_BUTTON)) & ~(_BV(PRESSURE1)) & ~(_BV(PRESSURE2_1)) & ~(_BV(PRESSURE2_2)));
 	/* Turn on pull-up */
	PORTD |= _BV(START_BUTTON) | _BV(PRESSURE1) | _BV(PRESSURE2_1) | _BV(PRESSURE2_2);
	input.pind = 0x0d;
}

/*
 * 
 * Generated freq is 
 */
void init_tim()
{	
	/* Timer clock = I/O clock 1000000 / 8 = 125000 */
	TCCR0A = (1<<WGM01)|(1<<WGM00);
	TCCR0B = (1<<CS01)|(1<<WGM02);
	/* Timer clock = I/O clock 128000 */
// 	TCCR0A = (1<<WGM01)|(1<<WGM00);
// 	TCCR0B = (1<<CS00)|(1<<WGM02);
	/* Output compare register A is TOP value */
	OCR0A = 125;
	/* Clear overflow flag */
	TIFR = 1<<TOV0;
	/* Enable Overflow Interrupt */
	TIMSK = (1<<TOIE0);
}

void pin_handle(uint8_t pin)
{
	uint8_t last_state = input.pind & _BV(pin);
	uint8_t state = PIND & _BV(pin);
	
	// compare the buttonState to its previous state
	if (state != last_state) {
		// if the state has changed, increment the counter
		if (state == HIGH) {
			// if the current state is HIGH then the button released
			input.pind |= _BV(pin);
		} else {
			// if the current state is LOW then the button pressed
			input.pind &= ~_BV(pin);
		}
		//probably need some delay there for debounce
	}
}

void pin_routine()
{
	static uint8_t prev = 0x00;
	uint8_t current, changed;
	current = PIND; // get input state of portD as it has now changed
	changed = current ^ prev; // use XOR to find out which bit(s) have changed
	if (changed & _BV(START_BUTTON)) {
		// start
		pin_handle(START_BUTTON);
	}
	if (changed & _BV(PRESSURE1)) {
		// input pressure PD1
		pin_handle(PRESSURE1);
	}
	if (changed & _BV(PRESSURE2_1)) {
		// input pressure PD3
		pin_handle(PRESSURE2_1);
	}
	if (changed & _BV(PRESSURE2_2)) {
		// input pressure PD4
		pin_handle(PRESSURE2_2);
	}
	prev = current; // remember for next time
	input.pind = prev;
}

void hw_init()
{
	/* 
	 * Set CPU clk prescaler
	 */
// 	CLKPR = _BV(CLKPCE);
// 	CLKPR = _BV(CLKPS1);

	init_port();
	init_tim();
	/* analog comparator disable */
	ACSR |= (1<<ACD);
	ACSR &= ~(1<<ACI);
	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
}