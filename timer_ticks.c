#include "timer_ticks.h"
#include <avr/interrupt.h>

volatile uint64_t timer_ticks = 0;  // number of timer0 interrupts since either power-on or the last call to clear_timer()

ISR (TIMER0_COMPA_vect)
{
	timer_ticks++;
	
	// with US_PER_TIMER_TICK = 200, and using a 64-bit counter, ticks will overflow about once every 117 million years
}

void init_timer (void)
{
	TCCR0A = (1 << WGM01);  // CTC mode (TOP = OCR0A), OC0A pin disconnected
	
	#if PRESCALER == 1
		TCCR0B = (1 << CS00);  // use /1 prescaling
	#elif PRESCALER == 8
		TCCR0B = (1 << CS01);  // use /8 prescaling
	#elif PRESCALER == 64
		TCCR0B = (1 << CS01) | (1 << CS00);  // use /64 prescaling
	#elif PRESCALER == 256
		TCCR0B = (1 << CS02);  // use /256 prescaling
	#elif PRESCALER == 1024
		TCCR0B = (1 << CS02) | (1 << CS00);  // use /1024 prescaling
	#else
		#define STRINGIFY(s) XSTRINGIFY(s)
		#define XSTRINGIFY(s) #s
		#pragma message ("Unknown prescaler value " STRINGIFY(PRESCALER))
		#error Unknown prescaler value
	#endif
	
	TIMSK0 = (1 << OCIE0A);  // enable timer 0 compare match A interrupt
	
	// generate an interrupt every (F_CPU / 1000000 / 64) * US_PER_TIMER_TICK microseconds
	OCR0A = CYCLES_PER_US * US_PER_TIMER_TICK / 64;
}

uint64_t get_timer_ticks (void)
{
	cli();
	const uint64_t ticks = timer_ticks;
	sei();
	
	return ticks;
}

uint64_t us_elapsed (void)
{
	return get_timer_ticks() * US_PER_TIMER_TICK;
}

uint64_t us_since (const uint64_t timer_tick)
{
	return (get_timer_ticks() - timer_tick) * US_PER_TIMER_TICK;
}

void clear_timer (void)
{
	cli();
	timer_ticks = 0;
	sei();
}
