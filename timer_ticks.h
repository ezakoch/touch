#ifndef TIMER_TICKS_H_
#define TIMER_TICKS_H_

#include <avr/io.h>

// the timing resolution can go down to a couple dozen microseconds if you need it to, though
// you'll probably have to change the prescaler from /64 to /8

#define US_PER_TIMER_TICK 128
#define CYCLES_PER_US (F_CPU / 1000000)

// the timing system uses an 8-bit timer (timer0)
void init_timer (void);

uint64_t get_timer_ticks (void);
uint64_t us_elapsed (void);
uint64_t us_since (const uint64_t timer_tick);

// the following macro ugliness tries to determine a good prescaler value at compile time:
#define PRESCALER 1
#if (CYCLES_PER_US * US_PER_TIMER_TICK / PRESCALER >= 256)
	#undef PRESCALER
	#define PRESCALER 8
#endif
#if (CYCLES_PER_US * US_PER_TIMER_TICK / PRESCALER >= 256)
	#undef PRESCALER
	#define PRESCALER 64
#endif
#if (CYCLES_PER_US * US_PER_TIMER_TICK / PRESCALER >= 256)
	#undef PRESCALER
	#define PRESCALER 256
#endif
#if (CYCLES_PER_US * US_PER_TIMER_TICK / PRESCALER >= 256)
	#undef PRESCALER
	#define PRESCALER 1024
#endif
#if (CYCLES_PER_US * US_PER_TIMER_TICK / PRESCALER >= 256)
	#error US_PER_TIMER_TICK value is too high, will not fit in OCR0A no matter what prescaler is used
#endif

#if (CYCLES_PER_US * US_PER_TIMER_TICK / PRESCALER <= 1)
	#error US_PER_TIMER_TICK value is too small
#endif
#if (CYCLES_PER_US * US_PER_TIMER_TICK % PRESCALER != 0)
	#warning US_PER_TIMER_TICK * CYCLES_PER_US does not divide cleanly by the PRESCALER value, which might cause timer inaccuracy
#endif


#endif
