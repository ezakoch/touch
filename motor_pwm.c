#include "motor_pwm.h"

#define MOTOR_PWM_COUNTS (MOTOR_PWM_FREQ * 2)

#define MOTOR_PRESCALER 1
#if ((F_CPU / MOTOR_PRESCALER) / MOTOR_PWM_COUNTS > 65535)
	#undef MOTOR_PRESCALER
	#define MOTOR_PRESCALER 8
#endif
#if ((F_CPU / MOTOR_PRESCALER) / MOTOR_PWM_COUNTS > 65535)
	#undef MOTOR_PRESCALER
	#define MOTOR_PRESCALER 64
#endif
#if ((F_CPU / MOTOR_PRESCALER) / MOTOR_PWM_COUNTS > 65535)
	#undef MOTOR_PRESCALER
	#define MOTOR_PRESCALER 256
#endif
#if ((F_CPU / MOTOR_PRESCALER) / MOTOR_PWM_COUNTS > 65535)
	#undef MOTOR_PRESCALER
	#define MOTOR_PRESCALER 1024
#endif
#if ((F_CPU / MOTOR_PRESCALER) / MOTOR_PWM_COUNTS > 65535)
	#error MOTOR_PWM_COUNTS is too low
#endif

#if ((F_CPU / MOTOR_PRESCALER) / MOTOR_PWM_COUNTS < 100)
	#error MOTOR_PWM_COUNTS is too high
#endif

static motor_pwm_channel output;


void init_motor_pwm (motor_pwm_channel output_channel)
{
	// phase and frequency-correct PWM, counting from 0 up to ICR1
	TCCR1B = (1 << WGM13);
	
	// timer top value
	ICR1 = (F_CPU / MOTOR_PRESCALER) / MOTOR_PWM_COUNTS;
	
	// initial duty cycle of 0
	OCR1A = 0;
	
	// initial counter value of 0
	TCNT1 = 0;
	
	// output pin selection
	output = output_channel;
	switch (output)
	{
		case MOTOR_CHANNEL_1A:
			DDRB |= (1 << 5);
			TCCR1A = (1 << COM1A1);
			break;
		case MOTOR_CHANNEL_1B:
			DDRB |= (1 << 6);
			TCCR1A = (1 << COM1B1);
			break;
		case MOTOR_CHANNEL_1C:
			DDRB |= (1 << 7);
			TCCR1A = (1 << COM1C1);
			break;
		case MOTOR_CHANNEL_NONE:
			TCCR1A = 0;
			break;
	}
	
	// timer prescaler
	#if MOTOR_PRESCALER == 1
		TCCR1B |= (1 << CS10);  // use /1 prescaling
	#elif MOTOR_PRESCALER == 8
		TCCR1B |= (1 << CS11);  // use /8 prescaling
	#elif MOTOR_PRESCALER == 64
		TCCR1B |= (1 << CS11) | (1 << CS10);  // use /64 prescaling
	#elif MOTOR_PRESCALER == 256
		TCCR1B |= (1 << CS12);  // use /256 prescaling
	#elif MOTOR_PRESCALER == 1024
		TCCR1B |= (1 << CS12) | (1 << CS10);  // use /1024 prescaling
	#else
		#define STRINGIFY(s) XSTRINGIFY(s)
		#define XSTRINGIFY(s) #s
		#pragma message ("Unknown prescaler value " STRINGIFY(MOTOR_PRESCALER))
		#error Unknown prescaler value
	#endif
}

static inline void set_motor_duty (uint16_t compare)
{
	switch (output)
	{
		case MOTOR_CHANNEL_1A:
			OCR1A = compare;
			break;
		case MOTOR_CHANNEL_1B:
			OCR1B = compare;
			break;
		case MOTOR_CHANNEL_1C:
			OCR1C = compare;
			break;
		case MOTOR_CHANNEL_NONE:
			break;
	}
}

static inline uint16_t get_motor_duty (void)
{
	switch (output)
	{
		case MOTOR_CHANNEL_1A:
			return OCR1A;
			break;
		case MOTOR_CHANNEL_1B:
			return OCR1B;
			break;
		case MOTOR_CHANNEL_1C:
			return OCR1C;
			break;
		case MOTOR_CHANNEL_NONE:
			break;
	}
	
	return 0;
}

void set_motor_duty_pct (uint8_t duty)
{
	if (duty > 100)
		duty = 100;
	
	set_motor_duty ((uint16_t)(((uint32_t)duty * (uint32_t)ICR1) / 100));
}

void set_motor_duty_8bit (uint8_t duty)
{
	set_motor_duty ((uint16_t)(((uint32_t)duty * (uint32_t)ICR1) / 256));
}

uint8_t get_motor_duty_pct (void)
{
	return (uint32_t)get_motor_duty() * 100ul / (uint32_t)ICR1;
}

uint8_t get_motor_duty_8bit (void)
{
	return (uint32_t)get_motor_duty() * 256ul / (uint32_t)ICR1;
}
