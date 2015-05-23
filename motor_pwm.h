#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <avr/io.h>

#define MOTOR_PWM_FREQ 40000

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

typedef enum
{
	MOTOR_CHANNEL_1A,
	MOTOR_CHANNEL_1B,
	MOTOR_CHANNEL_1C,
	MOTOR_CHANNEL_NONE
} motor_pwm_channel;

void init_motor_pwm (motor_pwm_channel channel);
void set_motor_duty_pct (uint8_t duty);  /* duty should go from 0 to 100 */
void set_motor_duty_8bit (uint8_t duty);  /* duty can be from 0 to 255 */
uint8_t get_motor_duty_pct (void);  /* returns the duty cycle as a value from 0 to 100 */
uint8_t get_motor_duty_8bit (void);  /* returns the duty cycle as a value from 0 to 255 */

uint16_t get_motor_raw_PWM (void);
void set_motor_raw_PWM (const uint16_t pwm);

#endif