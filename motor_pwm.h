#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <avr/io.h>

#define MOTOR_PWM_FREQ 40000

typedef enum
{
	MOTOR_CHANNEL_1A,
	MOTOR_CHANNEL_1B,
	MOTOR_CHANNEL_1C,
	MOTOR_CHANNEL_NONE
} motor_pwm_channel;

void init_motor_pwm (motor_pwm_channel channel);
void set_motor_duty_pct (uint8_t duty);  // duty should go from 0 to 100
void set_motor_duty_8bit (uint8_t duty);  // duty can be from 0 to 255

#endif