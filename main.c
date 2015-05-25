/*---------------------------------------------------------------------------
TOUCH Actuator Controller on M2 (Atmega32u4)
cd Dropbox/TOUCH/m2
date: May 2015
author: Eza Koch and Kent deVillafranca
-----------------------------------------------------------------------------
Library Inclusions
---------------------------------------------------------------------------*/

#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "timer_ticks.h"
#include "pc_communication.h"
#include "motor_pwm.h"
#include "adc.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>

// -----------------------------------------------------------------------------
// Definitions
// -----------------------------------------------------------------------------
#define MAXRPS (83.3 / 5) //Maxon 144325 assembly with gearhead 5000.0/60.0
// (divided by 5 for fudge factor)

#define TICKS 500.0 //counts per turn 110514 encoder

// -----------------------------------------------------------------------------
// Debugging
// -----------------------------------------------------------------------------
// #define DEBUG
// #define DEBUG_TEMP

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
volatile int count=0; //encoder ticks
float error_sum = 0; // Integral sum for PID control

#define US_PER_ACCEL_SAMPLE 1250
#define ACCEL_BUFFER_ELEMENTS (MAX_ACCEL_SAMPLES * 3)
int8_t accel_ring_buffer[ACCEL_BUFFER_ELEMENTS];  // circular buffer to hold acceleration data of multiple PC data packets
int8_t *accel_buffer_start = accel_ring_buffer;  // pointer to the beginning of valid data in the circular buffer
uint64_t accel_buffer_end_us = 0;  // timestamp for the end of the currently stored acceleration data
int8_t *accel_buffer_end = accel_ring_buffer;  // pointer to the end of valid data in the circular buffer (buffer is empty when start==end)

// -----------------------------------------------------------------------------
// Declare Helper Functions
// -----------------------------------------------------------------------------
void driveMotor(int dir, float rps_desired);
void disableMotor(void);

// -----------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------
int main(void){
	
	//Vars
	m_clockdivide(0); //16MHz
	m_disableJTAG();

	// -----------------------------------------------------------------------------
	// Variables
	// -----------------------------------------------------------------------------
	int dir = 0;
	float temp;
	float temp_diff;
	float temp_desired = 600.0; //need a separate sensor to check the surface
	float TECout;
	float Kp_temp = 2.0;
	// float scaling = 1.0;
	// float accel;

	// -----------------------------------------------------------------------------
	// USB COMMUNICATIONS
	// ls /dev/tty.*
	// screen /dev/tty.usbmodem###
	// -----------------------------------------------------------------------------

	m_usb_init();
	init_timer();
	init_motor_pwm (MOTOR_CHANNEL_1B);
	init_adc();

	// -----------------------------------------------------------------------------
	// SETUP INTERRUPTS: PIN CHANGE PCINT0 pin D0
	// -----------------------------------------------------------------------------
 	sei();//enable global interrupts
 	set(PCICR,PCIE0); //pin-change, cleared by default
 	set(PCMSK0,PCINT0); //remove mask for corresponding interrupt

 	// -----------------------------------------------------------------------------
	// SETUP TIMER 4 INTERRUPTS TO HANDLE ACCEL PACKAGES
	// -----------------------------------------------------------------------------
 	TCCR4B = (1 << CS41) | (1 << CS42); //Prescaler /32= 500kHz

	// set Timer4's compare match value so that the match interrupt fires at 800Hz
	TC4H = (625 >> 8);
	OCR4A = (625 & 0xff); //500000/625=800Hz Target Frequency, max 1023

 	set(TIMSK4, OCIE4A); //Interrupt TCNT4 matches OCR4A

	// -----------------------------------------------------------------------------
	// THERMAL ELEMENT USB DEBUG
	// -----------------------------------------------------------------------------
	/*
	m_pwm_timer(3, 10000.0);
	temp = m_adc(F4);
	temp_diff = temp_desired - temp;
	TECout = (float)(temp_diff*Kp_temp)/1023.0; //PID duty cycle for TEC

	#ifdef DEBUG_TEMP
		m_usb_tx_string("TEMP: ");
		m_usb_tx_int((int)(temp));
		m_usb_tx_string("diff: ");
		m_usb_tx_int((int)(temp_diff));
		m_usb_tx_string("duty temp: ");
		m_usb_tx_int((int)(TECout*100.0));
		m_usb_tx_string("\r\n");
	#endif

	m_pwm_duty(3, 1, TECout); //pin C6
	*/

	// -----------------------------------------------------------------------------
	// RUN LOOP
	// -----------------------------------------------------------------------------
	m_red(OFF);m_green(OFF);

	for (;;)
	{
		static uint64_t control_loop_last_run_time = 0;
		
		// run the control loop roughly once every 1000us=1ms
		if (us_elapsed() - control_loop_last_run_time > 1000ul)
		{
			//ADC user input from potentiometer
			cli();
			const uint16_t pot_state = adc_values.pot;
			sei();
			
			#ifdef DEBUG
				m_usb_tx_string("pot_state: ");
				m_usb_tx_int((int)(pot_state));
				m_usb_tx_string("\r\n");
			#endif
			const float y_desired=(float)MAXRPS*(((float)pot_state)/1023.0); //[rps]

			//Digital GPIO: Switching Motor ON/OFF
			if (check(PIND,4)){disableMotor();m_red(ON);m_green(OFF);}
			else{driveMotor(dir, y_desired);m_green(ON);m_red(OFF);}
			
			control_loop_last_run_time = us_elapsed();
		}

		if (received_pc_message())
		{
			process_pc_message();
			
			if (new_pc_data())
			{
				const pc_data *const latest_data = get_pc_data();
				
				if (latest_data->num_accel_samples > 0)
				{
					/* We've received new acceleration data.  Now we need to ensure that the new data gets smoothly
					 * incorporated into the existing data stream, despite the inevitable difference in clock
					 * speeds between the sensor and actuator.
					 */
					
					/*
					// get the number of used spaces in the acceleration buffer
					uint16_t buffer_filled_slots = 0;
					cli();
					const int8_t *buffer_start = accel_buffer_start;
					sei();
					while (buffer_start != accel_buffer_end)
					{
						buffer_filled_slots++;
						
						buffer_start++;
						if (buffer_start >= accel_ring_buffer + ACCEL_BUFFER_ELEMENTS)
							buffer_start = accel_ring_buffer;
					}
					
					// get the number of empty spaces in the buffer
					const uint16_t buffer_empty_slots = ACCEL_BUFFER_ELEMENTS - buffer_filled_slots;
					
					static int64_t adjust_us = 0;
					
					// take the difference between time of the last stored sample in our accel buffer and the time of the first incoming sample
					const int64_t data_time_offset_us = (int64_t)latest_data->timestamp_us - (int64_t)accel_buffer_end_us + adjust_us;
					const int64_t buffer_offset_elements = data_time_offset_us / US_PER_ACCEL_SAMPLE;
					
					if (buffer_offset_elements + latest_data->num_accel_samples >= buffer_empty_slots)
					{  // there's a large discontinuity between data samples that would put our newly-received data outside the buffer bounds
						// TODO: change adjust_us so that large time disparities (eg, one system powered on much earlier/later than the other) disappear
					}
					
					// TODO: add the new data to the appropriate place
					if (offset_elements > 0)
					{
						// There is a gap between the last sample of our current buffer and the first sample of the new data.
						// TODO: Interpolate between them to fill the gap
					}
					else if (offset_elements < 0)
					{
						// The new data overlaps with data in our current buffer.
						// TODO: Average the overlapping data
					}
					*/
					
					// UNTIL THE ABOVE CODE IS FINISHED: Just append the new data immediately after the last entry of the previous packet
					
					for (uint8_t i = 0; i < latest_data->num_accel_samples; i++)
					{
						*accel_buffer_end = latest_data->accel[i];  // store this value at the end of the buffer
						
						accel_buffer_end++;  // move the end of the buffer up one space, wrapping around if we've hit the end of the array
						if (accel_buffer_end >= accel_ring_buffer + ACCEL_BUFFER_ELEMENTS)
							accel_buffer_end = accel_ring_buffer;
						
						// buffer overrun condition
						// if we've run up against the start of the buffer, stop before we begin overwriting valid data
						if ((accel_buffer_end + 1 == accel_buffer_start) ||
							(accel_buffer_end == accel_ring_buffer + ACCEL_BUFFER_ELEMENTS - 1 &&
							 accel_buffer_start == accel_ring_buffer))
						{
							break;
						}
					}
				}  // end of accel_samples > 0 block
			}  // end of new_pc_data() block
		}  // end of received_pc_message() block
	}
}

// -----------------------------------------------------------------------------
//  PID CONTORLLER AND MOTOR DRIVER
// -----------------------------------------------------------------------------

static uint16_t motor_control_pwm = 0;  // value to hold the desired PWM from the motor control loop (actual motor PWM is this plus acceleration)

void driveMotor (int dir, float rps_desired){
	static uint64_t prev_time = 0;
	static float prev_error = 0;
	
	const uint64_t current_time = us_elapsed();
	const float dt = (float)(current_time - prev_time);
	const float Kp = 1.0;
	const float Kd = 0.0;
	const float Ki = 0.00003;

	const float revs = (float)count / (float)TICKS;
	const float rps_actual = revs * 1e6 / dt;
	const float error = rps_desired - rps_actual;

	#ifdef DEBUG
		m_usb_tx_string ("dt (ms) = ");
		m_usb_tx_int ((unsigned int)(dt / 1000));
		m_usb_tx_string ("\r\n");

		m_usb_tx_string("rps_desired: ");
		m_usb_tx_int((int)(rps_desired));
		m_usb_tx_string("\r\n");
		
		m_usb_tx_string("rps_actual: ");
		m_usb_tx_int((int)(rps_actual));
		m_usb_tx_string("\r\n");
		
		m_usb_tx_string("error: ");
		m_usb_tx_int((int)(error));
		m_usb_tx_string("\r\n");
	#endif
	
	error_sum += (error * dt);
	// -----------------------------------------------------------------------------
	// INTEGRAL WINDUP
	// -----------------------------------------------------------------------------
	if (error_sum * Ki > 100.0) //windup
		error_sum = 100.0 / Ki;

	const float error_diff = (error - prev_error) / dt;
	
 	float duty_cycle = (float) ((rps_desired) + (error * Kp) + (error_diff * Kd) + (error_sum * Ki));
 	
 	#ifdef DEBUG
 		m_usb_tx_string ("I term: ");
 		m_usb_tx_int ((int)(error_sum * Ki));
		m_usb_tx_string ("\r\nerror_diff: ");
 		m_usb_tx_int ((int)error_diff);
		m_usb_tx_string("\r\nduty_cycle before cap: ");
		m_usb_tx_int((int)(duty_cycle));
		m_usb_tx_string("\r\n===================\r\n");
	#endif

	// -----------------------------------------------------------------------------
	// CAPPING DUTY CYCLE
	// -----------------------------------------------------------------------------
 	if (duty_cycle > 100.0)
		duty_cycle = 100.0;
	if (duty_cycle < 0.0)
		duty_cycle = 0.0;
	
	set_motor_duty_pct((uint8_t)duty_cycle);
	motor_control_pwm = get_motor_raw_PWM();  // store the motor PWM that was set by the control loop, so it can be modified by the acceleration interrupt
	
	#ifdef DEBUG
		m_usb_tx_string("duty_cycle: ");
		m_usb_tx_int((int)(duty_cycle));
		m_usb_tx_string("\r\n===================\r\n");
	#endif
	
	count = 0;  // reset encoder count
	prev_error = error;  // update the previous error record
 	prev_time = current_time;  // update the elapsed time, to get dt when this function is run next
}

// -----------------------------------------------------------------------------
//  DISABLE MOTORS
// -----------------------------------------------------------------------------
void disableMotor(void){
	set_motor_duty_pct(0);
	error_sum = 0;  // prevent windup when disabled
}
// -----------------------------------------------------------------------------
// INTERRUPTS
// -----------------------------------------------------------------------------
ISR(PCINT0_vect)
{
	count++;  // increment the encoder count
}

ISR(TIMER4_COMPA_vect) //Timer 4 Interrupt Handler: TCNT4 matches OCR4A, update acceleration value
{
	#define ACCEL_SCALE 1
	
	// make sure there's valid accel data available
	if (accel_buffer_start != accel_buffer_end)
	{
		const int16_t new_accel_value = (int16_t)(*accel_buffer_start) * ACCEL_SCALE;  // get the new accel value, multiplied by our scaling factor
		
		// we have to cast to int32_t if adding acceleration to MOTOR_PWM_COUNTS could overflow a uint16_t
		#if MOTOR_PWM_COUNTS > (65535 - (128 * ACCEL_SCALE))
			// update the PWM of the motor driver with the new accel value
			int32_t new_pwm = (int32_t)motor_control_pwm + (int32_t)new_accel_value;
			if (new_pwm < 0)
				new_pwm = 0;
			if (new_pwm >= 65536)
				new_pwm = 65535;
			set_motor_raw_PWM((uint16_t)new_pwm);
		#else
			// update the PWM of the motor driver with the new accel value
			if (new_accel_value < 0 && motor_control_pwm < -new_accel_value)  // set PWM to 0 if adding the current PWM to the accel value would give a negative number
				set_motor_raw_PWM(0);  // (THIS CODE MUST BE UPDATED WHEN WE START DRIVING THE MOTOR BACKWARDS AND FORWARDS)
			else
				set_motor_raw_PWM(motor_control_pwm + new_accel_value);
		#endif
		
		accel_buffer_start++;  // move the start of the buffer up one space, wrapping around if we've hit the end of the array
		if (accel_buffer_start >= accel_ring_buffer + ACCEL_BUFFER_ELEMENTS)
			accel_buffer_end = accel_ring_buffer;
	}
	else
	{  // buffer underrun condition
		// we've run out of valid acceleration data, reset to the unaltered PWM from the motor control loop
		set_motor_raw_PWM(motor_control_pwm);
		
		// Alternatively, we could continue using the most recent accel value.
		// TODO: Either way, we should do some smoothing once new data arrives, to prevent the discontinuity from being felt too much
	}
	
	// reset the counter back to 0
	TC4H = 0;
	TCNT4 = 0;
}

