// TOUCH Actuator Controller on M2 (Atmega32u4)
// cd Dropbox/TOUCH/m2
// author: Eza Koch and Kent deVillafranca

//INCLUDES
#include "saast.h" //SAAST Library calls
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "timer_ticks.h"
#include "pc_communication.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>

//DEFINE CONSTANTS
#define TIMER 1 //Timers 0,1,3,4 availabl1
#define FREQ 40000.0 //floating point in Hz, aim for 40kHz
#define CHANNEL 1 //Pin D0 for motor PWM
#define MAXRPS 83.3 //Maxon 144325 assembly with gearhead 5000.0/60.0
#define TICKS 500.0 //counts per turn 110514 encoder

//SWITCH DEBUGGING
#define DEBUG
// #define DEBUG_TEMP

//GLOBAL VARS
volatile int count=0; //encoder ticks

//INITALIZE
void driveMotor(int dir, float rps_desired);

//MAIN FUNCTION
int main(void){
	
	//Vars
	m_clockdivide(0); //16MHz
	m_disableJTAG();

	int state = 0;
	int dir = 0;
	float pot_state;
	float y_desired;
	float temp;
	float temp_diff;
	float temp_desired = 600.0; //need a separate sensor to check the surface
	float TECout;
	float Kp_temp = 2.0;
	// float scaling = 1.0;
	// float accel;

	//Initialize USB Communication
	m_usb_init();
	init_timer();

	//RESET
	m_red(OFF);
	m_green(OFF);
	m_pwm_timer(TIMER, FREQ);
 	m_pwm_output(TIMER, CHANNEL, ON);

 	//Reset TEC timer
 	//m_pwm_timer(3, 10000.0);

 	sei();//enable interrupts, start ticks

 	//ENABLE PIN CHANGE INTERRUPTS: PCINT0 - D0
 	set(PCICR,PCIE0); //pin-change, cleared by default
 	set(PCMSK0,PCINT0); //remove mask for corresponding interrupt

	// THERMAL ELEMENT USB DEBUG
	/*
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

	

	for (;;)
	{
		static uint64_t control_loop_last_run_time = 0;
		
		// run the control loop roughly once every 20000us=20ms
		if (us_elapsed() - control_loop_last_run_time > 20000ul)
		{
			//Edit PWM duty cycle
			pot_state = m_adc(F6); 

			y_desired=(float)MAXRPS*(pot_state/1023.0); //[rps]

			//BUTTON PRESS
			if (m_gpio_in(D4)==ON){state=1;}
			else{state=0;}

			//Motor ON/OFF
			switch(state){
				case 0:
					m_red(ON);
					m_green(OFF);
					m_pwm_duty(TIMER, CHANNEL, 0);
					break;
				case 1:
					m_red(OFF);
					m_green(ON);
					dir=0;
					driveMotor(dir, y_desired);//command new DC
					break;
			}
			
			control_loop_last_run_time = us_elapsed();
		}
		
		
		if (received_pc_message())
			process_pc_message();
	}
}

//MOTOR CONTROL HELPER
void driveMotor (int dir, float rps_desired){
	static uint64_t prev_time = 0;
	static float prev_error = 0;
	static float error_sum = 0;
	
	const uint64_t current_time = us_elapsed();
	
	const float dt = (float)(current_time - prev_time);
	
	const float Kp = 5.0;
	const float Kd = 0.0;
	const float Ki = 0.0;

	// PID CONTROL LOOP
	const float revs = (float)count / (float)TICKS;
	const float rps_actual = revs * 1e6 / dt;
	const float error = rps_desired - rps_actual;

	#ifdef DEBUG
		m_usb_tx_string ("Time (ms) = ");
		m_usb_tx_int ((unsigned int)(current_time / 1000));
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
	if (error_sum * Ki > 100.0) //windup
		error_sum = 100.0 / Ki;

	const float error_diff = (error - prev_error) / dt;
	
 	float duty_cycle = (rps_desired) + (error * Kp) + (error_diff * Kd) + (error_sum * Ki);
 	
 	#ifdef DEBUG
 		m_usb_tx_string ("I term: ");
 		m_usb_tx_int ((int)(error_sum * Ki));
		m_usb_tx_string ("\r\nerror_diff: ");
 		m_usb_tx_int ((int)error_diff);
		m_usb_tx_string("\r\nduty_cycle before cap: ");
		m_usb_tx_int((int)(duty_cycle));
		m_usb_tx_string("\r\n===================\r\n");
	#endif

 	if (duty_cycle > 100.0)
		duty_cycle = 100.0;
	if (duty_cycle < 0.0)
		duty_cycle = 0.0;
	
 	//update PWM duty cycle per the controller
 	m_pwm_duty(TIMER, CHANNEL, duty_cycle); //Timer 1, Channel 1: pin B6 
	
	#ifdef DEBUG
		m_usb_tx_string("duty_cycle: ");
		m_usb_tx_int((int)(duty_cycle));
		m_usb_tx_string("\r\n===================\r\n");
	#endif
	
	count = 0;  // reset encoder count
	prev_error = error;  // update the previous error record
 	prev_time = current_time;  // update the elapsed time, to get dt when this function is run next
}

//INTERRUPTS
ISR(PCINT0_vect){count++;} //Timer 0, pin B0


