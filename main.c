//Sending values using Wireless
// cd Dropbox/TOUCH/m2
// Eza Koch

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
#define FREQ 40000.0 //floating point in Hz, aim for 20kHz
#define CHANNEL 1 //Pin D0 for motor PWM
#define MAXRPS 83.3 //Maxon 144325 assembly with gearhead 5000.0/60.0
#define TICKS 16.0 //per revolution
#define FUDGE 4

//SWITCH DEBUGGING
#define DEBUG
#define DEBUG_TEMP

//GLOBAL VARS
volatile int count=0; //encoder ticks

//INITALIZE
void driveMotor(int dir, float y_desired);

//MAIN FUNCTION
int main(void){
	//Vars
	m_clockdivide(0);
	m_disableJTAG();
	int state=0;
	int dir=0;
	float pot_state;
	float y_desired;
	float temp;
	float temp_diff;
	float temp_desired = 600.0; //need a separate sensor to check the surface
	float TECout;
	float error;
	float Kp_temp = 2.0;
	float duty_cycle = 0.0;
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

	while (TRUE){

		//Edit PWM duty cycle
		pot_state = m_adc(F6)/1023.0; //percentage 

		m_usb_tx_string("pot_state: ");
		m_usb_tx_int((int)(pot_state*100.0));
		m_usb_tx_string("\r\n");

		y_desired=(float) MAXRPS * pot_state; //rps
		//y_desired = MAXRPM * (pot_state/1023.0) * TICKS * (60.0); //ticks per second 

		//USB Communications
		//Find the serial object: ls /dev/tty.*
		//Start the session: screen /dev/tty.usbmodem411
		/*To end the session: press Ctrl-A then Ctrl-\ */

		//ACCELERATIONS	
		// accel = m_adc(F1);
		// duty_cycle = duty_cycle + (accel/scaling);

		// m_usb_tx_string("ACCELERATIONS: ");
		// m_usb_tx_int((int)(accel));
		// m_usb_tx_string("\r\n");

		//BUTTON PRESS
		if (m_gpio_in(D4)==ON){state=1;}
		else{state=0;}

		//ACTUATE STEPPER FOR FINGER - Motor ON/OFF
		switch(state){
			case 0:
				m_red(ON);
				m_green(OFF);
				dir=1;
				driveMotor(dir, 0.0); //stop
				break;
				//move finger down to a position (integration of velocity control)
			case 1:
				m_red(OFF);
				m_green(ON);
				dir=0;
				driveMotor(dir, y_desired);//command new DC
				break;
				//move finger up to a position (integration of velocity control)
		}


		// THERMAL ELEMENT USB DEBUG
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

		m_wait(1);//ms

	for (;;)
	{
		static uint64_t control_loop_last_run_time = 0;
		
		// run the control loop roughly once every millisecond
		if (us_elapsed() - control_loop_last_run_time > 1000ul)
		{
			//Edit PWM duty cycle
			pot_state = m_adc(F6); 

			y_desired=(float)MAXRPS*(pot_state/1023.0); //rps

			//BUTTON PRESS
			if (m_gpio_in(D4)==ON){state=1;}
			else{state=0;}

			//Motor ON/OFF
			switch(state){
				case 0:
					m_red(ON);
					m_green(OFF);
					dir=1;
					driveMotor(dir, 0.0); //stop
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
void driveMotor(int dir, float y_desired){
	static uint64_t prev_time = 0;
	
	const uint64_t current_time = us_elapsed();
	const uint64_t dt = current_time - prev_time;
	const float Kp = 1.0;


	#ifdef DEBUG
		m_usb_tx_string("y_desired: ");
		m_usb_tx_int((int)(y_desired));
		m_usb_tx_string("\r\n");
	#endif

	// y_desired=y_desired/MAXRPS * 100.0;//units

	//CONTROL LOOP
	float y_actual=(float) (count/TICKS)/((float)dt/1000000.0);//rps
	y_actual/=FUDGE; //FUDGE FACTOR
	y_desired=y_desired/MAXRPS * 100.0;//percentage
	error=error/MAXRPS*100;

	#ifdef DEBUG
		m_usb_tx_string("error: ");
		m_usb_tx_int((int)(error));
		m_usb_tx_string("\r\n");

		m_usb_tx_string("y_actual: ");
		m_usb_tx_int((int)(y_actual));
		m_usb_tx_string("\r\n");
	#endif

 	//update PWM duty cycle per the controller
 	m_pwm_duty(TIMER, CHANNEL, duty_cycle); //Timer 1, Channel 1: pin B6 

 	float duty_cycle = y_desired + (error * Kp); //expected output + gain
 	if (duty_cycle > 100.0)
 		duty_cycle = 100.0;
 	if (duty_cycle < 0.0)
 		duty_cycle = 0.0;

	#ifdef DEBUG
		m_usb_tx_string("duty_cycle: ");
		m_usb_tx_int((int)(duty_cycle));
		m_usb_tx_string("\r\n");
	#endif

 	clear_timer(); //reset

 	//PWM instructions
 	m_pwm_duty(TIMER, CHANNEL, duty_cycle);

	count = 0;  // reset encoder count
 	prev_time = current_time;  // update the elapsed time, to get dt when this function is run next
}

//INTERRUPTS
ISR(PCINT0_vect){count++;} //Timer 0, pin B0


