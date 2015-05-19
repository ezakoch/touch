/*---------------------------------------------------------------------------
TOUCH Actuator Controller on M2 (Atmega32u4)
cd Dropbox/TOUCH/m2
date: May 2015
author: Eza Koch and Kent deVillafranca
-----------------------------------------------------------------------------
Library Inclusions
---------------------------------------------------------------------------*/

//#include "saast.h" //SAAST Library
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "timer_ticks.h"
#include "pc_communication.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>

// -----------------------------------------------------------------------------
// Definitions
// -----------------------------------------------------------------------------
#define TIMER 1 //Timers 0,1,3,4 availabl1
#define FREQ 40000.0 //floating point in Hz, aim for 40kHz
#define CHANNEL 1 //Pin D0 for motor PWM
#define MAXRPS 83.3 //Maxon 144325 assembly with gearhead 5000.0/60.0
#define TICKS 500.0 //counts per turn 110514 encoder

// -----------------------------------------------------------------------------
// Debugging
// -----------------------------------------------------------------------------
#define DEBUG
// #define DEBUG_TEMP

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
volatile int count=0; //encoder ticks

// -----------------------------------------------------------------------------
// Initialize Helper Functions
// -----------------------------------------------------------------------------
void driveMotor(int dir, float rps_desired);
int adc(void);
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
	float pot_state;
	float y_desired;
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
	// To end the session, press Ctrl-A then Ctrl-\
	// -----------------------------------------------------------------------------

	m_usb_init();
	init_timer(); //do i need this?

	// -----------------------------------------------------------------------------
	// Setup PWM - Timer 1 Pin B6
	// -----------------------------------------------------------------------------
	//m_pwm_timer(TIMER, FREQ);
	set(TCCR1B,WGM13);	// up to OCR1A (max 65535)
	set(TCCR1B,WGM12);	// ^
	set(TCCR1A,WGM11);	// ^
	set(TCCR1A,WGM10);	// ^
	TCNT1 = 0;// reset counter
	OCR1A = (16000000/FREQ);//>250Hz		
	set(TCCR1B,CS10);	
	clear(TCCR1B,CS11);
	clear(TCCR1B,CS12);
 	//m_pwm_output(TIMER, CHANNEL, ON);
	set(DDRB,6);		// take over the output
	set(TCCR1A,COM1B1);	// clear on match with OCR1B

	// -----------------------------------------------------------------------------
	// Setup ADC and GPIO
	// -----------------------------------------------------------------------------

	set(ADMUX,REFS0);		// voltage ref to Vcc
	clear(ADMUX,REFS1);		// ^
//	set(ADCSRB,ADHSM);		// enable high-speed mode
	set(ADCSRA,ADPS0);		// prescaler to /128 = 125kHz
	set(ADCSRA,ADPS1);		// ^ 
	set(ADCSRA,ADPS2);		// ^

	clear(ADCSRB,MUX5); // configure the specified channel: ADC6 pin F6
	set(ADMUX,MUX2);    // ^
	set(ADMUX,MUX1);   // ^
	clear(ADMUX,MUX0);  // ^

	set(DIDR0,ADC7D);// Disable digital input circuitry for bit 7
	//set(ADATE,ADCSRA);//triggering
	//set(DIDR0,CHANNEL); // ^ ??
	ADMUX = (ADMUX & 0xE0) | CHANNEL;

	clear(DDRD,4); //Digital In: D4

	// -----------------------------------------------------------------------------
	// SETUP INTERRUPTS: PIN CHANGE PCINT0 pin D0
	// -----------------------------------------------------------------------------
 	sei();//enable interrupts
 	set(PCICR,PCIE0); //pin-change, cleared by default
 	set(PCMSK0,PCINT0); //remove mask for corresponding interrupt
	
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
		
		// run the control loop roughly once every 20000us=20ms
		if (us_elapsed() - control_loop_last_run_time > 20000ul)
		{
			//ADC user input from potentiometer
			pot_state = adc(); 
			#ifdef DEBUG
				m_usb_tx_string("pot_state: ");
				m_usb_tx_int((int)(pot_state));
				m_usb_tx_string("\r\n");
			#endif
			y_desired=(float)MAXRPS*(pot_state/1023.0); //[rps]

			//Digital GPIO: Swithcing Motor ON/OFF
			if (check(PIND,4)){disableMotor();m_red(ON);m_green(OFF);}
			else{driveMotor(dir, y_desired);m_green(ON);m_red(OFF);}
			
			control_loop_last_run_time = us_elapsed();
		}

		if (received_pc_message())
			process_pc_message();
	}
}

// -----------------------------------------------------------------------------
//  PID CONTORLLER AND MOTOR DRIVER
// -----------------------------------------------------------------------------
void driveMotor (int dir, float rps_desired){
	static uint64_t prev_time = 0;
	static float prev_error = 0;
	static float error_sum = 0;
	
	const uint64_t current_time = us_elapsed();
	const float dt = (float)(current_time - prev_time);
	const float Kp = 5.0;
	const float Kd = 0.0;
	const float Ki = 0.0;

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
	
 	//m_pwm_duty(TIMER, CHANNEL, duty_cycle); //Timer 1, Channel 1: pin B6 
	set(DDRB,6);		// take over the output
	set(TCCR1A,COM1B1);	// clear on match with OCR1B
 	OCR1B = (unsigned int)((float)OCR1A*(duty_cycle/100.0)); //Timer 1, Channel 1: pin B6 
	
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
//  ADC CONVERSIONS
// -----------------------------------------------------------------------------
int adc(void){
	set(ADCSRA,ADEN);			// enable ADC
	set(ADCSRA,ADSC);			// start conversion
	while(!check(ADCSRA,ADIF)){}; // wait for conversion to finish
	set(ADCSRA,ADIF);			// reset the flag
	return ADC;					// pass back the result
}
// -----------------------------------------------------------------------------
//  DISABLE MOTORS
// -----------------------------------------------------------------------------
void disableMotor(void){
	set(DDRB,6);		// take over the output
	set(TCCR1A,COM1B1);	// clear on match with OCR1B
}
// -----------------------------------------------------------------------------
// INTERRUPTS
// -----------------------------------------------------------------------------
ISR(PCINT0_vect){count++;} //Timer 0, pin B0


