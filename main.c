//Sending values using Wireless

//INCLUDES
#include "saast.h" //SAAST Library calls
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>

//DEFINE CONSTANTS
#define TIMER 0 //Timers 0,1,3,4 available
#define FREQ 100.0 //floating point in Hz, aim for 20kHz
#define CHANNEL 1  

//INITALIZE
//void driveMotor(float duty_cycle, int dir);
// void readPot(int CHANNEL);
// void pid(void);


//MAIN FUNCTION
int main(void){
	m_disableJTAG();
	int state=0;
	int dir=0;

	// //TEST
	// m_pwm_timer(TIMER, FREQ);
	// m_pwm_output(TIMER, CHANNEL, ON);
	// m_pwm_duty(TIMER, CHANNEL, 0.5);

	//TEST on TEST
	m_pwm_timer(0, 100);
	m_pwm_output(0, 1, ON);
	m_pwm_duty(0, 1, 0.5);

	//RESET
	m_red(OFF)
	m_green(OFF)


	while (1){

		//BUTTON PRESS
		if (m_gpio_in(D4)==ON)
		{
				state=1;
		}
			else
		{
				state=0;
		}

		//ACTUATE STEPPER FOR FINGER
		switch(state){
			case 0:
				m_red(ON);
				m_green(OFF);
				m_wait(50);
				dir=1;
				break;
				//move finger down to a position (integration of velocity control)
			case 1:
				m_red(OFF);
				m_green(ON);
				m_wait(50);
				dir=0;
				break;
				//move finger up to a position (integration of velocity control)
		}

		// //Edit PWM duty cycle
		// float pot_state=m_adc(F6); 
		// float duty_cycle;
		// if (pot_state<512){ //go left
		// 	dir=0; 
		// 	duty_cycle=(abs(512-pot_state)*2)/1023.0;
		// 	}
		// 	else{          //go right
		// 		dir=1;
		// 		duty_cycle=((pot_state-512)*2)/1023.0;
		// 	} 
		// duty_cycle=pot_state/1023.0;

		// //Command duty cycle to motors
		// driveMotor(duty_cycle,dir);
	}
}

// void driveMotor(float duty_cycle, int dir){
// // 	//PWM instructions
//  	m_pwm_timer(TIMER, FREQ);
//  	m_pwm_output(TIMER, CHANNEL, ON);
//  	m_pwm_duty(TIMER, CHANNEL, duty_cycle);
//  }

// void readPot(int CHANNEL){
// 	pot_state=m_adc(CHANNEL);
// }
