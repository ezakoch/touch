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
void driveMotor(float duty_cycle, int dir);
// void readPot(int CHANNEL);
// void pid(void);


//MAIN FUNCTION
int main(void){
	m_disableJTAG();
	int state=0;
	int dir=0;

	m_usb_init();

	//RESET
	m_red(OFF)
	m_green(OFF)

	m_pwm_timer(TIMER, FREQ);
 	m_pwm_output(TIMER, CHANNEL, ON);


	while (1){

		//Edit PWM duty cycle
		float pot_state = m_adc(F6); 
		
		float duty_cycle = 0.0;
		if (pot_state<512){ //go left
			dir=0; 
			duty_cycle=(abs(512-pot_state)*2)/10.230;
			}
			else{          //go right
				dir=1;
				duty_cycle=((pot_state-512)*2)/10.230;
			} 
		//duty_cycle=pot_state/1023.0;

		m_usb_tx_string("duty_cyclex100: ");
		m_usb_tx_int((int)(duty_cycle*100));
		m_usb_tx_string("\r\n");

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
				dir=1;
				driveMotor(0,dir); //stop
				break;
				//move finger down to a position (integration of velocity control)
			case 1:
				m_red(OFF);
				m_green(ON);
				dir=0;
				driveMotor(duty_cycle,dir);//command new DC
				break;
				//move finger up to a position (integration of velocity control)
		}
	}
}

void driveMotor(float duty_cycle, int dir){
// 	//PWM instructions
 	
 	m_pwm_duty(TIMER, CHANNEL, duty_cycle);
 }

// void readPot(int CHANNEL){
// 	pot_state=m_adc(CHANNEL);
// }
