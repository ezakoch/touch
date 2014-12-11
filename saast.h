// -----------------------------------------------------------------------------
// SAAST-specific M2 library header
// version: 1.7
// date: July 4, 2012
// author: J. Fiene
// -----------------------------------------------------------------------------

#ifndef saast__
#define saast__

// -----------------------------------------------------------------------------
// General AVR libraries that we'll need at times:
// -----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Useful pre-compile constants
// -----------------------------------------------------------------------------

#define TRUE	1
#define FALSE	0

#define OFF		0
#define ON		1
#define TOGGLE	2

#define F0	0
#define F1	1
#define F4	4
#define F5	5
#define F6	6
#define F7	7
#define D4	8
#define D6	9
#define D7	10
#define B4	11
#define B5	12
#define B6	13

#define B0	14
#define B1	15
#define B2	16
#define B3	17
#define B7	18
#define D0	19
#define D1	20
#define D2	21
#define D3	22
#define	D5	23	
#define C6	24
#define C7	25
#define E6	26

// -----------------------------------------------------------------------------
// Bit manipulation and validation:
// -----------------------------------------------------------------------------

#define set(reg,bit)		reg |= (1<<(bit))
#define clear(reg,bit)		reg &= ~(1<<(bit))
#define toggle(reg,bit)		reg ^= (1<<(bit))

#define check(reg,bit)		(bool)(reg & (1<<(bit)))
// As of version 2.0, this will return either 1 (TRUE) or 0 (FALSE)


// -----------------------------------------------------------------------------
// Disable JTAG to access F4-F7:
// -----------------------------------------------------------------------------

#define m_disableJTAG()		MCUCR = (1 << JTD); MCUCR = (1 << JTD)
// Setting the JTD bit in MCUCR twice within four clock cycles will allow user
// access to F4-F7 as normal port pins. Note that using |= is too slow for this
// operation to work correctly, so we are setting the entire register 
// (forutnately, all other bits in MCUCR are 0 anyway).


// -----------------------------------------------------------------------------
// Set the system clock:
// -----------------------------------------------------------------------------

#define m_clockdivide(val)	CLKPR = (1<<CLKPCE); CLKPR=val
// "val" must be an integer from 0 to 8
// this will divide the 16MHz system clock by 2^val:
// 0 = 16 MHz
// 1 = 8 MHz
// 2 = 4 MHz
// 3 = 2 MHz
// 4 = 1 MHz
// 5 = 500 kHz
// 6 = 250 kHz
// 7 = 125 kHz
// 8 = 62.5 kHz


// -----------------------------------------------------------------------------
// Wait for a specified number of milliseconds:
// -----------------------------------------------------------------------------

#define m_wait(val)			_delay_ms(val)
// "val" must be an integer from 1 to 65535
// this function assumes a 16MHz clock


// -----------------------------------------------------------------------------
// Quick initialization:
// -----------------------------------------------------------------------------
#define m_init()	m_disableJTAG(); m_clockdivide(0)


// -----------------------------------------------------------------------------
// Onboard LEDs
// -----------------------------------------------------------------------------

void m_green(char state);
// FUNCTIONALITY:
// turn on, turn off, or toggle the green onboard LED
// note that this takes control of pin E2
//
// TAKES:
// state : OFF (0), ON (1), or TOGGLE (2)
//
// RETURNS:
// nothing

void m_red(char state);
// FUNCTIONALITY:
// turn on, turn off, or toggle the green onboard LED
// note that this takes control of pin E6
//
// TAKES:
// state : OFF (0), ON (1), or TOGGLE (2)
//
// RETURNS:
// nothing


// -----------------------------------------------------------------------------
// Digital input/output:
// -----------------------------------------------------------------------------

bool m_gpio_in(char channel);
// FUNCTIONALITY:
// read one of the digital inputs
//
// TAKES:
// channel : the pin label (B0...F7)
//
// RETURNS:
// TRUE (1) or FALSE (0)

void m_gpio_out(char channel, bool state);
// FUNCTIONALITY:
// set a digital output
//
// TAKES:
// channel : the pin label (B0...F7)
//
// RETURNS:
// nothing


// -----------------------------------------------------------------------------
// ADC conversion
// -----------------------------------------------------------------------------

int m_adc(char channel);
// FUNCTIONALITY:
// read one of the onboard analog inputs
//
// TAKES:
// channel : 0, 1, 4-13
//
// RETURNS:
// an int between 0 (corresponds to a 0V input) and 1023 (corresponds to a 5V input)
// -1 for an invalid channel

// -----------------------------------------------------------------------------
// mBUS initialization
// -----------------------------------------------------------------------------

void m_bus_init(void);
// FUNCTIONALITY:
// initialize the M2 data bus, which uses pins D0-D2 and is available through the
// 5-pin end header.  When new data is available from a slave, the INT2_vect interrupt
// will be triggered, and you must act accordingly!
//
// TAKES:
// nothing
//
// RETURNS:
// nothing



// -----------------------------------------------------------------------------
// PWM Outputs:
// -----------------------------------------------------------------------------

// There are six PWM outputs on the M2:
//
// Timer | Channel |  Pin  | Resolution | Min Frequency (Hz)
// ----------------------------------------------------------
//   0	 |    1    |  D0   |     8      |       61.3
// ----------------------------------------------------------
//   1   |    1    |  B6   |    16      |       0.24
//   1   |    2    |  B7   |    16      |       0.24
// ----------------------------------------------------------
//   3   |    1    |  C6   |    16      |       0.24
// ----------------------------------------------------------
//   4   |    1    |  C7   |    10      |       1.00
//   4   |    2    |  B5   |    10      |       1.00
//   4   |    3    |  D7   |    10      |       1.00
// ----------------------------------------------------------


char m_pwm_timer(char timer, float freq);
// FUNCTIONALITY:
// configure the PWM timer to a specified frequency
// note: this function assumes a 16MHz clock!
//
// TAKES:
// timer : 0, 1, 3, or 4
// freq : in Hertz, see table above for minimums
//
// RETURNS:
// 0 if there is an error (invalid timer)
// 1 if successful

char m_pwm_output(char timer, char channel, char state);
// FUNCTIONALITY:
// connect or disconnect the output
//
// TAKES:
// timer : 0, 1, 3, or 4
// channel: see above for valid channels for each timer
// state : ON (1) or OFF (0)
// 
// RETURNS:
// 0 if there is an error (invalid timer or channel)
// 1 if successful

char m_pwm_duty(char timer, char channel, float duty);
// FUNCTIONALITY:
// set the PWM duty cycle for a specified channel
//
// TAKES:
// timer : 0, 1, 3, or 4
// channel: see above for valid channels for each timer
// duty : floating point value between 0.0 and 100.0
// 
// RETURNS:
// 0 if there is an error (invalid timer or channel, duty out of range)
// 1 if successful


// -----------------------------------------------------------------------------
// mX Expansion Module
// -----------------------------------------------------------------------------

// Faulhabers
#define CW	0
#define CCW	1

// GPIO Pins
#define A	F0
#define B	F1
#define C	F4
#define D	F5
#define E	F6
#define F	F7
#define G	D7
#define H	C7
#define I	C6
#define J	B5
#define K	B4

void mx_init(void);
// FUNCTIONALITY:
// configure many of the mX-specific subsystems


void mx_servo_init(char channel);
// FUNCTIONALITY:
// initialize a connected servo motor
//
// TAKES:
// channel : G, H, I, or J
//
// RETURNS:
// nothing

void mx_servo(char channel, float duty);
// FUNCTIONALITY:
// command a connected servo motor
//
// TAKES:
// channel : G, H, I, or J
// duty : a floating-point number between 0.0 and 100.0
//
// RETURNS:
// nothing

void mx_motor(char channel, char direction, float duty);
// FUNCTIONALITY:
// set the direction and speed of a connected DC motor
//
// TAKES:
// channel : 1 or 2
// direction : CW (0) or CCW (1)
// duty : a floating-point number between 0.0 and 100.0
//
// RETURNS:
// nothing

void mx_encoder_zero(char channel);
// FUNCTIONALITY:
// zero one of the Faulhaber encoders
//
// TAKES:
// channel : 1 or 2
//
// RETURNS:
// nothing

long mx_encoder(char channel);
// FUNCTIONALITY:
// read one of the Faulhaber encoders
//
// TAKES:
// channel : 1 or 2
//
// RETURNS:
// signed long corresponding to the number of ticks since last zero

// -----------------------------------------------------------------------------
// All content below here was duplicated from standard module header files
// as of July 4, 2012.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// mBUS peripheral communications:
// -----------------------------------------------------------------------------

void m_bus_init(void);
// FUNCTIONALITY:
// initialize the M2 data bus, which uses pins D0-D2 and is available through the
// 5-pin end header.  When new data is available from a slave, the INT2_vect interrupt
// will be triggered, and you must act accordingly!
//
// TAKES:
// nothing
//
// RETURNS:
// nothing

unsigned char m_read_register(unsigned char addr, unsigned char reg);
// FUNCTIONALITY:
// sends [START + W] [register address] [STOP] [START + R]
//
// TAKES:
// addr - I2C slave address
// reg - register address
//
// RETURNS:
// register value

unsigned char m_write_register(unsigned char addr, unsigned char reg, unsigned char value);
// FUNCTIONALITY:
// sends [START + W] [register address] [value] [STOP]
//
// TAKES:
// addr - I2C slave address
// reg - register address
// value - value to place in register
//
// RETURNS:
// 1 - success
// 0 - communication error

// -----------------------------------------------------------------------------
// mIMU sensor functions:
// -----------------------------------------------------------------------------

unsigned char m_imu_init(unsigned char accel_scale, unsigned char gyro_scale);
// FUNCTIONALITY
// initialize the 9-DOF mIMU board
//
// TAKES:
// accel_scale: 0 = +/-2G, 1 = +/-4G, 2 = +/-8G, 3 = +/-16G
// gyro_scale: 0 = +/-250d/s, 1 = +/-500d/s, 2 = +/-1000d/s, 3 = +/-2000d/s
//
// RETURNS:
// 1 : success
// 0 : communication error

unsigned char m_imu_raw(int* raw_data);
// FUNCTIONALITY
// places the raw sensor data into a pre-allocated 9-element int array as
// [a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z]
//
// TAKES:
// raw_data : (pointer to the first element of a nine-element int array)
//
// RETURNS:
// 1 : success
// 0 : communication error

unsigned char m_imu_accel(int* raw_data);
// FUNCTIONALITY
// places the raw accelerometer data into a pre-allocated 3-element int array as
// [a_x, a_y, a_z]
//
// TAKES:
// raw_data : (pointer to the first element of a three-element int array)
//
// RETURNS:
// 1 : success
// 0 : communication error

unsigned char m_imu_gyro(int* raw_data);
// FUNCTIONALITY
// places the raw gyroscope data into a pre-allocated 3-element int array as
// [g_x, g_y, g_z]
//
// TAKES:
// raw_data : (pointer to the first element of a three-element int array)
//
// RETURNS:
// 1 : success
// 0 : communication error

unsigned char m_imu_mag(int* raw_data);
// FUNCTIONALITY
// places the raw magnetometer data into a pre-allocated 3-element int array as
// [m_x, m_y, m_z]
//
// TAKES:
// raw_data : (pointer to the first element of a three-element int array)
//
// RETURNS:
// 1 : success
// 0 : communication error



// -----------------------------------------------------------------------------
// RF Communications
// -----------------------------------------------------------------------------

char m_rf_open(char channel, char RXaddress, char packet_length);
// FUNCTIONALITY:
// configure the RF communications channel and place in RX mode, 
// which will take INT2 low when data is available
//
// TAKES:
// channel : 1 to 32 (must match between sender/receiver)
// RXaddress : (the module's unique RX address)
// packet_length : 1 to 32 (must match between sender/receiver)
// 
// RETURNS:
// 1 : module acknowledged setup
// 0 : something went wrong


char m_rf_read(char* buffer, char packet_length);
// FUNCTIONALITY
// get the message from the module's receive buffer
//
// TAKES:
// buffer : (pointer to the first element of a buffer that is packet_length long)
// packet_length : 1 to 32 (must match how it was set up with m_rf_open!)
//
// RETURNS:
// 1 : something was read
// 0 : not connected, nothing to read, or buffer length mismatch


char m_rf_send(char TXaddress, char* buffer, char packet_length);
// FUNCTIONALITY:
// take the transmitter out of receive mode
// send a message to a specified receiver
// wait for an ACK packet
// resend up to three times
// return the transmitter to receive mode

// TAKES:
// buffer : (pointer to the first element of a buffer that is packet_length long)
// TXaddress : (the receiving module's unique RX address)
//
// RETURNS:
// 1 : indicates successful transmission to the mRF module (does NOT indicate receipt by receiver)
// 0 : something went wrong


// -----------------------------------------------------------------------------
// USB Communications
// -----------------------------------------------------------------------------

// INITIALIZATION: -------------------------------------------------------------

void m_usb_init(void);	
// initialize the USB subsystem

char m_usb_isconnected(void);
// confirm that the USB port is connected to a PC

// RECEIVE: -------------------------------------------------------------------

unsigned char m_usb_rx_available(void);		   		   
// returns the number of bytes (up to 255) waiting in the receive FIFO buffer

char m_usb_rx_char(void);		   			   
// retrieve a oldest byte from the receive FIFO buffer (-1 if timeout/error)

void m_usb_rx_flush(void);		   			   
// discard all data in the receive buffer

// TRANSMIT: ------------------------------------------------------------------

char m_usb_tx_char(unsigned char c);                 
// add a single 8-bit unsigned char to the transmit buffer, return -1 if error

void m_usb_tx_hexchar(unsigned char i);			   
// add an unsigned char to the transmit buffer, send as two hex-value characters

void m_usb_tx_hex(unsigned int i);			   
// add an unsigned int to the transmit buffer, send as four hex-value characters

void m_usb_tx_int(int i);
// add a signed int to the transmit buffer, send as a sign character then 5 decimal-value characters

void m_usb_tx_uint(unsigned int i);
// add an unsigned int to the transmit buffer, send as 5 decimal-value characters

void m_usb_tx_long(long i);
// add a signed long to the transmit buffer, send as a sign character then 5 decimal-value characters

void m_usb_tx_ulong(unsigned long i);
// add an unsigned long to the transmit buffer, send as 5 decimal-value characters

#define m_usb_tx_string(s) print_P(PSTR(s))
// add a string to the transmit buffer

void print_P(const char *s);


// -----------------------------------------------------------------------------
// mWII peripheral functions:
// -----------------------------------------------------------------------------

char m_wii_open(void);
// FUNCTIONALITY:
// configure the mWii Pixart sensor.
// (note:  the mWii does not generate interrupts)
//
// TAKES:
// nothing
//
// RETURNS:
// 1 : success
// 0 : failure


char m_wii_read(unsigned int* blobs);
// FUNCTIONALITY:
// read blob data
//
// TAKES:
// blobs : pointer to a 12-element unsigned int buffer
//
// RETURNS:
// 1 : success
// 0 : failure




#endif

