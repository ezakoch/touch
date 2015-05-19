#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "m_general.h"

// Below-finger pressure sensor: ADC6
#define B_FSR_DDR  DDRF
#define B_FSR_PORT PORTF
#define B_FSR_NUM  6
#define B_FSR_ADC  6
#define B_FSR_DREG DIDR0
#define B_FSR_DPIN ADC6D

#define NUM_ADCS 1

typedef struct
{
	uint16_t pot;
} adc_values_struct;

extern adc_values_struct adc_values;

bool init_adc (void);

#endif