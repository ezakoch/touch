#include "adc.h"

adc_values_struct adc_values;

const uint8_t adcIndex[NUM_ADCS] = {B_FSR_ADC};

uint16_t *adcPtrs[NUM_ADCS] = {&adc_values.pot};


static uint8_t get_prescaler_value (void)
{
	const uint16_t min_scale = F_CPU / 200000ull;
	const uint16_t max_scale = F_CPU / 50000ul;
	
	// valid scales: 2, 4, 8, 16, 32, 64, 128
	for (uint8_t i = 1; i < 8; i++)
	{
		if ((1 << i) >= min_scale && (1 << i) <= max_scale)
			return i;
	}
	
	return 0;
}

bool init_adc (void)
{
	// set the ADC pins as input, with no pull-up resistor
	clear (B_FSR_DDR,  B_FSR_NUM);
	clear (B_FSR_PORT, B_FSR_NUM);
	
	// disable digital inputs on the ADC pins
	set (B_FSR_DREG, B_FSR_DPIN);
	
	// set the ADC to use AVCC, with external AREF capacitor, and start at our first ADC value
	const uint8_t mux_bits = adcIndex[0];
	
	if (mux_bits == 0xff)
	{
		// internal temperature sensor
		set (ADCSRB, MUX5);
		ADMUX = (1 << REFS0) | (1 << REFS1) | 0b00000111;
	}
	else if (mux_bits > 7)
	{
		set (ADCSRB, MUX5);
		ADMUX = (1 << REFS0) | ((mux_bits - 8) & 0b00000111);
	}
	else
	{
		clear (ADCSRB, MUX5);
		ADMUX = (1 << REFS0) | (mux_bits & 0b00000111);
	}
	
	// get the prescaler bits for a 50kHz-200kHz ADC clock
	const uint8_t scale_factor_bits = get_prescaler_value() & 0b00000111;
	
	if (scale_factor_bits == 0)
		return false;
	
	// enable the ADC and its interrupt, set the prescaler, and begin the first conversions
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIE) | scale_factor_bits;
	
	return true;
}

ISR (ADC_vect)
{
	static uint8_t adc_index = 0;
	
#ifdef AVR_INTERNAL_TEMP_SENSOR
	static bool second_chip_therm_reading = false;
	
	if (adcIndex[adc_index] == 0xff && !second_chip_therm_reading)  // the temperature sensor requires two successive readings
	{
		second_chip_therm_reading = true;
	}
	else
	{
		second_chip_therm_reading = false;
#endif
		*adcPtrs[adc_index] = ADC & 0x3ff;  // read ADC and clip to 10-bit resolution
		
		adc_index++;
	
		if (adc_index >= NUM_ADCS)
			adc_index = 0;
		
		// set the mux bits to use the next ADC input in the series
		const uint8_t mux_bits = adcIndex[adc_index];
		
#ifdef AVR_INTERNAL_TEMP_SENSOR
		if (mux_bits == 0xff)
		{
			// internal temperature sensor
			set (ADCSRB, MUX5);
			ADMUX = (1 << REFS0) | (1 << REFS1) | 0b00000111;
		} else
#endif

#ifdef ADCS_ABOVE_7
		if (mux_bits > 7)
		{
			set (ADCSRB, MUX5);
			ADMUX = (1 << REFS0) | ((mux_bits - 8) & 0xff);
		}
		else
#endif
		{
			clear (ADCSRB, MUX5);
			ADMUX = (1 << REFS0) | (mux_bits  & 0xff);
		}
#ifdef AVR_INTERNAL_TEMP_SENSOR
	}
#endif
	
	// start the next conversion
	ADCSRA |= (1 << ADSC);
}