#ifndef PC_COMMUNICATION_H
#define PC_COMMUNICATION_H

#include <stdbool.h>

#define MAX_MESSAGE_LENGTH 512

#ifdef ENABLE_PC_COMMUNICATION_TIMEOUT
#define MESSAGE_TIMEOUT_US 2500
#endif

bool received_pc_message (void);
void process_pc_message (void);

#define ACCEL_DATA_HZ         800ul
#define MAX_DATA_COVERAGE_MS  100ul
#define MAX_ACCEL_SAMPLES     ((ACCEL_DATA_HZ * MAX_DATA_COVERAGE_MS) / 1000)

typedef struct
{
	uint64_t timestamp_us;  // microseconds since the sensor was powered on
	uint8_t num_accel_samples;
	uint8_t accel[MAX_ACCEL_SAMPLES];  // acceleration, scaling should be done on the PC side so that this can be fed directly into a DAC
	uint8_t temperatures[3];  // chip temp, board temp, ext. temp, where the temperatures should be in a range from 0=32F to 255=150F
	int16_t slip;  // x-axis speed
	uint8_t softness;  // some 0-255 measure of "softness", to be determined on the PC side by comparing finger FSR readings against the surface FSR
} pc_data;

bool new_pc_data (void);
pc_data *get_pc_data (void);

#endif