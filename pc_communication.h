#ifndef PC_COMMUNICATION_H
#define PC_COMMUNICATION_H

#define MAX_MESSAGE_LENGTH 255

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
	uint64_t timestamp;
	uint8_t num_accel_samples;
	int16_t accel[3][MAX_ACCEL_SAMPLES];  // x, y, z acceleration
	uint16_t temperatures[3];  // chip temp, board temp, ext. temp
	int16_t slip[2];  // x, y
} incoming_pc_data;

#endif

