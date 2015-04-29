#ifndef BASE64_H_
#define BASE64_H_

#define MAX_BASE64_LEN         512
#define MAX_BASE64_INPUT_BYTES (MAX_BASE64_LEN * 3 / 4)

extern char base64_output[MAX_BASE64_LEN + 1];

// encode to base64_output
void base64_encode (const uint8_t *buffer, const uint16_t len);

// encode and send via USB serial interface
void base64_begin_stream (void);
void base64_stream (const uint8_t *buffer, const uint16_t len);
void base64_end_stream (void);

#endif /* BASE64_H_ */