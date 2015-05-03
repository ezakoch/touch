#ifndef BASE64_H_
#define BASE64_H_

#include <stdbool.h>

// optional: comment out any functionality you don't need
//#define BASE64_ENCODING
#define BASE64_DECODING
//#define BASE64_STATIC
#define BASE64_STREAMING


#ifdef BASE64_STATIC
	#define MAX_BASE64_LEN   512
	#define MAX_BASE64_BYTES (MAX_BASE64_LEN * 3 / 4)
#endif

#ifdef BASE64_ENCODING
	#ifdef BASE64_STATIC
		extern char base64_encoded_output[MAX_BASE64_LEN + 1];
		// encode to base64_output
		void base64_encode (const uint8_t *buffer, const uint16_t len);
	#endif
	
	#ifdef BASE64_STREAMING
		// encode and send via USB serial interface
		void base64_begin_encode_stream (void);
		void base64_encode_stream (const uint8_t *buffer, const uint16_t len);
		void base64_end_encode_stream (void);
	#endif
#endif

#ifdef BASE64_DECODING
	// note: these functions treat any invalid base64 character as an end-of-string indicator
	#ifdef BASE64_STATIC
		extern uint8_t base64_decoded_output[MAX_BASE64_BYTES];
		// decode from base64
		void base64_decode (const char *base64str, uint16_t *len);
	#endif
	
	#ifdef BASE64_STREAMING
		// decode from a string of characters, output buffer pointer gets incremented automatically
		void base64_begin_decode_stream (void);
		bool base64_decode_stream (uint8_t **output, const char input);  // returns false when it reaches a terminating =
	#endif
#endif

#endif /* BASE64_H_ */