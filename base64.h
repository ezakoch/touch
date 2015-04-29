#ifndef BASE64_H_
#define BASE64_H_

#define MAX_BASE64_LEN         512
#define MAX_BASE64_INPUT_BYTES (MAX_BASE64_LEN * 3 / 4)

#define BASE64_ENCODING
#define BASE64_DECODING

#ifdef BASE64_ENCODING
	extern char base64_encoded_output[MAX_BASE64_LEN + 1];

	// encode to base64_output
	void base64_encode (const uint8_t *buffer, const uint16_t len);

	// encode and send via USB serial interface
	void base64_begin_encode_stream (void);
	void base64_encode_stream (const uint8_t *buffer, const uint16_t len);
	void base64_end_encode_stream (void);
#endif

#ifdef BASE64_DECODING
	// decode from base64
	void base64_decode (const char *buffer, const uin16_t len);
	
	// decode from a string of characters, output buffer pointer gets incremented automatically
	void base64_begin_decode_stream (void);
	void base64_decode_stream (uint8_t **output, const char *input, const uint16_t len);
	void base64_end_decode_stream (uint8_t **output);
#endif

#endif /* BASE64_H_ */