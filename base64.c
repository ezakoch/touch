// These two includes are here because streaming output was coded for the M2 (http://medesign.seas.upenn.edu/index.php/Guides/MaEvArM)
// For other systems, replace strcpy_P and m_usb_tx_char with strcpy and printf, or whatever is appropriate.
#include "m_general.h"
#include "m_usb.h"

#include "base64.h"

const char base64_err_str[] PROGMEM = "!BASE64!";

const char base64_chars[64] PROGMEM =
	{
		'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
		'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
		'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
		'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
		'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
		'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
		'w', 'x', 'y', 'z', '0', '1', '2', '3',
		'4', '5', '6', '7', '8', '9', '+', '/'
	};

inline char base64_char_from_byte (const uint8_t byte)
{
	return pgm_read_byte (base64_chars + (byte & 0b00111111));
}



#ifdef BASE64_ENCODING

static void encode_triplet (const uint8_t *input, uint8_t *output)
{
	// shove 6 bits of each input into the output bytes
	// input:  abcdefgh ijklmnop qrstuvwx
	// output: 00abcdef 00ghijkl 00mnopqr 00stuvwx
	output[0] = input[0] >> 2;
	output[1] = ((input[0] << 4) & 0b00111111) | (input[1] >> 4);
	output[2] = ((input[1] << 2) & 0b00111111) | (input[2] >> 6);
	output[3] = input[2] & 0b00111111;
}

static void encode_doublet (const uint8_t *input, uint8_t *output)
{
	// input:  abcdefgh ijklmnop
	// output: 00abcdef 00ghijkl 00mnop00 =
	output[0] = input[0] >> 2;
	output[1] = ((input[0] << 4) & 0b00111111) | (input[1] >> 4);
	output[2] = (input[1] << 2) & 0b00111111;
}

static void encode_singlet (const uint8_t *input, uint8_t *output)
{
	// input:  abcdefgh
	// output: 00abcdef 00gh0000 = =
	output[0] = input[0] >> 2;
	output[1] = (input[0] << 4) & 0b00111111;
}

#ifdef BASE64_STATIC

char base64_encoded_output[MAX_BASE64_LEN + 1];

void base64_encode (const uint8_t *input, const uint16_t len)
{
	if (len > MAX_BASE64_BYTES)
	{
		strcpy_P (base64_encoded_output, base64_err_str);
		return;
	}
	
	char *base64_output_ptr = base64_encoded_output;
	
	// 3 bytes of input fit into 4 bytes of output
	const uint16_t len_loops = len / 3;
	for (uint16_t i = 0; i < len_loops; i++)
	{
		uint8_t output[4];
		
		encode_triplet (input, output);
		
		// output values are 0-63
		for (int j = 0; j < 4; j++)
			base64_output_ptr[j] = base64_char_from_byte (output[j]);
		
		base64_output_ptr += 4;
		input += 3;
	}
	
	// if the number of input bytes isn't divisible by 3, we need to pad the end
	const uint8_t remainder = len % 3;
	if (remainder == 1)
	{
		uint8_t output[2];
		
		encode_singlet (input, output);
		
		base64_output_ptr[0] = base64_char_from_byte(output[0]);
		base64_output_ptr[1] = base64_char_from_byte(output[1]);
		base64_output_ptr[2] = '=';
		base64_output_ptr[3] = '=';
	}
	else if (remainder == 2)
	{
		uint8_t output[3];
		
		encode_doublet (input, output);
		
		base64_output_ptr[0] = base64_char_from_byte(output[0]);
		base64_output_ptr[1] = base64_char_from_byte(output[1]);
		base64_output_ptr[2] = base64_char_from_byte(output[2]);
		base64_output_ptr[3] = '=';
	}
	
	base64_output_ptr[4] = '\0';
}
#endif

#ifdef BASE64_STREAMING

static uint8_t buffered_bytes = 0;
static uint8_t bytes[3];

void base64_begin_encode_stream (void)
{
	buffered_bytes = 0;
}

void base64_encode_stream (const uint8_t *input, const uint16_t len)
{
	if (len == 0)
		return;
	
	uint8_t output[4];
	
	for (const uint8_t* offset = input; offset < input + len; offset++)
	{
		bytes[buffered_bytes] = *offset;
		buffered_bytes++;
		
		if (buffered_bytes == 3)
		{
			encode_triplet (bytes, output);
			
			for (int j = 0; j < 4; j++)
				m_usb_tx_char (base64_char_from_byte (output[j]));
			
			buffered_bytes = 0;
		}
	}
}

void base64_end_encode_stream (void)
{
	uint8_t output[3];
	
	if (buffered_bytes == 2)
	{
		encode_doublet (bytes, output);
		m_usb_tx_char (base64_char_from_byte (output[0]));
		m_usb_tx_char (base64_char_from_byte (output[1]));
		m_usb_tx_char (base64_char_from_byte (output[2]));
		m_usb_tx_char ('=');
	}
	else if (buffered_bytes == 1)
	{
		encode_singlet (bytes, output);
		m_usb_tx_char (base64_char_from_byte (output[0]));
		m_usb_tx_char (base64_char_from_byte (output[1]));
		m_usb_tx_char ('=');
		m_usb_tx_char ('=');
	}
	
	buffered_bytes = 0;
}
#endif

#endif


#ifdef BASE64_DECODING

inline bool valid_char (const char c)
{
	return (c >= '/' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '+';
}

static void decode_4char (const char *input, uint8_t *output)
{
	const uint8_t *in = (const uint8_t*)input;  // cast to unsigned to avoid signedness issues when bitshifting
	
	// restore the encoded bytes
	// input:  00abcdef 00ghijkl 00mnopqr 00stuvwx
	// output: abcdefgh ijklmnop qrstuvwx
	output[0] = (in[0] << 2) | (in[1] >> 4);
	output[1] = (in[1] << 4) | (in[2] >> 2);
	output[2] = (in[2] << 6) | in[3];
}

static void decode_3char (const char *input, uint8_t *output)
{
	const uint8_t *in = (const uint8_t*)input;  // cast to unsigned to avoid signedness issues when bitshifting
	
	// input:  00abcdef 00ghijkl 00mnop00 =
	// output: abcdefgh ijklmnop
	output[0] = (in[0] << 2) | (in[1] >> 4);
	output[1] = (in[1] << 4) | (in[2] >> 2);
}

static void decode_2char (const char *input, uint8_t *output)
{
	const uint8_t *in = (const uint8_t*)input;  // cast to unsigned to avoid signedness issues when bitshifting
	
	// input:  00abcdef 00gh0000 = =
	// output: abcdefgh
	output[0] = (in[0] << 2) | (in[1] >> 4);
}

#ifdef BASE64_STATIC
uint8_t base64_decoded_output[MAX_BASE64_BYTES];

void base64_decode (const char *base64str, uint16_t *len)
{
	char buffer[4];
	uint8_t buffer_idx = 0;
	
	*len = 0;
	
	while (*base64str && valid_char(*base64str) && *len < MAX_BASE64_BYTES - 3)
	{
		buffer[buffer_idx++] = *(base64str++);
		
		if (buffer_idx >= 4)
		{
			decode_4char (buffer, &base64_decoded_output[*len]);
			buffer_idx = 0;
			*len += 3;
		}
	}
	
	switch (buffer_idx)
	{
		case 1:  // there should never be only a single character left in the buffer
			base64_decoded_output[*len] = '!';
			(*len)++;
			break;
		case 2:
			decode_2char(buffer, &base64_decoded_output[*len]);
			(*len)++;
			break;
		case 3:
			decode_3char(buffer, &base64_decoded_output[*len]);
			(*len) += 2;
			break;
	}
}
#endif

#ifdef BASE64_STREAMING
static char decode_buffer[4];
static uint8_t decode_buffer_idx = 0;

void base64_begin_decode_stream (void)
{
	decode_buffer_idx = 0;
}

bool base64_decode_stream (uint8_t **output, const char input)
{
	if (!valid_char (input))
	{  // end of the stream
		switch (decode_buffer_idx)
		{
			// there should never be only a single character left in the buffer
			case 2:
			decode_2char(decode_buffer, *output);
			(*output) += 1;
			break;
			case 3:
			decode_3char(decode_buffer, *output);
			(*output) += 2;
			break;
		}
		return false;
	}
	
	decode_buffer[decode_buffer_idx] = input;
	decode_buffer_idx++;
		
	if (decode_buffer_idx >= 4)
	{
		decode_4char (decode_buffer, *output);
		decode_buffer_idx = 0;
		(*output) += 3;
	}
	return true;
}
#endif

#endif
