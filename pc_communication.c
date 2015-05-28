#include "timer_ticks.h"
#include "m_general.h"
#include "m_usb.h"
#include "pc_communication.h"
#include "base64.h"

static uint8_t message[MAX_MESSAGE_LENGTH];
static uint8_t *message_ptr = message;

pc_data latest_pc_data;
static bool new_data = false;

bool new_pc_data (void)
{
	return new_data;
}

pc_data *get_pc_data (void)
{
	new_data = false;
	return &latest_pc_data;
}

// message format:
//  : (colon) = start of message
//  ; (semicolon) = end of message
//  First two letters of message: message type

bool received_pc_message (void)
{
    #ifdef ENABLE_PC_COMMUNICATION_TIMEOUT
    static uint64_t recent_data_tick = get_timer_ticks();
    #endif
    
    static bool in_message_frame = false;
    
    if (m_usb_rx_available())
    {
        const char byte = m_usb_rx_char();
        
        switch (byte)
        {
            case ':':  // start of a new message
                message_ptr = message;
                in_message_frame = true;
                break;
            case ';':  // end of a message
                in_message_frame = false;
                if (message_ptr > message)  // if there's data between the start and end tokens
                    return true;
                break;
            default:  // other characters
                if (in_message_frame)  // ignore data that isn't within a :; frame
                {
                    *message_ptr = byte;
                    
                    // make sure there's space left in the buffer before incrementing message_ptr
                    if (message_ptr - message + 1 < MAX_MESSAGE_LENGTH)
                        message_ptr++;
                }
                break;
        }
        
        #ifdef ENABLE_PC_COMMUNICATION_TIMEOUT
        recent_data_tick = get_timer_ticks();
        #endif
    }
    #ifdef ENABLE_PC_COMMUNICATION_TIMEOUT
    else if (in_message_frame)
    {  // check for a timeout in the middle of a message
        if (us_since (recent_data_tick) >= MESSAGE_TIMEOUT_US)
        {
            in_message_frame = false;
            
            if (message_ptr > message)
                return true;
        }
    }
    #endif
    
    return false;
}

static inline void clear_pc_message (void)
{  // to be called when a message has been processed
    message_ptr = message;
}

static inline uint8_t atoi_2digit (const char *a)
{
    // ASCII '0' is 48
    
    if (a[0] < 48 || a[0] > 57 ||
        a[1] < 48 || a[1] > 57)
    {
        return 0;
    }
    
    return ((a[0] - 48) * 10) + (a[1] - 48);
}

static inline uint8_t atoi_3digit (const char *a)
{
    // ASCII '0' is 48
    
    if (a[0] < 48 || a[0] > 57 ||
        a[1] < 48 || a[1] > 57 ||
        a[2] < 48 || a[2] > 57)
    {
        return 0;
    }
    
    return ((a[0] - 48) * 100) + ((a[1] - 48) * 10) + (a[2] - 48);
}

static enum
{
    ECHO,      // for testing
    DATA,      // acceleration/slip/temperature/pressure from sensor
    UNKNOWN
} message_type (void)
{
    if (message[0] == 'E' && message[1] == 'C')
        return ECHO;
    if (message[0] == 'D' && message[1] == 'A')
        return DATA;
    return UNKNOWN;
}

static void m_usb_tx_u64 (uint64_t val)
{
	char string[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t string_pos = 20;
	
	do 
	{
		string[string_pos] = '0' + val % 10;
		val /= 10;
		string_pos--;
	} while (val != 0);
	
	for (uint8_t i = 0; i < 21; i++)
	{
		if (string[i])
			m_usb_tx_char (string[i]);
	}
}

void process_pc_message (void)
{
    const uint16_t message_len = message_ptr - message;
    
    if (message_len < 2)  // messages must be at least two characters long
    {
        m_usb_tx_string (":!;");  // command error
        m_usb_tx_push();
        clear_pc_message();
        return;
    }
    
    switch (message_type())
    {
        case ECHO:
            m_usb_tx_char (':');
            for (uint16_t i = 0; i < message_len; i++)
                m_usb_tx_char (message[i]);
            m_usb_tx_string ("(ECHO);");
            break;
        case DATA:
			{
				//Data format
				// :DA<base64-encoded pc_data struct>;
				
				// decode the base64 string in-place to re-use the message buffer
				// (will work because the base64 string is always longer than what it encoded)
				uint8_t *message_begin = message + 2;  // skip past the "DA" identifier
				uint8_t *decoded_end_ptr = message_begin;
				
				{
					bool decode_working = true;
				
					base64_begin_decode_stream();
					for (uint16_t i = 0; i < (message_len - 2) && decode_working; i++)
						decode_working = base64_decode_stream (&decoded_end_ptr, *(message_begin + i));
				}
				
				const uint16_t decoded_len = decoded_end_ptr - message_begin;
				
				if (decoded_len != sizeof(pc_data))
				{
					m_usb_tx_string (":DA!");  // send "invalid data" response
					m_usb_tx_int ((int)decoded_len);
					m_usb_tx_string ("vs");
					m_usb_tx_int ((int)sizeof(pc_data));
					m_usb_tx_char (',');
					if (decoded_len >= sizeof(uint64_t))
					{  // include the data's timestamp, if we got that much of it
						const uint64_t *out_ptr = (uint64_t*)message_begin;
						m_usb_tx_u64 (*out_ptr);
					}
					m_usb_tx_char (';');
				}
				else
				{
					// copy the decoded data into our waiting structure
					pc_data *pc_ptr = (pc_data*)message_begin;
					latest_pc_data = *pc_ptr;
					new_data = true;
					
					// debug: indicate successful message and write the bytes received
					m_usb_tx_string (":DAOK");
					m_usb_tx_u64(latest_pc_data.timestamp_us);
					m_usb_tx_string(", ");
					m_usb_tx_int((int)latest_pc_data.num_accel_samples);
					m_usb_tx_string(";");
				}
			}
            break;
        case UNKNOWN:
        default:
            m_usb_tx_string (":!;");  // command error
            break;
    }
    
    m_usb_tx_push();
    clear_pc_message();
}




