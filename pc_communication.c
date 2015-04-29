#include "timer_ticks.h"
#include "m_general.h"
#include "m_usb.h"
#include "pc_communication.h"
#include "base64.h"

uint8_t message[MAX_MESSAGE_LENGTH];
uint8_t *message_ptr = message;

incoming_pc_data in_data;

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

inline void clear_pc_message (void)
{  // to be called when a message has been processed
    message_ptr = message;
}

inline uint8_t atoi_2digit (const char *a)
{
    // ASCII '0' is 48
    
    if (a[0] < 48 || a[0] > 57 ||
        a[1] < 48 || a[1] > 57)
    {
        return 0;
    }
    
    return ((a[0] - 48) * 10) + (a[1] - 48);
}

inline uint8_t atoi_3digit (const char *a)
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

enum
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

void process_pc_message (void)
{
    const uint8_t message_len = message_ptr - message;
    
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
            for (uint8_t i = 0; i < message_len; i++)
                m_usb_tx_char (message[i]);
            m_usb_tx_string ("(ECHO);");
            break;
        case DATA:
            break;
        case UNKNOWN:
        default:
            m_usb_tx_string (":!;");  // command error
            break;
    }
    
    m_usb_tx_push();
    clear_pc_message();
}



/*
    Data format:
        :DATA<base64-encoded current timer_ticks value>,<base64-encoded accelerometer values>;
    or, if there are no accelerometer values in this message:
        :DATA<base64-encoded current timer_ticks value>,-;
    or, if the accelerometer isn't working:
        :DATA<base64-encoded current timer_ticks value>,!;
*/

