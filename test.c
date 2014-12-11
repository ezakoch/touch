/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include "saast.h" //SAAST Library calls


int main(void)
{
		DDRE |= 1<<6;
		PORTE &= !(1<<6);
    /* insert your hardware initialization here */
    for(;;){
        /* insert your main loop code here */
    }
    return 0;   /* never reached */
}
