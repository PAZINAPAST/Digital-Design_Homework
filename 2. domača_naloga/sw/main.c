/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdint.h>

#define GPIO 0xC0000000
#define TIMER 0xC0000080
#define TIMER_CNTL 0xC0000084
#define TIMER_CNTH 0xC0000088


int main()
{

    volatile uint32_t * led_device = (uint32_t *) (GPIO+4);
    volatile uint32_t * timer_config = (uint32_t *) TIMER ;
    volatile uint32_t * timer_count_low = (uint32_t *)(TIMER + 4); // second mistake: need to put parentheses around the addition
    volatile uint32_t * timer_count_high = (uint32_t *)(TIMER + 8);

    volatile uint64_t counter_new, counter_old, limit;


    volatile uint32_t led_value = 0x0000F0F0;
    *led_device = led_value;

    //reset counter
	*timer_config = 0x00000003;
	//start counter
	*timer_config = 0x00000001;
	// read counter
	limit = 50000000;


	while(1){

		led_value =  ~led_value;
	    *led_device = led_value;


		
    	counter_new = *timer_count_high;
    	counter_new = *timer_count_low + (counter_new << 32);
    	counter_old = counter_new;


    	while((counter_old + limit) > counter_new) {
    		counter_new = *timer_count_high;
    	    counter_new = *timer_count_low + (counter_new << 32);
    	}
		
		
		

    }

    return 0;
}
