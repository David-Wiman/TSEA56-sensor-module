/*
 * main.c
 *
 * Created: 2022-04-04 10:04:44
 * Author : alvgu648
 * Use second JTAG device in Daisy chain list.
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "i2c/avr_i2c.h"
#include "common/i2c_common.h"

volatile uint16_t time_var = 0;
volatile uint16_t left_hall_time = 0;
volatile uint16_t right_hall_time = 0;
volatile uint16_t pre_left_time_var = 0;
volatile uint16_t pre_right_time_var = 0;
volatile uint16_t times_without_hall = 0;

volatile int right_hall_prescaler = 0;
volatile int left_hall_prescaler = 0;
uint16_t IR_distance_mean = 0;


//volatile int count_r = 0;  // actually not really used
//volatile int count_l = 0;  // actually not really used
volatile uint16_t driven_distance_right = 0;  // In dm
volatile uint16_t driven_distance_left = 0;  // In dm

volatile uint16_t left_speed;  // In mm/s
volatile uint16_t right_speed;  //In mm/s
int speed_buffer_size = 10;
int left_speed_buffer[10];
int left_speed_buffer_index = 0;
int right_speed_buffer[10];
int right_speed_buffer_index = 0;



volatile uint16_t IR_output = 0;
volatile uint16_t IR_distance = 0;
int IR_buffer_size = 10;
int IR_buffer[10];
int IR_buffer_index = 0;



void init_hall_timer() {
	
	TCCR0A = 0;
	TCCR0B = (1<<CS01) | (0<<CS00);	//Set prescale to 8
	TIMSK0 = (1<<TOIE0); // Set interrupts on overflow
}

void init_ir_timer() {
	
	TCCR1A = 0;
	TCCR1B = (1<<WGM12) | (1<<CS01) | (1<<CS00);  // Set CTC and prescale = 64
	TIMSK1 = (1<<OCIE1A); // Set OCR1A as top-value
	OCR1A = 5000;
}

void init_interrupts_INT0_1() {
	EICRA = (1<<ISC00) | (1<<ISC01) | (1<<ISC10) | (1<<ISC11); // Set interrupts on rising edge
	EIMSK = (1<<INT1) | (1<<INT0);	//Enable interrupts on INT0 and INT1
}

void init_ADC() {
	ADMUX = (1<<REFS1) | (1<<REFS0);  // Set referense-voltage to internal 2.56V.
	ADCSRA = (1<<ADEN) | (1<<ADIE);	//Enable ADC and ADC Interrupt
}

// This function should only be used in the same block as IR_buffer is already accessed, to prevent multiple accesses at the same time
// Note that this function updates IR_distance_mean directly rather than returning it

uint16_t calc_buffer_mean(int *buffer, int buffer_size) {
	uint16_t sum = 0;
	for (int i=0; i<buffer_size; i++) {
		sum += buffer[i];
	}
	return sum/buffer_size;
}

 // Interupt 25 times per second. Check if speed=0 and start ADC.
ISR (TIMER1_COMPA_vect) {
	times_without_hall++;
	if (times_without_hall >= 10) {
		uint16_t message_names[] = {SENSOR_RIGHT_SPEED, SENSOR_LEFT_SPEED};
		times_without_hall = 0;
 		right_speed = 0;
		left_speed = 0;
		uint16_t messages[] = {right_speed, left_speed};
		I2C_pack(message_names, messages, 2);
	}

	ADCSRA |= (1<<ADSC);	// Start ADC conversion.
}

 // One time_var = 0.256ms
ISR (TIMER0_OVF_vect) {
	time_var++;
}
	
	// IR distance calculation
ISR (ADC_vect)  {
	IR_output = ADCL;  // Read bit 1-8.
	IR_output |= (ADCH<<8);  // Read bit 9-10
	if (IR_output < 700) {
		IR_distance = 400*63/IR_output;  // (1024 steps)/(2.56 V) = 400 steps/V, taking inverse to get distance
	}  else {
		IR_distance = 71 - 20*IR_output/400 ;  // See documentation for details about this calculation
	}
	if (IR_distance < 150) {
		IR_buffer[IR_buffer_index] = IR_distance;  // Put in buffer and rotate index of buffer
		IR_buffer_index++;
		if (IR_buffer_index >= IR_buffer_size) {
			IR_buffer_index = 0;
		}	
		IR_distance_mean = calc_buffer_mean(IR_buffer, IR_buffer_size);
	} else {
		IR_distance_mean = 0;
	}
	
	I2C_pack_one(SENSOR_OBSTACLE_DISTANCE, IR_distance_mean);
}
	
// Left hall sensor
ISR (INT0_vect) {
	times_without_hall = 0;
	left_hall_time = time_var - pre_left_time_var;
	pre_left_time_var = time_var;
	
	if (left_hall_time == 0) {
		return;
	}

	left_speed = 98175/left_hall_time; // 1000*0.0080*pi*1000/0.256 = 98175

	 // Averaging code for left_speed
	left_speed_buffer[left_speed_buffer_index] = left_speed;  // Put in buffer and rotate index of buffer
	left_speed_buffer_index++;
	if (left_speed_buffer_index >= speed_buffer_size) {
		left_speed_buffer_index = 0;
	}
	uint16_t left_speed_mean = calc_buffer_mean(left_speed_buffer, speed_buffer_size);

	 // Count up driven distance left
	left_hall_prescaler++;
	if (left_hall_prescaler >= 4) {
		driven_distance_left++;
		
		uint16_t message_names[] = {SENSOR_LEFT_SPEED, SENSOR_LEFT_DRIVING_DISTANCE};
		uint16_t messages[] = {left_speed_mean, driven_distance_left};
		I2C_pack(message_names, messages, 2);
		left_hall_prescaler = 0;

	}

}

// Right hall sensor
ISR (INT1_vect) {
	times_without_hall = 0;
	right_hall_time = time_var - pre_right_time_var;
	pre_right_time_var = time_var;
	
	if (right_hall_time == 0) {
		return;
	}
	right_speed = 98175/right_hall_time; // 1000*0.0080*pi*1000/0.256 = 98175

	 // Averaging code for right_speed
	right_speed_buffer[right_speed_buffer_index] = right_speed;  // Put in buffer and rotate index of buffer
	right_speed_buffer_index++;
	if (right_speed_buffer_index >= speed_buffer_size) {
		right_speed_buffer_index = 0;
	}
	uint16_t right_speed_mean = calc_buffer_mean(right_speed_buffer, speed_buffer_size);

	 // Count up driven distance right
	right_hall_prescaler++;
	if (right_hall_prescaler >= 4) {
		driven_distance_right++;
		
		uint16_t message_names[] = {SENSOR_RIGHT_SPEED, SENSOR_RIGHT_DRIVING_DISTANCE};
		uint16_t messages[] = {right_speed_mean, driven_distance_right};
		I2C_pack(message_names, messages, 2);
		right_hall_prescaler = 0;

	}
}



int main() {
	init_hall_timer();
	init_interrupts_INT0_1();
	init_ADC();
	init_ir_timer();

	I2C_init(SENSOR_MODULE_SLAVE_ADDRESS);
	sei();
    /* Replace with your application code */
    while (1) {}
	return 1;
}

