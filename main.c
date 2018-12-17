/*
|@author Oscar Rovira
|@version 1.0 14/12/2018
*/

#include "EE30186.h"
#include "system.h"
#include "socal/socal.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define key0 0xE
#define key1 0xD
#define key2 0xB
#define key3 0x7

//Pointers to the hardware address of the required I/O and counter
volatile int * LEDs =			(volatile int *)(ALT_LWFPGA_LED_BASE);
volatile int * Switches =		(volatile int *)(ALT_LWFPGA_SWITCH_BASE);
volatile int * push_button =		(volatile int *)(ALT_LWFPGA_KEY_BASE);
volatile int * GpioPort =		(volatile int *)(ALT_LWFPGA_GPIO_1A_BASE);
volatile int * GpioPort_test =		(volatile int *)(ALT_LWFPGA_GPIO_1B_BASE);
volatile int * Counter =		(volatile int *)(ALT_LWFPGA_COUNTER_BASE);
volatile int * nums_right =		(volatile int *)(ALT_LWFPGA_HEXA_BASE);
volatile int * nums_left =		(volatile int *)(ALT_LWFPGA_HEXB_BASE);


//*********************************************************************************

void set_pin(int bit_pos, int state) {
/**
	Sets the bit of GPIO Port to HIGH or LOW.

	@param bit_pos The position number of the bit to set HIGH or LOW.
	@param state A 0 or 1 depending if it is required as LOW or HIGH.
 	@return None.
*/
	//If state is 0, carry a 0 to the desired position
	if(state<1) *GpioPort &= ~(1UL << bit_pos);
	//If state is 1, carry a 1 to the desired position
	else *GpioPort |= (1UL << bit_pos);
}

//*********************************************************************************

int get_hex(int num) {
/**
	Get the hex equivalent for the 7SD given the decimal number.

	@param num The desired decimal number to output in the seven segment display.
	@return The hex value.
*/
	int val;
	switch(num) {
		case 0:
			val = 0x40;
			break;
		case 1:
			val = 0x79;
			break;
		case 2:
			val = 0x24;
			break;
		case 3:
			val = 0x30;
			break;
		case 4:
			val = 0x19;
			break;
		case 5:
			val = 0x12;
			break;
		case 6:
			val = 0x02;
			break;
		case 7:
			val = 0x78;
			break;
		case 8:
			val = 0x00;
			break;
		case 9:
			val = 0x18;
			break;
	}
	return val;
}

//*********************************************************************************

int get_size(int num) {
/**
	Get the size of the decimal number given as input. If num = 65 return 2.

	@param num A decimal number.
	@return The number of decimals the input number has.
*/

	int count = 0;
	while(num>0)
	{
		num /= 10;
		count +=1;
	}
	return count;
}

//*********************************************************************************

void delay(int time_ms) {
/**
	Create a delay of time_ms milliseconds.

	@param time_ms The time in miliseconds to delay.
	@return None.
*/

	int prev_t = *Counter;
	//Convert from time to clock cycle counts
	int count_ms = time_ms*0xC350;
	//While the difference is not greater than the cycles
	//that correspond to the given ms, keep on looping.
	while(abs(*Counter - prev_t) <= count_ms);
}

//*********************************************************************************

void Display(int number) {
/**
	Display the given number in the seven segment display

	@param number The number to display.
	@return None.
*/

	int val_r, val_l;
	val_l = 0xFFFF;
	val_r = 0xFFFFFFFF;

	if(number<0) {
		number = -number;
		//val_l = 0x3FFF;
		val_r = 0xFFFFFF3F;
	}

	int input = number;
	int input_cp = input;
	int shift = 8;

	//If input == 0
	if (input==0) {
		val_l = 0xFFFF;
		val_r = 0xFFFFFF40;
	}

	int len_num = get_size(input);
	int each_num, hex_num_raw, hex_num_shifted;

	//Shift default value to allow for incoming numbers on the right display
	val_r = val_r << len_num*shift;

	//Shift default value to allow for incoming numbers on the left display
	if(len_num>4) {
		val_l = val_l << (len_num-4)*shift;
	}


	for(int i=0; i<len_num;i++)
	{
		//Get the lowest decimal number from input
		each_num = input_cp%10;

		//Switch to the left decimal
		input_cp /= 10;

		//Get the hex value
		hex_num_raw = get_hex(each_num);

		//Shift input val (if required)
		hex_num_shifted = hex_num_raw << shift*i;

		//If overflow on the right means you need to use the left displays
		if (hex_num_shifted<1) {
			//Shift input val (if required)
			hex_num_shifted = hex_num_raw << shift*(i-4);
			val_l = val_l | hex_num_shifted;
		}
		else {
			//Update Right displays
			val_r = val_r | hex_num_shifted;
		}

	}

	*nums_right = val_r;
	*nums_left = val_l;

}

//*********************************************************************************

void Display_char(char str[]) {
/**
	Display the given word in the seven segment display

	@param str[] The string to display.
	@return None.
*/

	int val_l = 0xFFFF;
	int val_r = 0xFFFFFFFF;

	int letter;
	int val_on[2] = { 0x2B, 0x40};
	int val_off[3] = {0xE, 0xE, 0x40};
	int val_open[4] = {0x48, 0x06, 0x0C, 0x40};


	//Display ON
	if(strlen(str) == 2)
	{
		//Shift default value to allow for incoming letters on the right display
		val_r = val_r << strlen(str)*8;
		for(int i=0;i<strlen(str);i++)
		{
			letter = val_on[i] << 8*i;
			val_r = val_r | letter;
		}

	}
	//Display OFF
	if(strlen(str) == 3)
	{
	//Shift default value to allow for incoming letters on the right display
		val_r = val_r << strlen(str)*8;
		for(int i=0;i<strlen(str);i++)
		{
			letter = val_off[i] << 8*i;
			val_r = val_r | letter;
		}

	}
	//Display OPEN
	if(strlen(str) == 4)
	{
	//Shift default value to allow for incoming letters on the right display
		val_r = val_r << strlen(str)*8;
		for(int i=0;i<strlen(str);i++)
		{
			letter = val_open[i] << 8*i;
			val_r = val_r | letter;
		}

	}
	//Display CLOSED
	if(strlen(str) == 6)
	{
		val_l = 0x4647;
		val_r = 0x40120621;

	}

*nums_right = val_r;
*nums_left = val_l;
}

//*********************************************************************************

int check(const int a[], int n) {
/**
	Function that checks that all values in array are equal.
	Inspired from:
	https://stackoverflow.com/questions/14120346/c-fastest-method-to-check-if-all-array-elements-are-equal

	@param a[] Array to check if numbers are equal.
	@param n Size of array a[].
	@return 1 if all numbers in a[] are equal, otherwise return 0.
*/

	while(--n>0 && a[n]==a[0]);
    return n==0;
}

//*********************************************************************************

void knight_rider() {
/**
	Function that displays the knight rider animation.

	@param None.
	@return None.
*/
	int state = 0;
	*LEDs = state;
	for(int t=0;t<2;t++) {
		for(int i=0; i<10;i++) {
			delay(50);
			state |= (1<<1*i);
			*LEDs = state;
		}
		for(int i=0; i<10;i++){
			delay(50);
			state = state << 1;
			*LEDs = state;
		}
	}

}

//*********************************************************************************

void LED_state(int duty_cycle) {
/**
	Function that displays the duty cycle in the set of LEDs.

	@param duty_cycle The current duty cycle to display in LED's.
	@return None.
*/
	//Get the first decimal value of the duty_cycle
	int pos = duty_cycle/10;
	//Shift the 1's from left to right
	*LEDs = 0xFFF << (10 - pos);
}


int get_pin(int port) {
/**
	Function that returns the value of the bit at GpioPort.

	@param port Desired bit position to get.
	@return None.
*/
	return (*GpioPort & (1 << port)) >> port;
}

//*********************************************************************************

int read_encoder(int encoder_A, int encoder_B, int encoder) {
/**
	Function that creates a count from the encoder. If encoder is rotated
	clock-wise the counter increases, otherwise if encoder is rotated
	anti clock-wise the counter decreases.

	@param encoder_A Bit that contains the state of the encoder A signal.
	@param encoder_B Bit that contains the state of the encoder B signal.
	@param encoder Previous value of the count encoder.
	@return encoder New value of the count encoder.
*/
	int valA = get_pin(encoder_A);
	int valB = get_pin(encoder_B);
	static int rotation = 0;

	if(rotation == 1) {
		if(valA == 1 && valB == 0){
			if(encoder>0){
				encoder -=3;
			}
		}
		else if (valA == 0 && valB == 1)encoder +=3;
		rotation = 0;
	}

	if(valA == valB && valA == 1){
		rotation = 1;
	}
	//Max the encoder to 100
	if(encoder>100)encoder = 100;
	//Min the encoder to 0
	if(encoder<0)encoder = 0;

	return encoder;
}

//*********************************************************************************

int mode_selector(int key_on, float D, int real_rpm, int ideal_rpm, int error,  int enable_pid) {
/**
	Function that changes the mode of the program to Closed or Open according to user input.
	Also displays the corresponding values according to the pressed key.
	Displays the corresponding values and words required for a seamless experience.

	@param key_on Variable that contains the state of pressed keys.
	@param D Duty Cycle of PWM.
	@param real_rpm Tachometer speed of fan in RPM.
	@param ideal_rpm Target speed in RPM that the user selects.
	@param error Error between the target speed and the tachometer speed.
	@param enable_pid Flag used to let the system know when closed loop mode is ON.
	@return key_on Variable that contains the state of pressed keys.
*/
	switch(*push_button) {
		case key0:
			key_on = 0;
			break;
		case key1:
			key_on = 1;
			break;
		case key2:
			key_on = 2;
			break;
		case key3:
			key_on = 3;
			break;
	}

	if(key_on == 0) {
		//Show Duty Cycle
		//*nums_left = 0x7063;
		*nums_left = 0xA127;
		Display(D);
	}

	else if(key_on == 1) {
		//Show Tachometer RPM's
		//*nums_left = 0xAF06;
		*nums_left = 0x120C;
		Display(real_rpm);
	}

	else if(key_on == 2) {
		//Show ideal RPM
		if(enable_pid == 1) {
			*nums_left = 0x07AF;
			Display(ideal_rpm);
		}
		else {
			//*nums_left = 0xAF06;
			*nums_left = 0x120C;
			Display(real_rpm);
		}

	}
	else if(key_on == 3) {
		//Show error
		if(enable_pid == 1) {
			*nums_left = 0x6AF;
			Display(error);
		}
		else {
			//*nums_left = 0xAF06;
			*nums_left = 0x120C;
			Display(real_rpm);
		}
	}
	return key_on;
}

//*********************************************************************************

int main(int argc, char** argv) {
/**
	Main Function

	@param argc.
	@param arv.
	@return None.
*/

	//function call to initialise the FPGA configuration
	EE30186_Start();


	volatile int * GpioDdr = GpioPort + 1;
	//Define only fan switch to be output
	*GpioDdr = 0x8;

	volatile int * GpioDdr_test = GpioPort_test + 1;
	//Define TP4 and TP5 of the external board to be outputs
	*GpioDdr_test = 0xC;
	*GpioPort_test = 0;

	//------------------------------------------------------------------------------
	//Variables definition for encoder reader

	//Define pin location of encoder A and B in GPIO Port
	const int encoder_A = 17;
	const int encoder_B = 19;
	//Encoder counter
	int encoder = 0;
	//Initialise encoder counter
	int encod_prev_count;

	//------------------------------------------------------------------------------
	//Variables definition for PWM Transmitter

	//Frequency of FPGA counter
	int freq_counter = 50000000;
	//Define fsw of PWM to be 750Hz.
	int fsw = freq_counter/750;
	//Duty Cycle [0, 100]
	float D = 0;
	//Define pin location of fan in GPIO Port
	const int fan_pin = 3;
	//Variable used to know if you are in high or low state of PWM
	int flag_fan = 1;
	//Variable that contains the duty time
	int on_duty;
	//Initialise PWM counter
	int pwm_prev_t;

	//------------------------------------------------------------------------------
	//Variables definition for Tachometer

	//Define pin location of tachometer in GPIO Port
	const int tach_pin = 1;
	//Store in array the clock counts from timer
	double tacho_counters[2] = {0,0};
	//Init clock counter
	double clk_tacho = 0;
	//Variables defined for checking the state of reading the tachometer signal
	int state_tacho = 0;
	int pos_prev_count;
	//Counter used to know if the clock is at the first or second positive edge of
	//the tachometer
	int num_pos_edge = 0;
	//Array used to store the frequency values of tachometer
	int tacho_freq[6];
	//Counter used to store the 21 frequency values
	int num_tacho_freq = 0;
	//Initialise the average frequency readings from the content of tacho_freq
	int average = 0;
	//Initialise bool of when to start tachometer counter
	int start_counter;
	//Initialise variable that contains the actual RPM of the fan
	int real_rpm = 0;

	//------------------------------------------------------------------------------
	//PID Variables

	//Initialise error, counter, time difference and PID enabler.
	int error, pid_prev_count, pid_dt, enable_pid;
	//Initialise PID
	int init_PID = 0;

	//------------------------------------------------------------------------------
	//Init Variables

	//Initialise all LED's to off
	*LEDs = 0;
	//Variable that tells if system is initialised or not (Controlled with switches)
	int init = 1;
	//Initialise keys on state and ideal RPM set by user
	int key_on, ideal_rpm;


	//******************************************************************************
	//--------------------------While Loop of the system----------------------------
	//------------------------------------------------------------------------------

	while (1) {

		//Turn on the system by switching the left switch
		while(*Switches < 512) {
			//Display off in the seven segment display
			Display_char("off");
			//Turn off the fan
			set_pin(fan_pin, 0);
			//Tell the while loop that the system is init
			init = 0;
			//Turn off all the LED's
			*LEDs = 0;
		}

		if(init==0) {
			Display_char("on");
			delay(200);
			knight_rider();
			//Initialise bool of when to start tachometer counter and a general init bool
			start_counter = init = 1;
			//Initialise encoder counter, PWM counter and PID time error counter
			encod_prev_count = pwm_prev_t = pid_prev_count = *Counter;
			//Initialise keys, encoder, real RPM, ideal RPM, PID
			key_on = encoder = real_rpm = ideal_rpm = init_PID = 0;

		}

		//------------------------------------------------------------------------------
		//Tachometer Reader
		/*
		 * Check what is the time difference between the first not-noise positive edge and the second not-noise positive edge.
		 * To know if a positive edge is noise or not, check that the time difference between going up and down is between 5ms and 30ms
		 *  if so, use this positive edge for the tachometer speed measurement.
		 */

		state_tacho = get_pin(tach_pin);
		if (state_tacho == 1 && start_counter == 1) {
			pos_prev_count = *Counter;
			start_counter = 0;
		}
		else if(state_tacho == 0 && start_counter == 0)	{
			//If time difference at pos_edge is greater than 5ms or less than 30ms, use pos_prev_count to calculate frequency.
			if(abs(*Counter - pos_prev_count) >= 250000 && abs(*Counter - pos_prev_count) <= 1500000 && num_pos_edge == 0) {
				tacho_counters[num_pos_edge] = pos_prev_count;
				//Enable checking for next pos_edge, which will be used to calculate tachometer frequency.
				num_pos_edge = 1;
			}
			//If time difference at pos_edge is greater than 5ms or less than 30ms, use pos_prev_count to calculate frequency.
			else if(abs(*Counter - pos_prev_count) >= 250000 && abs(*Counter - pos_prev_count) <= 1500000 && num_pos_edge == 1)	{
				tacho_counters[num_pos_edge] = pos_prev_count;
				//Calculate tachometer frequency
				clk_tacho = abs(tacho_counters[1] - tacho_counters[0]);
				//Store RPM values to average after
				tacho_freq[num_tacho_freq] = 30000/((clk_tacho*1000)/freq_counter);
				num_tacho_freq++;
				//Reset counter
				if(num_tacho_freq >= 5)	{
					for (int i=0; i<5; i++)	{
						average += tacho_freq[i];
					}
					//Average the different measured tachometer RPM values and used the average as output
					real_rpm = average/num_tacho_freq;
					//Reset
					num_tacho_freq = average = 0;
				}

				//Reset checking for first pos_edge.
				num_pos_edge = 0;
			}
			start_counter = 1;
		}

		//------------------------------------------------------------------------------
		//Encoder Reader
		/*
		 * Read the encoder every 5000 loop cycles (There's no need to be constantly reading the encoder)
		 */

		if(abs(*Counter - encod_prev_count) >= 5000){
				encoder = read_encoder(encoder_A, encoder_B, encoder);
				encod_prev_count = *Counter;
			}

		//------------------------------------------------------------------------------
		//Mode Selector
		/*
		 * Displays numbers and characters according to the user mode selection (Open or Closed)
		 *  and key selection.
		 */

		key_on = mode_selector(key_on, D, real_rpm, ideal_rpm, error,enable_pid);

		//------------------------------------------------------------------------------
		//PID

		//Define ideal RPM according to the encoder reader. Start counting from 1100 RPM
		//map the target fan speed from [0, 100](Encoder) to [1100, 2600](Target min-max limits)

		ideal_rpm = 1100 +(encoder*15);

		error = ideal_rpm - real_rpm;
		pid_dt = abs(*Counter - pid_prev_count);
		pid_prev_count = *Counter;

		float kp = 0.000000000000001;
		float ki = 0.000000001;
		float kd = 0.00000001;

		if(*Switches >= 513){
			if(init_PID == 1){
				Display_char("closed");
				delay(500);
				init_PID = 0;
			}
			enable_pid = 1;
			//Output the duty cycle according to the PID controller
			D += ki*error*pid_dt + kd*error/pid_dt + kp*error;
		}
		else{
			if(init_PID == 0){
				Display_char("open");
				delay(500);
				init_PID = 1;
				Display(D);
			}
			enable_pid = 0;
			//Output the duty cycle according to the encoder reader
			D = encoder;
		}

		//------------------------------------------------------------------------------
		//PWM Transmitter
		/*
		 * Set output pin high for the amount of clock cycles that corresponds to ON duty cycle
		 *  and set the output pin low for the amount of clock cycles that corresponds to OFF duty cycle
		 */

		//Max and Min the duty cycle
		if(D>=100) D=100;
		else if(D<0.5) D=0.6;

		//map the duty cycle from [0,100] to [0,1] because it's going to use to multiply times
		//the PWM frequency which will give the corresponding clock cycles of on-duty cycle
		//Multiply times the frequency of PWM to
		//create on-duty_cycle
		on_duty = (D/100)*fsw;

		//Flow control of PWM
		if(((abs(*Counter - pwm_prev_t) >= on_duty) && flag_fan==1) || (D<=0.7))
		{
			set_pin(fan_pin, 0);
			flag_fan = 0;
		}
		else if(((abs(*Counter - pwm_prev_t) >= fsw) && flag_fan==0) || (D>=99.5))
		{
			pwm_prev_t = *Counter;
			set_pin(fan_pin, 1);
			flag_fan = 1;
		}

		//------------------------------------------------------------------------------
		//Show the Duty cycle in LED's
		LED_state(D);
	}
	//------------------------------------------------------------------------------
	//END While Loop

	//function call to clean up and close the FPGA configuration
	EE30186_End();
	return 0;

}
