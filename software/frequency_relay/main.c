/*
 * Suraj Kumar
 */

#include <system.h>
#include <altera_avalon_pio_regs.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/alt_irq.h>
#include <sys/alt_alarm.h>
#include <string.h>

enum state_type{RR,GR,YR,RG,RY,GRP,RGP};
volatile enum state_type state = RR;
volatile enum state_type prev_state = RY;

volatile int mutex_lock_flag = 0;

volatile int mode = 0;

enum ped_state{IDLE,NSP,EWP};
volatile enum ped_state NSstate = IDLE;
volatile enum ped_state EWstate = IDLE;
volatile int pedNS = 0;
volatile int pedEW = 0;
volatile int NShandled = 0;
volatile int EWhandled = 0;
volatile int NSraised = 0;
volatile int EWraised = 0;
volatile int button_event = 0;
volatile int cam_count = 0;
volatile int vehicle_in = 0;
volatile int snap_taken = 0;
volatile int timer_event = 0;
volatile int entered_on_green = 0;

volatile int predef_time = 200; // 200 * 10ms = 2s

volatile int T1 = 500;
volatile int T2 = 6000;
volatile int T3 = 2000;
volatile int T4 = 500;
volatile int T5 = 6000;
volatile int T6 = 2000;
volatile int timeout_set = 0;

#define RR_LED  IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x24)
#define GR_LED  IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x21)
#define YR_LED  IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x22)
#define RG_LED  IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x0c)
#define RY_LED  IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x14)
#define GRP_LED IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x61)
#define RGP_LED IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x8C)
#define ALL_LED IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0xFF)
#define NO_LED IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, 0x00)

#define READ_BUTTONS IORD_ALTERA_AVALON_PIO_DATA(BUTTONS_BASE)
#define READ_SWITCHES 262143 - IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE)
#define RED_LED_SWITCHES IOWR_ALTERA_AVALON_PIO_DATA(LEDS_RED_BASE, READ_SWITCHES)

#define ESC 27
#define CLEAR_LCD_STRING "[2J"



// timer isr for the main traffic light controller
// sets a flag to be handled in the main()
alt_u32 tlc_timer_isr(void* context){
	timer_event = 1;
	return 0;

}


// timer isr for the red light camera
// counts up by one every second
alt_u32 camera_timer_isr(void* context){
	cam_count++;
	printf("cam_count: %i \n", cam_count);
	return 10;
}


// button isr
void button_isr(void* context, alt_u32 id) {
	if (mutex_lock_flag == 0) {
		mutex_lock_flag = 1;
		int buttonValue = READ_BUTTONS;
		if (buttonValue == 6) { // KEY0 - NS Pedestrian Button
			NSraised = 1;
			NShandled = 0;
		} else if (buttonValue == 5) { // KEY1 - EW Pedestrian Button
			EWraised = 1;
			EWhandled = 0;
		} else if (buttonValue == 4) { // KEY0 and KEY1 at the same time
			EWraised = 1;
			EWhandled = 0;
			NSraised = 1;
			NShandled = 0;
		} else if (mode == 4 && buttonValue == 3) { // KEY2
			if (vehicle_in == 0) { // vehicle entering
				vehicle_in = 1;
				button_event = 1;
				snap_taken = 0;
			} else { // vehicle leaving
				vehicle_in = 0;
				button_event = 1;
			}
		}

		// clear edge capture register
		IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE, 0);

		mutex_lock_flag = 0;
	}
}


//////////////////////////////////////////////////          MAIN          \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

int main(void)
{
	alt_alarm timer;
	alt_alarm timer2;

	FILE *lcd;
	FILE *uart;

	uart = fopen(UART_NAME, "w");
	fprintf(uart,"\f");
	fclose(uart);

	int context = 1;
	int buttons_initialised = 0;
	int temp_mode = 0;
	int next_time = 0;


	int timeCountMain = 0;
	void* timerContext = (void*) &timeCountMain;
	alt_alarm_start(&timer, 500, tlc_timer_isr,timerContext);

	while(1) {

		temp_mode = switch_read_mode();
		// handles the switching of modes
		// only change mode when in safe state, when a proper mode (i.e. 1,2,3, or 4) has been chosen, and the mode is not the same mode
		if ((state == RR && temp_mode != 0 && temp_mode != mode) || (mode == 0)) {
			if (mode == 0) {
				lcd_mode(); // print "enter mode" to LCD
				NO_LED;
				// blocking until a mode is selected
				while(mode == 0) {
					change_mode(temp_mode);
					temp_mode = switch_read_mode();
				}
				// if selecting mode right at the start, reset the states and the timer

				alt_alarm_stop(&timer);
				state = RR;
				prev_state = RY;
				alt_alarm_start(&timer, 500, tlc_timer_isr,timerContext);
			} else {
				change_mode(temp_mode);
			}
			lcd_mode(); // print mode to LCD
		}

		// handles the main traffic light LEDs (all modes)
		simple_tlc_leds();

		// set the red LEDs based on which switches are turned on
		RED_LED_SWITCHES;

		// handles the changing of states (only when the timer flag is set, all modes)
		if (timer_event == 1 && mutex_lock_flag == 0) {
			mutex_lock_flag = 1;
			alt_alarm_stop(&timer);
			next_time = tlc_state_machine();
			alt_alarm_start(&timer, next_time, tlc_timer_isr,timerContext);
			timer_event = 0;
			mutex_lock_flag = 0;
		}


		if (mode == 1) { // MODE 1 ONLY
			init_buttons_pio(0,(void*) 1); // disable button interrupts in mode 1
		} else { // MODE 2, 3, 4 ONLY
			if (buttons_initialised == 0) { // enable button interrupts
				init_buttons_pio(1,(void*) 1);
				buttons_initialised = 1;
			}
			pedestrian_tlc();
		}


		if (mode == 3 || mode == 4) { // MODE 3, 4 ONLY
			// in RR state, if SW4 and SW2 are on, or if SW4 and SW3 are on, configure new timeout values
			// also, don't configure new values if timeout_set == 1 (which means that new values have been set but SW4 hasn't been turned off since then)
			if (state == RR && (READ_SWITCHES == 20 || READ_SWITCHES == 24) && timeout_set == 0) {
				alt_alarm_stop(&timer);
				// blocking function to handle new timeout values
				simple_tlc_leds(state); // sometimes the leds don't get set properly before the blocking function, so fixed it by setting them here
				while(timeout_data_handler(*uart));
				int temp_t;
				if (prev_state == RY) {
					temp_t = T1;
				} else if (prev_state == YR) {
					temp_t = T4;
				}
				alt_alarm_start(&timer, temp_t, tlc_timer_isr,timerContext);
			// once SW4 is turned off, set timeout_set back to zero so that new values can be configured when SW4 is turned back on
			} else if (state != RR && (READ_SWITCHES != 20 && READ_SWITCHES != 24 && READ_SWITCHES != 16)) {
				timeout_set = 0;
			}
		}
		if (mode == 4) { // MODE 4 ONLY
			if (vehicle_in == 1 && button_event == 1){
				// start vehicle timer, and print "Vehicle entered" to uart
				alt_alarm_start(&timer2,1000,camera_timer_isr,timerContext);
				button_event = 0;
				camera_uart(4,*uart);
				if (state == RG || state == GR) { // if car enters on a green, do not activate the camera
					entered_on_green = 1;
				} else {
					camera_uart(1,*uart);
					entered_on_green = 0;
				}
				if (state == RR) {
					// if vehicle enters on a red light, take snapshot immediately (but timer will still be active until the vehicle leaves)
					camera_uart(3,*uart);
					snap_taken = 1;
				}
			}
			else if (vehicle_in == 0 && button_event == 1){
				// stop red light camera timer, print "Vehicle left" and camera time to uart
				if (cam_count != 0) {
					alt_alarm_stop(&timer2);
					button_event = 0;
					camera_uart(2,*uart);
					cam_count = 0;
				}
			} else if (vehicle_in == 1 && (cam_count >= predef_time) && snap_taken == 0 && entered_on_green == 0) {
				// if vehicle is still in the intersection after a predefined time (and the snapshot wasn't already taken), take a snapshot
				camera_uart(3,*uart);
				snap_taken = 1;
			}
		}
	}

	return 0;
}

//////////////////////////////////////////////////////////// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void change_mode(int temp_mode) {
	// change mode
	mode = temp_mode;

	// reset all relevant global variables
	pedNS = 0;
	pedEW = 0;
	NShandled = 0;
	EWhandled = 0;
	NSraised = 0;
	EWraised = 0;
	button_event = 0;
	cam_count = 0;
	vehicle_in = 0;
	snap_taken = 0;
	timer_event = 0;

	// if switching to mode 1 or 2, change back to default timeout values
	if (temp_mode == 1 || temp_mode == 2) {
		T1 = 500;
		T2 = 6000;
		T3 = 2000;
		T4 = 500;
		T5 = 6000;
		T6 = 2000;

		// timeout_set should not be reset if switching between modes 3 and 4
		// if it was reset in mode 3/4, new values would have to be entered right away when switching between modes if SW4 is still on
		timeout_set = 0;
	}

}

// print to LCD
void lcd_mode(FILE *lcd) {
	lcd = fopen(LCD_NAME, "w");

	if (mode == 0) {
		fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
		fprintf(lcd, "Enter mode\n");
	} else {
		fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
		fprintf(lcd, "Mode: %d\n", mode);
	}

	fclose(lcd);
}

// state machine for main traffic lights
int tlc_state_machine() {
	int next_time = 0;
	if (state == RR) {
		if (prev_state == RY){
			next_time = T2;
			if ( pedNS == 0) {
				state = GR;
			} else if (mode != 1 && pedNS == 1) {
				state = GRP;
			}

		}
		else if (prev_state == YR){
			next_time = T5;
			if ( pedEW == 0) {
				state = RG;
			} else if ( mode!= 1 && pedEW == 1) {
				state = RGP;
			}
		}
		prev_state = RR;
	} else if(state == GR){
		next_time = T3;
		state = YR;
		prev_state = GR;
	}  else if(state == YR){
		next_time = T4;
		state = RR;
		prev_state = YR;
	}  else if (state == RG){
		next_time = T6;
		state = RY;
		prev_state = RG;
	} else if (state == RY){
		next_time = T1;
		state = RR;
		prev_state = RY;
	} else if (state == GRP) {
		next_time = T3;
		state = RY;
		prev_state = RGP;
		NShandled = 1;
		NSraised = 0;
	} else if (state == RGP) {
		next_time = T6;
		state = RY;
		prev_state = RGP;
		EWhandled = 1;
		EWraised = 0;
	}

	// returns the next timer time for the new state
	return next_time;
}

int switch_read_mode() {
	unsigned int switchesValue = READ_SWITCHES;
	if (switchesValue == 1) {
		return 1;
	} else if (switchesValue == 2) {
		return 2;
	} else if (switchesValue == 4 || switchesValue == 20) {
		return 3;
	} else if (switchesValue == 8 || switchesValue == 24) {
		return 4;
	} else {
		return 0;
	}
}

// MODE 1 FUNCTIONS

// set the LEDs based on the state
void simple_tlc_leds() {

		if (state == RR){
			RR_LED;
		}
		else if(state == GR){
			GR_LED;
		}
		else if (state == GRP) {
			GRP_LED;
		}
		else if(state == YR){
			YR_LED;
		}
		else if (state == RG){
			RG_LED;
		}
		else if (state == RGP){
			RGP_LED;
		}
		else if (state == RY){
			RY_LED;
		}
}

// MODE 2 FUNCTIONS


void init_buttons_pio(int enable, void* context_going_to_be_passed) {
	// clear the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE, 0);
	// enables/disables interrupts for all buttons
	if (enable == 1) {
		IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTONS_BASE, 0x7);
	} else {
		IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTONS_BASE, 0x0);
	}
	// register the ISR
	alt_irq_register(BUTTONS_IRQ,context_going_to_be_passed,button_isr);
}

// state machine(s) for pedestrian lights
void pedestrian_tlc() {

	if (NSstate == IDLE && NSraised == 1) {
		NSstate = NSP;
		pedNS = 1;
	} else if (NSstate == NSP && NShandled == 1) {
		NSstate = IDLE;
		pedNS = 0;
	}

	if (EWstate == IDLE && EWraised == 1) {
		EWstate = EWP;
		pedEW = 1;
	} else if (EWstate == EWP && EWhandled == 1) {
		EWstate = IDLE;
		pedEW = 0;
	}

}



// MODE 3 FUNCTIONS

int timeout_data_handler(FILE *uart) {
	printf("Waiting for new timeout values...\n");
	uart = fopen(UART_NAME, "w");
	fprintf(uart,"Enter new timeout values... \r\n");
	fclose(uart);
	uart = fopen(UART_NAME, "r");
	char data_array[35];
	// format: #,#,#,#,#,#[\r]\n
	// where # is a 1-4 digit integer
	// \r is ignored (and may or may-not be received)

	int current_char = 0;
	int previous_char = 0;
	int i = 0;

	printf("Input: ");



	// get values until "\n" entered
	while ((current_char != 'n' || previous_char != '\\') && current_char != 10) { // will terminate if an actual new line character is entered (ascii 10), or if a '\', followed by a 'n' is entered
		if (i > 34) { // too many values entered,
			uart = fopen(UART_NAME, "w");
			fprintf(uart,"\r\nToo many Timeout Values, please enter new ones.\r\n");
			fclose(uart);
			printf("\nToo many Timeout Values, please enter new ones. \n");
			return 1;
		}
		printf("%c",current_char);
		previous_char = current_char;
		current_char = fgetc(uart);
		data_array[i] = (char)current_char;
		i++;
	}
	fclose(uart);

	printf("%c",current_char);
	printf("\n");

	const char s[2] = ",";

	char *token = strtok(data_array,s);

	int temp[6];

	i = 0;

	// split input by ","
	while(token != NULL) {
		temp[i] = atoi(token);
		token = strtok(NULL, s);
		i++;
	}


	// return 1 (i.e. start the function again) if character is invalid)
	for (i = 0; i < 6; i++) {
		if (temp[i] <= 0 || temp[i] >= 10000 || temp[i] == NULL) {
			uart = fopen(UART_NAME, "w");
			fprintf(uart,"\r\nBad Timeout Values, please enter new ones.\r\n");
			fclose(uart);
			printf("Bad Timeout Values, please enter new ones. \n");
			return 1;
		}
	}

	// set timeouts

	T1 = temp[0];
	T2 = temp[1];
	T3 = temp[2];
	T4 = temp[3];
	T5 = temp[4];
	T6 = temp[5];
	timeout_set = 1;

	uart = fopen(UART_NAME, "w");
	fprintf(uart,"T1: %d, T2: %d, T3: %d, T4: %d, T5: %d, T6: %d\r\n\n", T1, T2, T3, T4, T5, T6);
	fclose(uart);

	printf("New Timeout Values -  T1: %d, T2: %d, T3: %d, T4: %d, T5: %d, T6: %d\n", T1, T2, T3, T4, T5, T6);

	return 0;
}


// MODE 4 FUNCTIONS

void camera_uart(int s,FILE *uart){
	uart = fopen(UART_NAME, "w");

	// formatting looks ugly here but good on PuTTY

	printf("entered_on_green: %d", entered_on_green);

	if (s == 1){
		fprintf(uart,"    ---- Camera Activated ----    \r\n\n");

	}
	else if(s == 2){
		fprintf(uart,"      ---- Vehicle Left ----    \r\n");
		// cam_count is the amount of 10s of ms that have passed, so divide by 10 to display in seconds
		fprintf(uart,"  Vehicle left after %.2f seconds    \r\n\n\n",((double)cam_count)/100);
	}

	else if (s == 3) {
		fprintf(uart,"          Snapshot Taken    \r\n\n");
	}

	else if (s == 4) {
		fprintf(uart,"\f     ---- Vehicle Entered ----    \r\n");
	}

	fclose(uart);
}






