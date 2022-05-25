#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "system.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include "sys/alt_timestamp.h"
#include "io.h"
#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

// Resource definitions
#define READ_SWITCHES IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Task Stacks
#define TASK_STACKSIZE 2048

// Task priorities
#define STORE_FREQUENCY_PRIORITY 6
#define SYSTEM_STATUS_PRIORITY 5
#define CONFIG_INPUT_PRIORITY 4
#define SWITCH_STATUS_PRIORITY 3
#define LOAD_MANAGEMENT_PRIORITY 2
#define RECORD_DISPLAY_PRIORITY 1

// Task execution delays
const TickType_t switchStatusDelay = 500;
const TickType_t systemStatusDelay = 100;
const TickType_t recordDisplayDelay = 250;

// Definition of Message Queue
#define   CURRENT_FREQ_QUEUE_SIZE  1
#define	  CONFIG_INPUT_QUEUE_SIZE 30
#define	  STATUS_QUEUE_SIZE 30
QueueHandle_t CurrentFreqQueue;
QueueHandle_t ConfigInputQueue;
QueueHandle_t StatusQueue;

// Definition of Semaphores
SemaphoreHandle_t current_freq_sem;
SemaphoreHandle_t configure_sem;
SemaphoreHandle_t maintenance_mode_sem;
SemaphoreHandle_t switch_status_sem;
SemaphoreHandle_t system_status_sem;
SemaphoreHandle_t detection_sem;
SemaphoreHandle_t measurement_time_sem;

// Timer definition
TimerHandle_t DeadlineTimer;
const TickType_t deadlinePeriod = 500;
TimerHandle_t SampleTimer;
const TickType_t samplingPeriod = 200;

// Event definition
EventGroupHandle_t systemStatusEvent;
#define STORE_BIT	( 1 << 0 )
#define TIMER_BIT	( 1 << 1 )
const uint32_t store_task = (1 << 0);
const uint32_t timer_task = (1 << 1);
const uint32_t both_tasks = ((1 << 0) | (1 << 1));

// Global variables
volatile int keyPress = 0;
double currentFreq = 0;
double currentROC = 0;
double underFreqValue = 49;
double maxROCValue = 9.99;
volatile int maintenanceMode = 0;
volatile char switchStatus[] = "";
volatile int systemStatus = 0;
volatile int detection = 0;
volatile int measurementTime[5] = {0,0,0,0,0};
volatile double avg = 0;
volatile int min = 9999;
volatile int max = 0;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

// Timer Call back functions
void vDeadlineTimerCallback(TimerHandle_t DeadlineTimer)
{
	xEventGroupSetBits(systemStatusEvent,TIMER_BIT);
}
//ISRs
void freq_read_ISR()
{
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0);
	xQueueOverwriteFromISR(CurrentFreqQueue,(void *)&temp, &xHigherPriorityTaskWoken);
	if (xSemaphoreTakeFromISR(detection_sem,&xHigherPriorityTaskWoken) == pdTRUE)
	{
		detection = (int) xTaskGetTickCount();
	}
	return;
}
void configuration_input_ISR(void *context, alt_u32 id)
{
	  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	  char ascii;
	  int status = 0;
	  unsigned char key = 0;
	  KB_CODE_TYPE decode_mode;
	  status = decode_scancode (context, &decode_mode , &key , &ascii) ;
	  if ( status == 0 ) //success
	  {
	    switch ( decode_mode )
	    {
	      case KB_ASCII_MAKE_CODE :
	    	if (keyPress == 0)
	    	{
	    		//Converts input ascii to equivalent keyboard number
	    		int temp = ascii - 48;
	    		xQueueSendFromISR(ConfigInputQueue, (void *)&temp, &xHigherPriorityTaskWoken);
	    		//global flag used to only send key press and not key release
	        	keyPress = 1;
	    	}
	    	else
	    	{
	    		keyPress = 0;
	    	}
	        break;
	      default :
	        break ;
	    }
	  }
}

void maintenance_isr(void* context, alt_u32 id)
{
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);
	xSemaphoreTakeFromISR(maintenance_mode_sem,xHigherPriorityTaskWoken);
	// Checks KEY3
	if (*temp == 4 && maintenanceMode == 1)
	{
		maintenanceMode = 0;
		printf("Resuming load management\n");
	}
	else if (*temp == 4 && maintenanceMode == 0)
	{
		maintenanceMode = 1;
		printf("Entering maintenance mode\n");
	}
	xSemaphoreGiveFromISR(maintenance_mode_sem,xHigherPriorityTaskWoken);
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}



// Tasks
void store_frequency_task (void *pvParameters)
{
	alt_irq_register(FREQUENCY_ANALYSER_IRQ,0,freq_read_ISR);
	int *temp;
	int oldADCValue = 0;
	int newADCValue = 0;
	double oldFreq = 0;
	double newFreq = 0;
	double ROC = 0;
	while(1)
	{
			if(xQueueReceive(CurrentFreqQueue,&temp,portMAX_DELAY) == pdPASS)
			{
				// Only store and compute ROC after two frequency readings
				if (oldADCValue == 0)
				{
					oldADCValue = (int)temp;
				}
				else
				{
					newADCValue = (int)temp;
					oldFreq = 16000.0 / oldADCValue;
					newFreq = 16000.0 / newADCValue;
					ROC = ((newFreq - oldFreq) * 32000.0) / (oldADCValue + newADCValue);
					oldADCValue = 0;
					newADCValue = 0;
					xSemaphoreTake(current_freq_sem,portMAX_DELAY);
					currentROC = fabs(ROC);
					currentFreq = oldFreq;
					xSemaphoreGive(current_freq_sem);
					xEventGroupSetBits(systemStatusEvent,STORE_BIT);
				}


			}
	}
}

void system_status_task (void *pvParameters)
{
	uint32_t uxBits;
	int status;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	int oldStatus = 2;
	// 1 = unstable
	// 0 = stable
	while(1)
	{
		uxBits = xEventGroupWaitBits(systemStatusEvent,both_tasks,pdTRUE,pdFALSE,2000);
		if (uxBits == STORE_BIT || uxBits == TIMER_BIT || uxBits == (TIMER_BIT | STORE_BIT))
		{// waits on bit set by either software timer or store frequency task
			xSemaphoreTake(current_freq_sem,portMAX_DELAY);
			xSemaphoreTake(configure_sem, portMAX_DELAY);
			if (currentFreq < underFreqValue || currentROC > maxROCValue )
			{//unstable condition. Exclusive comparison

				if (oldStatus != 1 || uxBits > 1)
				{// trigger load management either during change of state for first time or timer expiry
					status = 1;

					xTimerReset(DeadlineTimer,0);

					//only set status to 1 during first breach or timer expiry not during flactuation of states
					//status of 2 is arbitrary, doesn't cause any load management
					if (oldStatus == 2 || oldStatus == 1)
					{
						status = 1;
					}
					else
					{
						status = 2;
					}
					xQueueSend(StatusQueue,(void *)&status,xHigherPriorityTaskWoken);
					oldStatus = 1;
				}
				else if (oldStatus == 1 )
				{//Reset the measurement timer when corresponding frequency doesnt require load managament
					xSemaphoreGive(detection_sem);
				}
			}
			else {

				if (oldStatus != 0 || uxBits > 1)
				{
					status = 0;
					xTimerReset(DeadlineTimer,0);
					//only set status to 1 during first breach or timer expiry not during flactuation of states
					if (oldStatus == 0 || oldStatus == 2)
					{
						status = 0;
					}
					else
					{
						status = 2;
					}
					xQueueSend(StatusQueue,(void *)&status,xHigherPriorityTaskWoken);
					oldStatus = 0;

				}
				else if (oldStatus == 0)
				{
					xSemaphoreGive(detection_sem);
				}

			}
			xSemaphoreTake(system_status_sem,portMAX_DELAY);
			systemStatus = status;
			xSemaphoreGive(system_status_sem);
			xSemaphoreGive(current_freq_sem);
			xSemaphoreGive(configure_sem);
			vTaskDelay(systemStatusDelay);
		}
	}
}

void configuration_input_task (void *pvParameters)
{
	//Initialise Ps2 peripheral
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	if(ps2_device == NULL){
	  printf("can't find PS/2 device\n");
	}
	alt_up_ps2_clear_fifo (ps2_device) ;
	alt_irq_register(PS2_IRQ, ps2_device, configuration_input_ISR);
	IOWR_8DIRECT(PS2_BASE,4,1);
	//Variable to store what is received from queue
	int *temp;
	//Flags used to convey configuration state
	int CUnderFreqFlag = 0;
	int CMaxROCFlag = 0;
	int UnderFreq[2];
	int MaxROC[3];
	int i = 0;
	//Operation of configuration of threshold values explained in README
	while(1)
	{
		 if (xQueueReceive(ConfigInputQueue, &temp, portMAX_DELAY) == pdPASS)
		 {
			 if ((int)temp == 19){// 19 represents 'c' character
				 if (CUnderFreqFlag == 0 && CMaxROCFlag == 0)
				 {
					 //seven seg says 'c' when expecting further config values
					 IOWR(SEVEN_SEG_BASE,0,12);
					 printf("Enter integer value for lower frequency limit (max 99):\n");
				 	 CUnderFreqFlag = 1;
				 }
				 else if (CUnderFreqFlag == 1)
				 {
					 xSemaphoreTake(configure_sem, portMAX_DELAY);
					 printf("\nNew lower frequency threshold configured\n");
					 underFreqValue = UnderFreq[0]*10 + UnderFreq[1];
					 printf("Value is: %f\n",underFreqValue);
					 printf("Enter 2dp double value for maximum rate of change ( ie; 1.56): \n");
					 CMaxROCFlag = 1;
					 CUnderFreqFlag = 0;
					 i = 0;
					 xSemaphoreGive(configure_sem);
				 }
				 else if (CMaxROCFlag == 1){
					 xSemaphoreTake(configure_sem,portMAX_DELAY);
					 printf("\nNew rate of change threshold value configured\n");
					 maxROCValue = MaxROC[0] + (MaxROC[1]/10.0) + (MaxROC[2]/100.0);
					 printf("Value is: %f\n",maxROCValue);
					 IOWR(SEVEN_SEG_BASE,0,'0');
					 CMaxROCFlag = 0;
					 i = 0;
					 xSemaphoreGive(configure_sem);
				 }
			 }
			 else if (CUnderFreqFlag == 1 && i < 2)
			 {
				 UnderFreq[i] = (int) temp;
				 printf("%d",UnderFreq[i]);
				 i++;
			 }
			 else if (CMaxROCFlag == 1 && i < 3)
			 {
				 MaxROC[i] = (int) temp;
				 if (i == 0)
				 {
					 printf("%d.",MaxROC[i]);
				 }
				 else
				 {
					 printf("%d",MaxROC[i]);
				 }
				 i++;
			 }
		 }

	}
}

void switch_status_task (void *pvParameters)
{
	char switchValue[]= "";
	int buttonValue = 0;
	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	// enable interrupts for all buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	// register the ISR
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, maintenance_isr);
	while(1)
	{
		//reads switches values and stores into global char array
		int switchValDec = READ_SWITCHES;
		int i = 0;
		xSemaphoreTake(switch_status_sem,portMAX_DELAY);
		for(i = 0; i < 5;i++) // converting decimal value to binary to store into array
		{
			if (switchValDec % 2 == 0)
			{
				switchValue[4-i] = '0';
			}
			else
			{
				switchValue[4-i] = '1';
			}
			switchValDec = switchValDec / 2;
		}

		for (i = 0; i < 5; i++)
		{
			switchStatus[i] = switchValue[i];
		}
		xSemaphoreGive(switch_status_sem);
		vTaskDelay(switchStatusDelay);
	}
}

void load_management_task (void *pvParameters)
{
	int status;
	int *temp;
	char rLEDS[] = "00000";
	char gLEDS[] = "00000";
	int binary = 2;
	int rLedValue = 0;
	int gLedValue = 0;
	int maintenance;
	int i;
	int time = 0;
	int sum = 0;
	int count = 0;
	while(1)
	{
		if (xQueueReceive(StatusQueue,&temp,portMAX_DELAY) == pdPASS)
		{
			status = (int)temp;
			xSemaphoreTake(switch_status_sem,portMAX_DELAY);
			xSemaphoreTake(maintenance_mode_sem,portMAX_DELAY);
			maintenance = maintenanceMode;
			xSemaphoreGive(maintenance_mode_sem);
			//Checks if any manual switches are turned down
			for (i = 0;i < 5;i++){
				if (switchStatus[i] == '0')
				{
					rLEDS[i] = '0';
					gLEDS[i] = '0';
				}
			}
			if (maintenance == 1) // Checks maintenance mode
			{
				for (i = 0;i < 5;i++ )
				{
					if (switchStatus[4-i] == '1')
					{
						rLEDS[4-i] = '1';
					}
					gLEDS[4-i] = '0';

				}
			}
			else if (status == 1) // Management when system unstable
			{
				for (i = 0;i < 5; i ++)
				{
					if (switchStatus[4-i] == '1' && rLEDS[4-i] == '1')
					{
						rLEDS[4-i] = '0';
						gLEDS[4-i] = '1';
						break;
					}
				}
			}
			else if (status == 0) // Management when system stable
			{
				for (i = 0; i < 5; i++)
				{
					if (switchStatus[i] == '1' && rLEDS[i] == '0')
					{
						rLEDS[i] = '1';
						gLEDS[i] = '0';
						break;
					}
				}
			}
			//sets integer value for LEDs depending on array of load states
			binary = 2;
			rLedValue = 0;
			gLedValue = 0;
			for (i = 0; i < 5; i++)
			{
				if (rLEDS[4-i] == '1' )
				{
					rLedValue = rLedValue + pow(binary,i);
				}
				if (gLEDS[4-i] == '1' && maintenance == 0)
				{
					gLedValue = gLedValue + pow(binary,i);
				}
			}


			// Takes measurement of load management readings
			xSemaphoreTake(measurement_time_sem,portMAX_DELAY);
			time = (int) xTaskGetTickCount() - detection;
			xSemaphoreGive(detection_sem);
			if (time != 0 )
			{
				for (i = 4; i > 0; i--)
				{
					measurementTime[i] = measurementTime[i-1];
				}
				measurementTime[0] = time;
				sum += measurementTime[0];
				count++;
				avg =(double) sum / count;
				if (measurementTime[0] >= max)
				{
					max = measurementTime[0];
				}
				if (measurementTime[0] <= min)
				{
					min = measurementTime[0];
				}
			}
			xSemaphoreGive(measurement_time_sem);
			// sets green and red LEDs
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, rLedValue);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, gLedValue);
			xSemaphoreGive(switch_status_sem);

		}

	}
}

void record_display_task (void *pvParameters)
{
	FILE *lcd;
	lcd = fopen(CHARACTER_LCD_NAME, "w");
	int tick;
	while(1)
	{
			xSemaphoreTake(current_freq_sem,portMAX_DELAY);
			xSemaphoreTake(system_status_sem,portMAX_DELAY);
			fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
			fprintf(lcd, "%.2fHz ROC:%.2f\n",currentFreq,currentROC);
			if (systemStatus == 0)
			{
				fprintf(lcd,"System Stable\n");
			}
			else
			{
				fprintf(lcd,"System Unstable\n");
			}
			xSemaphoreGive(current_freq_sem);
			xSemaphoreGive(system_status_sem);
			tick = (int) xTaskGetTickCount();
			xSemaphoreTake(measurement_time_sem,portMAX_DELAY);
			printf("-------------------------------------------------------\n");
			printf("System uptime : %d.%d s\n",tick / 1000,(tick -  (tick/1000)*1000)/10 );
			printf("Time Measurements (ms):\n");
			printf ("%d  %d  %d  %d  %d\n", measurementTime[0],measurementTime[1],measurementTime[2],measurementTime[3],measurementTime[4]);
			printf("Average Time: %f  Minimum Time: %d  Maximum Time: %d\n",avg,min,max);
			printf("-------------------------------------------------------\n");
			xSemaphoreGive(measurement_time_sem);
			vTaskDelay(recordDisplayDelay);
	}
}



int main()
{
  IOWR_ALTERA_AVALON_PIO_DATA(SEVEN_SEG_BASE,'0');
  initCreateTasks();
  initOSDataStructs();
  vTaskStartScheduler();
  for (;;);
  return 0;
}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	// Queues
	ConfigInputQueue = xQueueCreate( CONFIG_INPUT_QUEUE_SIZE, sizeof( int ) );
	CurrentFreqQueue = xQueueCreate(CURRENT_FREQ_QUEUE_SIZE, sizeof(int));
	StatusQueue = xQueueCreate(STATUS_QUEUE_SIZE,sizeof(int));
	// Semaphores
	current_freq_sem = xSemaphoreCreateMutex();
	configure_sem = xSemaphoreCreateMutex();
	maintenance_mode_sem = xSemaphoreCreateMutex();
	switch_status_sem = xSemaphoreCreateMutex();
	system_status_sem = xSemaphoreCreateMutex();
	detection_sem = xSemaphoreCreateBinary();
	xSemaphoreGive(detection_sem);
	measurement_time_sem = xSemaphoreCreateMutex();
	// Timers
	DeadlineTimer = xTimerCreate("deadline",deadlinePeriod,pdFALSE,(void *) 0, vDeadlineTimerCallback);
	// Events
	systemStatusEvent = xEventGroupCreate();
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(store_frequency_task,"store_frequency_task",TASK_STACKSIZE,NULL,STORE_FREQUENCY_PRIORITY,NULL);
	xTaskCreate(system_status_task,"system_status_task",TASK_STACKSIZE,NULL,SYSTEM_STATUS_PRIORITY,NULL);
	xTaskCreate(configuration_input_task,"configuration_input_task",TASK_STACKSIZE,NULL,CONFIG_INPUT_PRIORITY,NULL);
	xTaskCreate(switch_status_task,"switch_status_task",TASK_STACKSIZE,NULL,SWITCH_STATUS_PRIORITY,NULL);
	xTaskCreate(load_management_task,"load_management_task",TASK_STACKSIZE,NULL,LOAD_MANAGEMENT_PRIORITY,NULL);
	xTaskCreate(record_display_task,"record_display_task",TASK_STACKSIZE,NULL,RECORD_DISPLAY_PRIORITY,NULL);
	return 0;
}



