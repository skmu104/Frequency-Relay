
//TODO : Store current frequency as a global using a semaphore

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "system.h"
#include "sys/alt_irq.h"
#include "io.h"
#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

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
const TickType_t configDelay = 1000;
const TickType_t storeFreqDelay = 2000;

// Definition of Message Queue
#define   CURRENT_FREQ_QUEUE_SIZE  30
#define	  CONFIG_INPUT_QUEUE_SIZE 30
QueueHandle_t CurrentFreqQueue;
QueueHandle_t ConfigInputQueue;

// Definition of Semaphores
SemaphoreHandle_t current_freq_sem;
//is this needed?
SemaphoreHandle_t current_ROC_sem;
SemaphoreHandle_t under_freq_sem;
SemaphoreHandle_t max_ROC_sem;


// Global variables

volatile int keyPress = 0;
double CurrentFreq = 0;
double CurrentROC = 0;
double UnderFreqValue = 50;
double MaxROCValue = 5;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

//ISRs
void freq_read_ISR()
{
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0);
	//double tempHz = 16000.0/ (double) temp;
	xQueueSendFromISR(CurrentFreqQueue,(void *)&temp, &xHigherPriorityTaskWoken);
	//printf("%f Hz\n", 16000/(double)temp);
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
	    //IOWR(SEVEN_SEG_BASE,0 ,key);
	  }
}



// Tasks
void store_frequency_task (void *pvParameters)
{

	alt_irq_register(FREQUENCY_ANALYSER_IRQ,0,freq_read_ISR);
	int *temp;
	while(1)
	{
		//printf("It is what it issss\n");
		if(xQueueReceive(CurrentFreqQueue,&temp,portMAX_DELAY) == pdPASS)
		{
			//printf("It is what it issss\n");
			xSemaphoreTake(current_freq_sem,portMAX_DELAY);
			CurrentFreq =  (int)temp;
			xSemaphoreGive(current_freq_sem);

		}
	}
}

void system_status_task (void *pvParameters)
{
	while(1)
	{
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

	int testing = 0;
	double testing1;

	while(1)
	{
		 if (xQueueReceive(ConfigInputQueue, &temp, portMAX_DELAY) == pdPASS)
		 {
			 if (temp == 19){
				 if (CUnderFreqFlag == 0 && CMaxROCFlag == 0)
				 {
					 printf("Enter integer value for lower frequency limit (max 99):\n");
				 	 CUnderFreqFlag = 1;
				 }
				 else if (CUnderFreqFlag == 1)
				 {
					 xSemaphoreTake(under_freq_sem, portMAX_DELAY);
					 printf("\nNew lower frequency threshold configured\n");

					 UnderFreqValue = UnderFreq[0]*10 + UnderFreq[1];

					 printf("Value is: %f\n",UnderFreqValue);
					 printf("Enter 2dp double value for maximum rate of change ( ie; 1.56): \n");
					 CMaxROCFlag = 1;
					 CUnderFreqFlag = 0;
					 i = 0;
					 xSemaphoreGive(under_freq_sem);
				 }
				 else if (CMaxROCFlag == 1){
					 xSemaphoreTake(max_ROC_sem,portMAX_DELAY);
					 printf("\nNew rate of change threshold value configured\n");
					 MaxROCValue = MaxROC[0] + (MaxROC[1]/10.0) + (MaxROC[2]/100.0);
					 printf("Value is: %f\n",MaxROCValue);
					 CMaxROCFlag = 0;
					 i = 0;
					 xSemaphoreGive(max_ROC_sem);
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
	while(1)
	{

	}
}

void load_management_task (void *pvParameters)
{
	while(1)
	{

	}
}

void record_display_task (void *pvParameters)
{
	while(1)
	{

	}
}



int main()
{
  printf("Hello from Nios II!\n");

  initCreateTasks();
  initOSDataStructs();
  vTaskStartScheduler();
  for (;;);
  return 0;
}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	ConfigInputQueue = xQueueCreate( CONFIG_INPUT_QUEUE_SIZE, sizeof( int ) );
	CurrentFreqQueue = xQueueCreate(CURRENT_FREQ_QUEUE_SIZE, sizeof(int));
	//shared_resource_sem = xSemaphoreCreateCounting( 9999, 1 );
	//printf("yooo");
	current_freq_sem = xSemaphoreCreateMutex();
	under_freq_sem = xSemaphoreCreateMutex();
	max_ROC_sem = xSemaphoreCreateMutex();
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(store_frequency_task,"store_frequency_task",TASK_STACKSIZE,NULL,STORE_FREQUENCY_PRIORITY,NULL);
	//xTaskCreate(system_status_task,"system_status_task",TASK_STACKSIZE,NULL,SYSTEM_STATUS_PRIORITY,NULL);
	xTaskCreate(configuration_input_task,"configuration_input_task",TASK_STACKSIZE,NULL,CONFIG_INPUT_PRIORITY,NULL);
	//xTaskCreate(switch_status_task,"switch_status_task",TASK_STACKSIZE,NULL,SWITCH_STATUS_PRIORITY,NULL);
	//xTaskCreate(load_management_task,"load_management_task",TASK_STACKSIZE,NULL,LOAD_MANAGEMENT_PRIORITY,NULL);
	//xTaskCreate(record_display_task,"record_display_task",TASK_STACKSIZE,NULL,RECORD_DISPLAY_PRIORITY,NULL);
	//printf("hmmm");
	return 0;
}


