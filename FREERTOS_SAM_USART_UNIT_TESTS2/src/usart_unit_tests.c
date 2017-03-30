#include <stdint.h>
#include <stdbool.h>
#include <board.h>
#include <sysclk.h>
#include <string.h>
#include <unit_test/suite.h>
#include <stdio_serial.h>
#include <usart.h>
#include <conf_test.h>
#include <conf_board.h>
#include <conf_example.h>
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_usart_serial.h"
#include "packet.h"

/**
 *
 * Introduction
 * This is the Robot Real Time code. It receives commands from
 * the serial port and executes those commands.
 *
 * files Main Files
 *  usart_unit_tests.c
 *  FreeRTOSConfig.h
 *  conf_board.h
 *  conf_clock.h
 *  conf_example.h
 *  conf_test.h
 *  conf_usart_serial.h
 *
 *  device_info Device Info
 * Arduino Due is used.
 *
 */

/*
* PACKET format:
*  2 BYTE 1 BYTE	1 BYTE	  	UP TO 80	1 BYTE	1 BYTE
*
* TYPE	PKT_LEN 	MSG_ID		DATA 	PKT_NUM	NUM_PACKETS
* See packet.h
*/

/* The baud rate to use. */
#define USART_BAUD_RATE         (115200)
/*-----------------------------------------------------------*/

static void create_usart_tasks(Usart *usart_base, uint16_t stack_depth_words,
		unsigned portBASE_TYPE task_priority);
static void usart_task(void *pvParameters);
static void run_usart_test(const struct test_case *test);
static void usart_echo_tx_task(void *pvParameters);

/*-----------------------------------------------------------*/

/* The buffer provided to the USART driver to store incoming character in. */
static uint8_t receive_buffer[PKT_BUFFER_SIZE] = {0};
/* USART instance */
static freertos_usart_if freertos_usart;

/*
 * FreeRTOS hook (or callback) functions that are defined in this file.
 */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask,	signed char *pcTaskName);
void vApplicationTickHook(void);

static uint8_t TX_buffer[PKT_BUFFER_SIZE];
static uint8_t rx_buffer[PKT_BUFFER_SIZE];
xQueueHandle txQueue; // queue to use for sending data to the USART tx task.
xQueueHandle rxQueue;
xQueueHandle sevoQueue;
xQueueHandle DC_MotorQueue;
xQueueHandle LidarQueue;
xTaskHandle xComTaskHandle;
/**
 *  Run USART unit tests
 */
int main(void)
{
	/* Prepare the hardware to run this demo. */
	prvSetupHardware();
	
	/* The queue is created to hold a maximum of 5 values, each of which is
	large enough to hold a variable of type packet. */
	txQueue = xQueueCreate( 4, sizeof( packet_t ) );
	if(txQueue == NULL)
	{
		while(1);	// freeze because the queue init did not work
	}
	
	rxQueue = xQueueCreate( 4, sizeof( packet_t ) );
	if(rxQueue == NULL)
	{
		while(1);	// freeze because the queue init did not work
	}	
	
	/* Create the test task. */
	create_usart_tasks(BOARD_USART, 1024, tskIDLE_PRIORITY);
	/* Create the communications task */
	xTaskCreate(Com_task, (const signed char *const) "Communication Task", 1024, NULL, 1, &xComTaskHandle);

	/* Start the RTOS scheduler. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for (;;) {
	}
}

static void create_usart_tasks(Usart *usart_base, uint16_t stack_depth_words,
		unsigned portBASE_TYPE task_priority)
{
	freertos_peripheral_options_t driver_options = {
		receive_buffer,									/* The buffer used internally by the USART driver to store incoming characters. */
		PKT_BUFFER_SIZE,									/* The size of the buffer provided to the USART driver to store incoming characters. */
		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,	/* The priority used by the USART interrupts. */
		USART_RS232,									/* Configure the USART for RS232 operation. */
		(USE_TX_ACCESS_SEM | USE_RX_ACCESS_MUTEX)
	};

	const sam_usart_opt_t usart_settings = {
		USART_BAUD_RATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		0 /* Only used in IrDA mode. */
	}; /*_RB_ TODO This is not SAM specific, not a good thing. */

	/* Initialise the USART interface. */
	freertos_usart = freertos_usart_serial_init(usart_base, &usart_settings, &driver_options);
	configASSERT(freertos_usart);

	/* Enable USART normal mode. */
	usart_base->US_MR = (usart_base->US_MR & ~US_MR_CHMODE_Msk) | US_MR_CHMODE_NORMAL;

	/* Create the two tasks as described above. */
	xTaskCreate(usart_tx_task, (const signed char *const) "Tx", stack_depth_words, (void *) freertos_usart,
			task_priority,
			NULL);
	xTaskCreate(usart_rx_task, (const signed char *const) "Rx", stack_depth_words, (void *) freertos_usart,
			task_priority + 1, NULL);
}

/******************************************************************
* usart_rx_task recieves packets of information on the serial
* port from the Master controller of the robot. It then
* passes the data to the command parser(Com_RXtask).
******************************************************************/
static void usart_rx_task(void *pvParameters)
{
	UNUSED(pvParameters);
	uint32_t received;
	portBASE_TYPE rxStatus;
	
	for (;;)
	{
		memset(rx_buffer, 0x00, sizeof(rx_buffer));
		received = freertos_usart_serial_read_packet(freertos_usart, rx_buffer, PACKET_SIZE, portMAX_DELAY);
		if(received != PACKET_SIZE)
			log_error(USART_RX);
		rxStatus = xQueueSendToBack( rxQueue, rx_buffer, 0 ); // send to Com_RXtask for parsing
		if( xStatus != pdPASS )
		{
			while(1);
		}
	}
}

/******************************************************************
* usart_tx_task() 
* This task recieves character data in a queue and transmits it.
*******************************************************************/

static void usart_tx_task(void *pvParameters)
{
	UNUSED(pvParameters);
	
	const portTickType xTicksToWait = portMAX_DELAY; // wait for ever
	portBASE_TYPE xStatus;
	
	const portTickType time_out_definition = (100UL / portTICK_RATE_MS),
			short_delay = (10UL / portTICK_RATE_MS);
			
		/* TX notification sem */
	xSemaphoreHandle TXnotification_semaphore;
	
		/* Create the semaphore to be used to get notified of end of
	transmissions. */
	vSemaphoreCreateBinary(TXnotification_semaphore);
	configASSERT(notification_semaphore);
	
	for(;;)
	{
		xStatus = xQueueReceive( txQueue, TX_buffer, xTicksToWait );
		if( xStatus == pdPASS )
		{
			/* Start with the semaphore in the expected state - no data has been sent
			yet.  A block time of zero is used as the semaphore is guaranteed to be
			as it has only just been created. */
			xSemaphoreTake(TXnotification_semaphore, 0);
			/* Start send. */
			freertos_usart_write_packet_async(freertos_usart, TX_buffer, strlen((char *) TX_buffer),
				time_out_definition, TXnotification_semaphore);
			configASSERT(returned_status == STATUS_OK);
			/* The async version of the write function is being used, so wait for
			the end of the transmission.  No CPU time is used while waiting for the
			semaphore.*/
			xSemaphoreTake(TXnotification_semaphore, time_out_definition * 2);
		}
	}

}

	/*******************************************************************
	* Communications TX task: This task is the only place in the code that
	* controls outgoing messages to the Linux master board. It gets 
	* data from the tasks and sends them to the transmit queue.
	* It functions as follows: Various tasks send data to be sent 
	* to the master via a queue. Com_RXtask() creates a packet and
	* places that packet into the transmit queue to be sent to the 
	* Master.
	*******************************************************************/
static void Com_RXtask(void *pvParameters)
{
	UNUSED(pvParameters);
	portBASE_TYPE txStatus;
	static uint8_t task_resp_buffer[TASK_BUFFER_SIZE];
	static uint8_t rcvd_buffer[PKT_BUFFER_SIZE]
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
	packet_t *master_pkt_buffer; // packets going to master
	// Right here: figure out how to give each message an ID
	master_pkt_buffer = (packet_t *)pvPortMalloc(sizeof(packet_t));
	// TODO: check for failure
	
	if(master_pkt_buffer != NULL) // if the alloc passed
	{
		for( ; ; )
		{
			/********************************************************************
			* get the data from the usart_rx_task() sent on queue 
			********************************************************************/
			if(uxQueueMessagesWaiting(rxQueue)) // first make sure something is there
			{
				xStatus = xQueueReceive(rxQueue, rcvd_buffer, 0);
				if( xStatus == pdPASS )
				{
					uint8_t type = rcvd_buffer[0]; // has typeh byte
						// which task is it for?
					uint8_t which = find_task(type);
					if(which != NULL)
					{
						// send the packet to the right task
						send_packet_to_task(which, rcvd_buffer);
					}
				}
				else
					log_error(COM_QUE_ERR);
			}
			/********************************************************************
			* Block on the task_out queue for data from tasks to be sent to the
			* Master. 
			********************************************************************/
			xStatus = xQueueReceive( Task_outQueue, task_resp_buffer, xTicksToWait );			
			/********************************************************************
			* The task_resp_buffer will have been filled in by the task.
			********************************************************************/
			// make_packet figure out how to make the pkts from task data in buffer
			/********************************************************************
			* Transmit it
			********************************************************************/
			txStatus = xQueueSendToBack(txQueue)
		}
	}
}


/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	const usart_serial_options_t usart_serial_options = {
		.baudrate   = CONF_TEST_BAUDRATE,
		.paritytype = CONF_TEST_PARITY
	};

	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(0);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

	/* Enable unit test output peripheral. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_USART, &usart_serial_options);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask,
		signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}

	/*******************************************************************
	* create a packet to be transmitted. info is fron a task. *buffer
	* is where to place the packet.
	*******************************************************************/
void make_packet(unsigned char *info, packet_t *buffer)
{
	buffer->typeh = *info++;
	buffer->typel = *info++;
	
	buffer->data = *info;
	
	
}

void log_error(uint8_t err_num)
{
	
}

uint8_t find_task(uint8_t type)
{
	switch(type)
	{
		case	SERVO_MV:
			return(SERVO_TASK);
			break;
		case	DC_MOTOR:
			return(DC_MOTOR_TASK);
			break;
		case	LIDAR:
			return(LIDAR_TASK);
			break;
		default:
			return(NULL);
	}
}
	/*******************************************************************
	* based upon num, send the packet to the correct task
	*******************************************************************/
void send_packet_to_task(uint8_t  num, uint8_t *buffer)
{
	portBASE_TYPE Qstatus;
	
	// first create the task queues
	sevoQueue = xQueueCreate(1, sizeof( packet_t ));
	if(sevoQueue == NULL)
	{
		while(1);
	}
	
	DC_MotorQueue = xQueueCreate(1, sizeof( packet_t ));
	if(DC_MotorQueue == NULL)
	{
		while(1);
	}
	LidarQueue = xQueueCreate(1, sizeof( packet_t ));
	if(LidarQueue == NULL)
	{
		while(1);
	}
	
	switch(num)
	{
		case	SERVO_TASK:
					Qstatus = xQueueSendToBack(sevoQueue , buffer, 0 );
					if( QStatus != pdPASS )
					{
						while(1);
					}
					break;
		case	DC_MOTOR_TASK:
					Qstatus = xQueueSendToBack(DC_MotorQueue , buffer, 0 );
					if( QStatus != pdPASS )
					{
						while(1);
					}
					break;
		case	LIDAR_TASK:
				Qstatus = xQueueSendToBack(LidarQueue , buffer, 0 );
				if( QStatus != pdPASS )
					{
						while(1);
					}
					break;
				default:
					break;
	}
}
