/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/* Local Macros */
#define NUM_OF_TASKS	6

#define PERIOD_1		pdMS_TO_TICKS(50)
#define PERIOD_2		pdMS_TO_TICKS(50)
#define PERIOD_3		pdMS_TO_TICKS(100)
#define PERIOD_4		pdMS_TO_TICKS(20)
#define PERIOD_5		pdMS_TO_TICKS(10)
#define PERIOD_6		pdMS_TO_TICKS(100)

#define BUTTON1			PORT_1, PIN0
#define BUTTON2			PORT_1, PIN1

#define TASK1_TRACE		PORT_0, PIN0
#define TASK2_TRACE		PORT_0, PIN1
#define TASK3_TRACE		PORT_0, PIN2
#define TASK4_TRACE		PORT_0, PIN3
#define TASK5_TRACE		PORT_0, PIN4
#define TASK6_TRACE		PORT_0, PIN5
#define IDLE_TRACE		PORT_0, PIN6
#define TICK_TRACE		PORT_0, PIN7

#if(configUSE_TRACE_FACILITY == 1)
#define Trace_Port						PORT_0
#define CPU_LOAD_POLLING_RATE			pdMS_TO_TICKS(100)

static pinX_t Trace_Pins[NUM_OF_TASKS+1] = {PIN0, PIN1, PIN2, PIN3, PIN4, PIN5, PIN6};
static TickType_t IdleTime;
#endif

#if(configUSE_STATS_FORMATTING_FUNCTIONS == 1)
#define RUNTIME_STATS_POLLING_RATE		pdMS_TO_TICKS(1000)
#endif

typedef struct{
	char *Msg;
	int size;
} Q_Msg;
/*-----------------------------------------------------------*/

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/* Tasks Prototypes */
void Button_1_Monitor 	(void *pvParameters);
void Button_2_Monitor 	(void *pvParameters);
void PeriodicTransmitter(void *pvParameters);
void UART_Receiver 		(void *pvParameters);
void Load_1_Simulation 	(void *pvParameters);
void Load_2_Simulation	(void *pvParameters);
/*-----------------------------------------------------------*/

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	QueueHandle_t Consumer_q;
	
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
#if(configUSE_EDF_SCHEDULER == 1)
	/* Create Comsumer buffer */
	Consumer_q = xQueueCreate(3, sizeof(Q_Msg *));
	
	/* Reset Trace GPIO PINS */
	
	GPIO_write(TASK1_TRACE, PIN_IS_LOW);
	GPIO_write(TASK2_TRACE, PIN_IS_LOW);
	GPIO_write(TASK3_TRACE, PIN_IS_LOW);
	GPIO_write(TASK4_TRACE, PIN_IS_LOW);
	GPIO_write(TASK5_TRACE, PIN_IS_LOW);
	GPIO_write(TASK6_TRACE, PIN_IS_LOW);
	GPIO_write(IDLE_TRACE, PIN_IS_LOW);
	GPIO_write(TICK_TRACE, PIN_IS_LOW);
	
    /* Create Tasks here */
	
		xTaskPeriodicCreate(Button_1_Monitor, 
							"Button_1_Monitor", 
							configMINIMAL_STACK_SIZE, 
							(void *)&Consumer_q, 
							1, 
							NULL,
							PERIOD_1);
		
		xTaskPeriodicCreate(Button_2_Monitor, 
							"Button_2_Monitor", 
							configMINIMAL_STACK_SIZE, 
							(void *)&Consumer_q, 
							1, 
							NULL,
							PERIOD_2);
							
		xTaskPeriodicCreate(PeriodicTransmitter, 
							"PeriodicTransmitter", 
							configMINIMAL_STACK_SIZE, 
							(void *)&Consumer_q, 
							1, 
							NULL,
							PERIOD_3);
							
		xTaskPeriodicCreate(UART_Receiver, 
							"UART_Receiver", 
							configMINIMAL_STACK_SIZE, 
							(void *)&Consumer_q,
							1, 
							NULL,
							PERIOD_4);
							
		xTaskPeriodicCreate(Load_1_Simulation, 
							"Load_1_Simulation", 
							configMINIMAL_STACK_SIZE, 
							NULL, 
							1, 
							NULL,
							PERIOD_5);
							
		xTaskPeriodicCreate(Load_2_Simulation, 
							"Load_2_Simulation", 
							configMINIMAL_STACK_SIZE + (((NUM_OF_TASKS+1) * 40 * (configUSE_STATS_FORMATTING_FUNCTIONS && 1)) / sizeof(size_t)),
							NULL, 
							1, 
							NULL,
							PERIOD_6);
#endif
	
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/
/* Create Tasks Here */
#if(configUSE_EDF_SCHEDULER == 1)
/* Task 1 */
void Button_1_Monitor(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = PERIOD_1;
	
	const QueueHandle_t Consumer_q = *((QueueHandle_t *)pvParameters);
	
	pinState_t Status;
	static pinState_t oldStatus = PIN_IS_HIGH;
	
	Q_Msg *Msg_Ptr[2];
	Q_Msg Msg0 = {"Button 1 Falling\n", 17};
	Q_Msg Msg1 = {"Button 1 Rising\n" , 16};
	
	Msg_Ptr[0] = &Msg0;
	Msg_Ptr[1] = &Msg1;
	
	/* Super loop */
	while(1){		
		/* Check Button 1 Status */
		Status = GPIO_read(BUTTON1);
		if(Status != oldStatus){
			/* Send respective msg to the consumer queue */
			if(Consumer_q != NULL){
				xQueueSend(Consumer_q, (void *)&Msg_Ptr[Status], 0);
			}
			oldStatus = Status;
		}
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		xLastWakeTime = xTaskGetTickCount();	
	}
}
/*-----------------------------------------------------------*/

/* Task 2 */
void Button_2_Monitor(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = PERIOD_2;
	
	const QueueHandle_t Consumer_q = *((QueueHandle_t *)pvParameters);
	
	pinState_t Status;
	static pinState_t oldStatus = PIN_IS_HIGH;
	
	Q_Msg *Msg_Ptr[2];
	Q_Msg Msg0 = {"Button 2 Falling\n", 17};
	Q_Msg Msg1 = {"Button 2 Rising\n" , 16};
	
	Msg_Ptr[0] = &Msg0;
	Msg_Ptr[1] = &Msg1;
		
	/* Super loop */
	while(1){
		/* Check Button 2 Status */
		Status = GPIO_read(BUTTON2);
		if(Status != oldStatus){
			/* Send respective msg to the consumer queue */
			if(Consumer_q != NULL){
				xQueueSend(Consumer_q, (void *)&Msg_Ptr[Status], 0);
			}
			oldStatus = Status;
		}
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		xLastWakeTime = xTaskGetTickCount();
	}
}
/*-----------------------------------------------------------*/

/* Task 3 */
void PeriodicTransmitter(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = PERIOD_3;
	
	const QueueHandle_t Consumer_q = *((QueueHandle_t *)pvParameters);
	
	Q_Msg Msg = {"Periodic Check\n", 15};
	Q_Msg *Msg_ptr = &Msg;
		
	/* Super loop */
	while(1){
		/* Send respective msg to the consumer queue */
		if(Consumer_q != NULL){
			xQueueSend(Consumer_q, (void *)&Msg_ptr, 0);
		}
		
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		xLastWakeTime = xTaskGetTickCount();
	}
}
/*-----------------------------------------------------------*/

/* Task 4 */
void UART_Receiver(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = PERIOD_4;
	
	const QueueHandle_t Consumer_q = *((QueueHandle_t *)pvParameters);
	Q_Msg *Msg_Buffer;
	
	/* Super loop */
	while(1){
		if(Consumer_q != NULL){
			/* Read a Msg from queue then send it on UART */
			if(xQueueReceive(Consumer_q, (void *)&Msg_Buffer, 0) == pdTRUE){
				while(vSerialPutString(Msg_Buffer->Msg, Msg_Buffer->size) != pdTRUE);
			}
		}
	
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		xLastWakeTime = xTaskGetTickCount();
	}
}
/*-----------------------------------------------------------*/

/* Task 5 */
void Load_1_Simulation(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = PERIOD_5;
	
	int i;
	
	/* Super loop */
	while(1){
		/* Loop for 5ms approx. */
		for(i=0; i<37000; i++);
			
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		xLastWakeTime = xTaskGetTickCount();
	}
}
/*-----------------------------------------------------------*/

/* Task 6 */
void Load_2_Simulation(void *pvParameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = PERIOD_6;
	int i;
	
#if(configUSE_TRACE_FACILITY == 1)
	TickType_t CurrentTime;
	static TickType_t LastCheckTime;
	static uint16_t CPU_Load;
	static uint8_t count;
#endif
	
	/* Super loop */
	while(1){
		/* Loop for 12ms approx. */
		for(i=0; i<90000; i++);
			
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		xLastWakeTime = xTaskGetTickCount();

#if(configUSE_TRACE_FACILITY == 1)		
		count++;
		if((count % (CPU_LOAD_POLLING_RATE / PERIOD_6)) == 0){
			CurrentTime = T1TC;
			CPU_Load = 100 - (IdleTime * 100 / (CurrentTime - LastCheckTime));
			IdleTime = 0;
			LastCheckTime = CurrentTime;
		}
		
	#if(configUSE_STATS_FORMATTING_FUNCTIONS == 1)
		if((count % (RUNTIME_STATS_POLLING_RATE / PERIOD_6)) == 0){
			char logs[40*(NUM_OF_TASKS+1)];
			char *s;
			int i;
			
			vTaskGetRunTimeStats(logs);
			
			s = logs;
			i = 0;
			while(*s++){
				i++;
			}
			
			while(vSerialPutString(logs, i) == pdFALSE);
			count = 0;
		}
	#endif
#endif
	}
}
/*-----------------------------------------------------------*/
#endif

/* IDLE Hook */
#if(configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook( void ){
	GPIO_write(TASK1_TRACE, PIN_IS_LOW);
	GPIO_write(TASK2_TRACE, PIN_IS_LOW);
	GPIO_write(TASK3_TRACE, PIN_IS_LOW);
	GPIO_write(TASK4_TRACE, PIN_IS_LOW);
	GPIO_write(TASK5_TRACE, PIN_IS_LOW);
	GPIO_write(TASK6_TRACE, PIN_IS_LOW);
	GPIO_write(IDLE_TRACE, PIN_IS_HIGH);
}
#endif
/*-----------------------------------------------------------*/

/* Tick Hook */
#if(configUSE_TICK_HOOK == 1)
void vApplicationTickHook( void ){
	GPIO_write(TICK_TRACE, PIN_IS_HIGH);
	GPIO_write(TICK_TRACE, PIN_IS_LOW);
}
#endif
/*-----------------------------------------------------------*/

/* Context Switch Trace Function */
#if(configUSE_TRACE_FACILITY == 1)
void TaskTrace(UBaseType_t TaskNumber, uint8_t State){
	static TickType_t TimeStamp;
	TickType_t CurrentTime = T1TC;

	switch(State){
		case 0:
			GPIO_write(Trace_Port, Trace_Pins[TaskNumber], PIN_IS_LOW);
			if(TaskNumber == NUM_OF_TASKS){
				IdleTime += (CurrentTime - TimeStamp);
			}
		break;
		
		case 1:
			GPIO_write(Trace_Port, Trace_Pins[TaskNumber], PIN_IS_HIGH);
			if(TaskNumber == NUM_OF_TASKS){
				TimeStamp = CurrentTime;
			}
		break;
	}
}
#endif
/*-----------------------------------------------------------*/
