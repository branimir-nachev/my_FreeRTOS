/*
 * led_task.c
 *
 *  Created on: Aug 1, 2022
 *      Author: branimirnachev
 */

#include <global_include.h>
#include <utils/uartstdio.h>
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//*****************************************************************************
//
// The stack size for the LED toggle task.
//
//*****************************************************************************
#define LEDTASKSTACKSIZE        128         // Stack size in words

//*****************************************************************************
//
// The item size and queue size for the LED message queue.
//
//*****************************************************************************
#define LED_ITEM_SIZE           sizeof(uint8_t)
#define LED_QUEUE_SIZE          5

//*****************************************************************************
//
// Default LED toggle delay value. LED toggling frequency is twice this number.
//
//*****************************************************************************
#define LED_TOGGLE_DELAY_1        125
#define LED_TOGGLE_DELAY_2        250

//*****************************************************************************
//
// The queue that holds messages sent to the LED3 task.
//
//*****************************************************************************
xQueueHandle g_pLEDQueue;


extern xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void LEDTask1(void *pvParameters)
{
    portTickType ui32WakeTime;
    uint32_t ui32LEDToggleDelay;
    uint8_t ui8Message;

    //
    // Initialize the LED Toggle Delay to default value.
    //
    ui32LEDToggleDelay = LED_TOGGLE_DELAY_1;

    //
    // Get the current tick count.
    //
    ui32WakeTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {
        ui8Message = 0U;
        if(xQueueSend(g_pLEDQueue, &ui8Message, portMAX_DELAY) !=
           pdPASS)
        {
            //
            // Error. The queue should never be full. If so print the
            // error message on UART and wait for ever.
            //
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
            }
        }

        //
        // Turn on the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        //
        // Guard UART from concurrent access. Print the currently
        // blinking frequency.
        //
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("RED Led blinking frequency is %d ms.\n",
                   (ui32LEDToggleDelay * 2));
        xSemaphoreGive(g_pUARTSemaphore);

        //
        // Wait for the required amount of time.
        //
        vTaskDelayUntil(&ui32WakeTime, ui32LEDToggleDelay / portTICK_RATE_MS);

        //
        // Turn off the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

        //
        // Pass the value of the LED hide to LEDTask3.
        //
        ui8Message = 1U;
        if(xQueueSend(g_pLEDQueue, &ui8Message, portMAX_DELAY) !=
           pdPASS)
        {
            //
            // Error. The queue should never be full. If so print the
            // error message on UART and wait for ever.
            //
            UARTprintf("\nQueue full. This should never happen.\n");
            while(1)
            {
            }
        }
        //
        // Wait for the required amount of time.
        //
        vTaskDelayUntil(&ui32WakeTime, ui32LEDToggleDelay / portTICK_RATE_MS);
    }
}


//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void LEDTask2(void *pvParameters)
{
    portTickType ui32WakeTime;
    uint32_t ui32LEDToggleDelay;

    //
    // Initialize the LED Toggle Delay to default value.
    //
    ui32LEDToggleDelay = LED_TOGGLE_DELAY_2;

    //
    // Get the current tick count.
    //
    ui32WakeTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {

        //
        // Turn on the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Guard UART from concurrent access. Print the currently
        // blinking frequency.
        //
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("BLUE Led blinking frequency is %d ms.\n",
                   (ui32LEDToggleDelay * 2));
        xSemaphoreGive(g_pUARTSemaphore);

        //
        // Wait for the required amount of time.
        //
        vTaskDelayUntil(&ui32WakeTime, ui32LEDToggleDelay / portTICK_RATE_MS);

        //
        // Turn off the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        //
        // Wait for the required amount of time.
        //
        vTaskDelayUntil(&ui32WakeTime, ui32LEDToggleDelay / portTICK_RATE_MS);
    }
}

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency. User
// can make the selections by pressing the left and right buttons.
//
//*****************************************************************************
static void LEDTask3(void *pvParameters)
{
    uint8_t ui8Message;

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Read the next message, if available on queue.
        //
        if(xQueueReceive(g_pLEDQueue, &ui8Message, 0U) == pdPASS)
        {
            //
            // If left button, update to next LED.
            //
            if(ui8Message)
            {
                //
                // Turn on the LED.
                //
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

                //
                // Guard UART from concurrent access. Print the currently
                // blinking frequency.
                //
                xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
                UARTprintf("GREEN Led is blinking !\n");
                xSemaphoreGive(g_pUARTSemaphore);

            }
            else {
                //
                // Turn off the LED.
                //
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
            }
        }
    }
}


uint32_t led_task_init()
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Create a queue for sending messages to the LED task.
    //
    g_pLEDQueue = xQueueCreate(LED_QUEUE_SIZE, LED_ITEM_SIZE);

    //
    // Enable the GPIO pin for the LED (PG2).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Create the LED task.
    //
    if(xTaskCreate(LEDTask1, (const portCHAR *)"LED1", LEDTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_LED_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Create the LED task.
    //
    if(xTaskCreate(LEDTask2, (const portCHAR *)"LED2", LEDTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_LED_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Create the LED task.
    //
    if(xTaskCreate(LEDTask3, (const portCHAR *)"LED3", LEDTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_LED_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //

    return(0);

}
