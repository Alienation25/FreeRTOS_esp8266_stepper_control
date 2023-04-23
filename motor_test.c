/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "stepper_control.h"





void pinMode(int number_pin,int gpio_mode)
{
    gpio_config(& (gpio_config_t) {
            .pin_bit_mask = (1ULL << number_pin),
            .mode = gpio_mode,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        });
    
}



void set_uart_speed(int baudrate)
{
    int uart_num = UART_NUM_0; // используем UART0
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_baudrate(uart_num, baudrate);
}





void task(void *pvParameters)
{
    while (1) {
    TickType_t start_time = xTaskGetTickCount();
    int start_counter = esp_timer_get_time();

    //

    int end_counter = esp_timer_get_time();
    TickType_t end_time = xTaskGetTickCount();
    printf("Task execution time: %d microseconds\n", end_counter - start_counter);
    printf("Task start_counter time: %d microseconds\n", start_counter);
    printf("Task end_counter time: %d microseconds\n", end_counter);
    printf("Task execution time: %d ticks\n", end_time - start_time);
    }

}

void task2(void *pvParameters)
{

    struct stepper_control my_object;
    MyClass_Init(&my_object, 0, 1,2,120,2000,1000,16,200,0);
    begin_pin(&my_object);
    while (1) {
        enable(&my_object,0);
        move(&my_object,-my_object.motor_steps*my_object.microsteps);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        move(&my_object,my_object.motor_steps*my_object.microsteps);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        enable(&my_object,1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    //vTaskDelete(NULL);
}







void app_main()
{

    //set_uart_speed(115200); 
    //xTaskCreate(task, "task", 2048, NULL, 1, NULL);
    xTaskCreate(task2, "task2", 2048, NULL, 1, NULL);
    
   
}

