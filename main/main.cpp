/*
*DS-S009 Broadcast Example
*ESP-IDF v5.4.4
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DSServo.h"

#define SERVO_UART_NUM UART_NUM_1
#define SERVO_BAUDRATE 1000000
#define SERVO_TX_PIN 20
#define SERVO_RX_PIN 21

DSServo ds_servo;

extern "C" void app_main(void)
{
    //Intialize the Serial Port
     ds_servo.begin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN, SERVO_BAUDRATE);
     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the serial port to stabilize

    //Broadcast torque enable for all servos
    ds_servo.setTorque(DS_BROADCAST_ID, true);
     vTaskDelay(pdMS_TO_TICKS(500)); // Wait for the command to take effect

     while(1){
        //Broadcast: all servos move to position 4095 in 1 second (360 degree)
        ds_servo.setPosition(DS_BROADCAST_ID, 4095, 1000);
         vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for the movement to complete

         //Broadcast: all servos move back to position 0 in 1 second (0 degree)
        ds_servo.setPosition(DS_BROADCAST_ID, 0, 1000);
         vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for the movement to complete
     }
}