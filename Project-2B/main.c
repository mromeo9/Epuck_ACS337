#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "motors.h"
#include "selector.h"
#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"

/*Defining the bus process*/
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*Functions Initialisations*/
void ret_prox(int *prox);
void motor_control(int *prox, int sensor);
void start(void);

/*Main function*/
int main(void)
{
/*Bus initialisation*/
    messagebus_init(&bus,&bus_lock, &bus_condvar);

/*Pre function initialisations*/
    halInit();
    chSysInit();
    mpu_init();
    motors_init();
    serial_start();
    proximity_start(0);
    calibrate_ir();
    clear_leds();
    spi_comm_start();

/*Variable initialisation*/
    int proxs[8];

    /*Infinite loop*/
    while (1) {
        /*Retrieve the proximity information*/
        start();
        ret_prox(proxs);
    }
}

void ret_prox(int *prox){
    /*
    Fuction that returns the readings to each proximity sensor
    INPUT - Pointer to an array
    OUTPUT - None
    */
	int obj = 0;
	int prob = NULL;
    for(int i =0; i<8; i++){
        prox[i] = get_calibrated_prox(i);
        If(prox[i] >= 1000 && (i ~= 2 || i ~= 5)){
            obj = 1;
            if((prox[i] > prob || prob == NULL)){
                prob = i;
            }

        if(obj == 1){
        	motor_control(proxs, prob);
        }
        }

    }
}

void motor_control(int *proxs, int sensor){
    /*
    Fuction to control the motors of the
    INPUT
    proxs - state the robot is in
    sensor - which sensor is reading the highest value
    OUTPUT - None
    */
	int wan_speed = 10;
    int con_speed = wan_speed*1000/15.4

    /*Initial stop*/
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	int dir;
	int uTurn = 0;

    /*Logic for the direction of turn*/
    if(proxs[2] >= 300 && proxs[5] >= 300){
        uTurn = 1;
        dir = 1;
    }
    else{
        if(proxs[2] < proxs[5]){
            dir = -1;
        }
        else if(proxs[5] < proxs[2]){
            dir = 1;
        }
        else{
            dir = 1;
        }
    }

    /*Initiate turn*/
    left_motor_set_speed(-dir*con_speed);
	right_motor_set_speed(dir*con_speed);

    /*Keep turning until the sensor that is the problem is no
    longer a problem*/
    while(1){
        if(proxs[sensor] <= 800){
            break;
        }
    }

    /*Resume forward motion*/
    left_motor_set_speed(con_speed);
	right_motor_set_speed(con_speed);

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

