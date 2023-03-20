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
#include "sensors/vl53l0x/vl53l0x.h"

/*Defining the bus process*/
message_t(bus);
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_codvar);

/*Functions Initialisations*/
void clear_leds(void);
void spi_comm_start(void);
void set_led(led_name_t led_number, unsigned int value);
void ret_prox(int *prox);
void motor_control(int state, int *p_rT);

/*Main function*/
int main(void)
{

/*Pre function initialisations*/
    halInit();
    chSysInit();
    mpu_init();
    motors_init();
    serial_start();
    usb_start();
    proximity_start();
    calibrate_ir();
    clear_leds();
    spi_comm_start();
    vl53l0x_start();

/*Variable initialisation*/
    int proxs[8];
    int rT = 0; /*turn count*/
    int state; /*State*/
    int *p_rT; /*pointer to rT*/
    p_rT = &rT;

/*Bus initialisation*/
    messagebus_init(&bus,&bus_lock, &bus_codvar);

    /* Infinite loop. */
    while (1) {
        /*Retrieve the proximity information*/
        ret_prox(proxs);

        /*If statement to control state*/
        if(){/*State 0 - Obsticle in front*/
            state = 0;
        }
        else if(){/*State 1-Obsticle in front and to right*/
            state = 1;
        }
        else if(){/*State 2-Obsticle in front and to left*/
            state = 2;
        }
        else{/*State 3 - Constant motion*/
            state = 3;
        }

        motor_control(state, p_rT);


    }
}

void ret_prox(int *prox){
    /*
    Fuction that returns the readings to each proximity sensor
    INPUT - Pointer to an array
    OUTPUT - None
    */
    for(int i =0; i<8; i++){
        prox[i] = get_calibrated_prox((unsigned int)i);
    }
}

void motor_control(int state, int *p_rT){
    /*
    Fuction to control the motors of the 
    INPUT
    State - state the robot is in 
    rT - Indicating what direction to turn
    OUTPUT - None
    */
    float size = 8;
    int wan_speed = 8; /*cm/s*/
    int cont_speed = wan_speed*1000/15.4;
    int dir;
    switch(state){
        case 0:
            left_motor_set_speed(0);
            right_motor_set_speed(0);
            if(*p_rT == 0){
                dir = 1;
                *p_rT = 1;
            }
            else{
                dir = -1;
                *p_rT = 0;
            }

            float rads = 0.2*cont_speed/size;
            float t = (math.pi/2)/rads;

            /*Adjust speeds*/
            left_motor_set_speed(dir*cont_speed);
            right_motor_set_speed(-dir*cont_speed);

            chThdSleepMilliseconds((int)t);

            left_motor_set_speed(cont_speed);
            right_motor_set_speed(cont_speed);     

            chThdSleepMilliseconds((int)(size/wan_speed));     

            left_motor_set_speed(dir*cont_speed);
            right_motor_set_speed(-dir*cont_speed);

            chThdSleepMilliseconds((int)t);

            left_motor_set_speed(cont_speed);
            right_motor_set_speed(cont_speed);   

            break;
        case 1:
            break;
        case 2:
            break;
        case 3:
            left_motor_set_speed(cont_speed);
            right_motor_set_speed(cont_speed);
            break;
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
