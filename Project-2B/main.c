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
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"

/*Defining the bus process*/
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*Functions Initialisations*/
void ret_prox(int *prox);
void motor_control(int *prox, int sensor);
void led_switch(int *prox);
void follow(int *prox);
void sound_control(void);

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
    VL53L0X_start();
    dac_start();
    playSoundFileStart();

/*Variable initialisation*/
    int proxs[8];
	int con;
	char str[100];
	int str_length;
    /*Infinite loop*/
    while (1) {
        /*Retrieve the proximity information*/
        con = get_selector();
    	if(con == 0){
    	    left_motor_set_speed(800);
    		right_motor_set_speed(800);
            ret_prox(proxs);
        }
        else if(con == 1){
    	    left_motor_set_speed(0);
    		right_motor_set_speed(0);
            follow(proxs);
        }
    }
}

void follow(int *prox){

	char str[100];
	int str_length;
	int close = 0;

    int set_speed = 800;
	int dist =  VL53L0X_get_dist_mm();

	for(int i = 0; i<8; i++){
		prox[i] = get_calibrated_prox(i);
		if(prox[i] >= 800){
			close = 1;
		}
	}

    led_switch(prox);

    if(!close){
    	if(dist >= 20 && dist <= 150){
    			if(dist > 50){
    				left_motor_set_speed(set_speed );
    				right_motor_set_speed(set_speed );
    				while(1){
    					dist = VL53L0X_get_dist_mm();
    					if(dist < 65 || dist > 200){
    						break;
    					}
    				}
    				left_motor_set_speed(0);
    				right_motor_set_speed(0);
    			}
    		}
    }

    if(close){
		int rT = prox[1]+ prox[2] + prox[3];
		int lT = prox[4] + prox[5] + prox[6];
		int check;

		if(rT/4 > prox[0] +100){
			int check = prox[0];
			int count = 0;
			left_motor_set_speed(set_speed);
			right_motor_set_speed(-set_speed);
			int x;
			while(1){
				check = get_calibrated_prox(7);
				for(int i =0; i<8; i++){
					x = get_calibrated_prox(i);
					if(x < 800){
						count++;
					}
				}
				if(check > 800|| count == 8){
					break;
				}
			}
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
		else if(lT/4 > prox[7] +100){
			int check = prox[7];
			int count = 0;
			left_motor_set_speed(-set_speed);
			right_motor_set_speed(set_speed);
			int x;
			while(1){
				check = get_calibrated_prox(0);
				for(int i =0; i<8; i++){
					x = get_calibrated_prox(i);
					if(x < 800){
						count++;
					}
				}
				if(check > 800 || count == 8){
					break;
				}
			}
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
    }

}



void ret_prox(int *prox){
     /*
    Fuction that returns the readings to each proximity sensor
    INPUT - Pointer to an array
    OUTPUT - None*/
    
	int obj = 0;
	int prob = -1;
	int value;
    for(int i =0; i<8; i++){
    	value = get_calibrated_prox(i);
    	if(value >= 50){
    	    prox[i] = value;
    	}
    	else{
    		prox[i] = 0;
    	}

        if(prox[i] >= 900 && (i != 2 || i != 5)){
            obj = 1;
            if((prox[i] > prob || prob == -1)){
                prob = i;
            }
        }
        
    }   

    /*Should fix the LED*/
    led_switch(prox);
    
    /*if moving this doesn't fix the smoothness, try error count*/
    if(obj){
        /*sound_control();*/
        motor_control(prox, prob);
    }
}

void motor_control(int *prox, int sensor){
    /*
    Fuction to control the motors of the
    INPUT
    proxs - state the robot is in
    sensor - which sensor is reading the highest value
    OUTPUT - None*/
    
	int wan_speed = 13;
    int con_speed = wan_speed*1000/15.4;
    int epi;

    /*Initial stop*/
	left_motor_set_speed(0);
	right_motor_set_speed(0);

	int dir;
	int uTurn = 0;
    int lT;
    int rT;

    /*Set averages*/

    rT = prox[0] + prox[1] + prox[2] + prox[3];
    lT = prox[4] + prox[5] + prox[6] + prox[7];

    /*Logic for the direction of turn*/
    if(rT/4 >= 250 && lT/4 >= 250){
        uTurn = 1;
        dir = 1;
    }
    else{
        if(rT < lT){
            dir = -1;
        }
        else if(lT < rT){
            dir = 1;
        }
        else{
            epi = rand()%100 + 1;
            if(epi <= 50){
                dir = 1;
            }
            else{
                dir = -1;
            }
        }
    }

    /*Initiate turn*/
    left_motor_set_speed(-dir*con_speed);
	right_motor_set_speed(dir*con_speed);

    /*Keep turning until the sensor that is the problem is no
    longer a problem*/
	int senA;
    int senB;
    while(1){
        if(uTurn){
            senA = get_calibrated_prox(0);
            senB = get_calibrated_prox(7);

            if((senA + senB)/2 <=50){
                break;
            }

        }
        else{
            senA = get_calibrated_prox(sensor);

            if(senA <= 850){
                break;
            }
        }
    }

    /*Resume forward motion*/
    left_motor_set_speed(con_speed);
	right_motor_set_speed(con_speed);

}
/*
void sound_control(void){
    playSoundFile("beep-04.mp3", option, 10000);
    waitSoundFileHasFinished()
}
*/
void led_switch(int *prox){

    int rgb[3][3] = {{0,10,0},{10,6,0},{10,0,0}};
    int red_val[] = {0,2,1};
    int sel;
    int j;
    led_name_t leds[] ={LED1, LED2, LED3, LED4, 
                        LED5, LED6, LED7, LED8};
    for(int i = 0; i<8; i++){
        sel = prox[i]/300;
        if(i == 0 || i == 7){
            set_led(leds[0],red_val[sel]);
        }
        else if(i != 3 && i !=4){
            j = i + i/5;
            if(i%2 == 0 || i%5 == 0){
                set_led(leds[j], red_val[sel]);
            }
            else{
                set_rgb_led(leds[j], rgb[sel][0],
                            rgb[sel][1],rgb[sel][2]);
            }
        }
        else{
            set_led(leds[4], red_val[sel]);
            if(i == 3){
                sel = (prox[i] + prox[i-1])/3;
                set_rgb_led(leds[i], rgb[sel][0],
                            rgb[sel][1],rgb[sel][2]);
            }
            else{
                sel = (prox[i] + prox[i+1])/3;
                set_rgb_led(leds[i+1], rgb[sel][0],
                            rgb[sel][1],rgb[sel][2]);
            }
        }
    } 
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

