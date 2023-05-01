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

/*Variable initialisation - prox is the initialisation of the proximity sensor array and con is used to
control which strategy is being implemented*/
    int proxs[8];
	int con;

    /*Infinite loop*/
    while (1) {
        /*If the con value is 0, the first strategy is implemented and if it is 1, the second*/
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
    /*
    Function is used to control the following functionality of the Epuck
    Inputs: Prox - An array to store the proximity values
    Outputs: NONE
    */

    /*Initial values - close is used for checking if the object is close, set_speed keeps speed consistent*/
	int close = 0;
    int set_speed = 800;

    /*Find the distance using the distance sensor*/
	int dist =  VL53L0X_get_dist_mm();

    /*Find all the proximity values*/
	for(int i = 0; i<8; i++){
		prox[i] = get_calibrated_prox(i);
		if(prox[i] >= 800){
			close = 1;
		}
	}

    /*Activate the LEDs based on the proximity values*/
    led_switch(prox);

    /*If the object is not close, rely on the distance sensor to see if it appears*/
    if(!close){
    	if(dist >= 20 && dist <= 150){
    			if(dist > 50){
    				left_motor_set_speed(set_speed );
    				right_motor_set_speed(set_speed );
    				while(1){/*While loop to keep the movement going until the object is within range*/
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
    /*If it is close, rely on the proximity sensors for the controls*/
    if(close){
		int rT = prox[1]+ prox[2] + prox[3]; /*Using average values of the proximity sensors*/
		int lT = prox[4] + prox[5] + prox[6];
		int check;
        /*If statements to see if a left or right turn is nessesary to face the object
        If the averages to the right of the Epuck is greater than a threshold, turn right*/
		if(rT/4 > prox[0] +100){
			int check = prox[0];
			left_motor_set_speed(set_speed);
			right_motor_set_speed(-set_speed);
			while(1){/*While loop to keep it turning until the condition is met to break*/
				check = get_calibrated_prox(7);
				if(check > 800){
					break;
				}
			}
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
        /*If the averages of the left proximity sensors are more than a threshold turn left*/
		else if(lT/4 > prox[7] +100){
			int check = prox[7];
			left_motor_set_speed(-set_speed);
			right_motor_set_speed(set_speed);
			while(1){/*While loop to keep it turning until the condition is met to break*/
				check = get_calibrated_prox(0);
				if(check > 800){
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
    /*Finding each proximity value for the sensors. A threshold is added so that
    values that are too small will not be looked at. Of the 8 if any are a problem 
    an obj variable will be signalled starting motor control. The sensor with the highest value 
    will be only taken into consideration*/
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

    /*Control the LEDs based on proximity sensors*/
    led_switch(prox);
    
    /*If statement to respond whether motor control is needed*/
    if(obj){
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
    
    /*Initial conditions - wan_speed is the wanted speed in cm/s, con_speed is the control speed
    sent ot the Epuck, epi will be generated as a random value later, dir controls the direction of turn,
    uTurn is a condition checking if a uTurn is best, lT sums right sensors, and rT sums left sensors*/
	int wan_speed = 13;
    int con_speed = wan_speed*1000/15.4;
    int epi;
    int dir;
	int uTurn = 0;
    int lT;
    int rT;

    /*Initial stop to stop from any imidiate danger and start controlling the Epuck*/
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	
    /*Set averages - checking whether the right or the left side is has any closer obsticles to choose the
    best way to turn*/
    rT = prox[0] + prox[1] + prox[2] + prox[3];
    lT = prox[4] + prox[5] + prox[6] + prox[7];

    /*Logic for the direction of turn - if there is more to the left of the Epuck, turn right and vice
    versa. If both left and right have too much, then a uTurn is prefered*/
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
        else{/*If a left or right turn could both work, then the direction is randomised*/
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
    longer a problem - If a uTurn is initiated then checking until the front sensors are no longer
    having any problems. If a normal turn is initialted then keep turning until the problem sensor 
    no longer shows that anything is too close*/
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

void led_switch(int *prox){
    /*Function used to control the LED sensors, based on the proximity sensors
    INPUT: prox - an array containing the proximity values
    OUTPUT: NONE*/

    /*Initial values - rgb is an array to help with color control of the rgb LEDS, based on proximity values,
    red_val is an array to help with the control of the red LEDS, based on proximity values, sel is a selector
    for the control of the LEDS, j is a variable to help with picking the right LED for each sensor, and leds
    sotres the call for each LED to make it easier when looping through each proximity value*/
    int rgb[3][3] = {{0,10,0},{10,6,0},{10,0,0}};
    int red_val[] = {0,2,1};
    int sel;
    int j;
    led_name_t leds[] ={LED1, LED2, LED3, LED4, 
                        LED5, LED6, LED7, LED8};
    for(int i = 0; i<8; i++){
        sel = prox[i]/300;
        if(i == 0 || i == 7){ /*Control for front LED*/
            set_led(leds[0],red_val[sel]);
        }
        else if(i != 3 && i !=4){
            j = i + i/5;
            if(i%2 == 0 || i%5 == 0){/*Control for left and right red LEDs*/
                set_led(leds[j], red_val[sel]);
            }
            else{/*control for the front two rgb LEDS*/
                set_rgb_led(leds[j], rgb[sel][0],
                            rgb[sel][1],rgb[sel][2]);
            }
        }
        else{/*Control for the back LEDs*/
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

