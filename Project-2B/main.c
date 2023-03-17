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


/*Functions Initialisations*/
void clear_leds(void);
void spi_comm_start(void);
void set_led(led_name_t led_number, unsigned int value);

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
	messagebus_init(&bus, &bus_lock, &bus_condvar);

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

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
    	int zero = get_prox(0);
    	if (SDU1.config->usbp->state == USB_ACTIVE) {
    	chprintf((BaseSequentialStream *)&SDU1, "%4d,", zero);
    	}

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
