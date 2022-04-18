#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//#include <messagebus.h>

#include <ch.h>
#include <hal.h>
#include "memory_protection.h"
#include <main.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>

#include "IR_detection.h"
#include "signals_processing.h"
#include "choreography.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);



static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
	/* System init */
	halInit();
	chSysInit();
	serial_start();
	motors_init();

	detection_init();
	choreography_init();
	/* Bus init */

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//chprintf((BaseSequentialStream *)&SD3, "===================================================================\n");
	/* Main loop */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
