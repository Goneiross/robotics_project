#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <msgbus/messagebus.h>

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
//static float send_tab[CHUNK_SIZE/2];
//static float send_rms[WINDOW_SIZE];
//static float send_cor[2*WINDOW_SIZE];


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
	usb_start();
  
	messagebus_init(&bus, &bus_lock, &bus_condvar);
	motors_init();
	detection_init();
  signals_processing_init();
	choreography_init();

	//chprintf((BaseSequentialStream *)&SD3, "===================================================================\n");
	/* Main loop */


	while(1){
		uint16_t amp = get_music_amplitude();
		chprintf((BaseSequentialStream *)&SD3, " amplitude:%d \n", amp);

		//arm_copy_f32(get_audio_buffer_ptr(), send_tab, CHUNK_SIZE/2);
		//arm_copy_f32(get_rms_frequencies(), send_rms, WINDOW_SIZE);
		//arm_copy_f32(get_auto_correlation(), send_cor, 2*WINDOW_SIZE);


		//SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, CHUNK_SIZE/2);
		//SendFloatToComputer((BaseSequentialStream *) &SD3, send_rms, WINDOW_SIZE);
		//SendFloatToComputer((BaseSequentialStream *) &SD3, send_cor, 2*WINDOW_SIZE);
	}
	while (1) {
		chThdSleepMilliseconds(1000);
	}
}

void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
