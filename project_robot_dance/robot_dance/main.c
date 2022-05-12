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

#include "choreography.h"

#define CHUNK_SIZE 	1024
#define WINDOW_SIZE 64
#define DATA_TO_COMPUTER
#define DATA_TO_COMPUTER3

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
#ifdef DATA_TO_COMPUTER1
static float send_tab[CHUNK_SIZE/2];
#endif
#ifdef DATA_TO_COMPUTER2
static float send_rms[WINDOW_SIZE];
#endif
#ifdef DATA_TO_COMPUTER3
static float send_cor[2*WINDOW_SIZE];
#endif



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
	choreography_init();

	//chprintf((BaseSequentialStream *)&SD3, "===================================================================\n");
	/* Main loop */


	while(1){
		#ifdef DATA_TO_COMPUTER
		wait_send_to_computer();
		#endif
		// uint16_t interval = get_music_interval();
		// uint16_t tempo = get_music_tempo();
		//chprintf((BaseSequentialStream *)&SD3, " interval:%d, tempo: %d \n", interval, tempo);
		#ifdef DATA_TO_COMPUTER1
		arm_copy_f32(get_audio_buffer_ptr(), send_tab, CHUNK_SIZE/2);
		SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, CHUNK_SIZE/2);
		#endif
		#ifdef DATA_TO_COMPUTER2
		arm_copy_f32(get_rms_frequencies(), send_rms, WINDOW_SIZE);
		SendFloatToComputer((BaseSequentialStream *) &SD3, send_rms, WINDOW_SIZE);
		#endif
		#ifdef DATA_TO_COMPUTER3
		arm_copy_f32(get_auto_correlation(), send_cor, 2*WINDOW_SIZE);
		SendFloatToComputer((BaseSequentialStream *) &SD3, send_cor, 2*WINDOW_SIZE);
		#endif
		//chThdSleepMilliseconds(1000);
	}
}
#ifdef DATA_TO_COMPUTER
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size)
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}
#endif


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
