#include <stdint.h>
#include <ch.h>
#include <hal.h>
#include <audio/microphone.h>
#include <chprintf.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#include "signals_processing.h"
#include "choreography.h"

static float mic_cmplx_input[2 * CHUNK_SIZE];
static float mic_output[CHUNK_SIZE];

static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

static THD_WORKING_AREA(waThdSignalsProcessing, 128);
static THD_FUNCTION(ThdSignalsProcessing, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void signals_processing_init(void){
	mic_start(&processAudioData);
}

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/
	int i = 0;
	static int wait10 = 0;
	static int iteration = 0;

	while((iteration < 2*CHUNK_SIZE) && (i < num_samples/4)){
			//micRight_cmplx_input[iteration] = data[4*i];
			mic_cmplx_input[iteration] = data[4*i+1];
			//micBack_cmplx_input[iteration] = data[4*i+2];
			//micFront_cmplx_input[iteration] = data[4*i+3];
			i++;
			iteration ++;
	}

	if(iteration >= CHUNK_SIZE*2){
		iteration = 0;

		doFFT_optimized(CHUNK_SIZE, mic_cmplx_input);
		arm_cmplx_mag_f32(mic_cmplx_input, mic_output, CHUNK_SIZE);

		if(wait10 == 10){
			wait10 = 0;
			chBSemSignal(&sendToComputer_sem);
		} else {
			wait10 ++;
		}
	}
}

float* get_audio_buffer_ptr(void){
		return mic_output;
}

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

uint16_t get_music_tempo(){

}

uint16_t get_music_pitch(){

}
