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
static float mic_amplitude[CHUNK_SIZE];

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
			mic_cmplx_input[iteration] = data[4*i+2];
			i++;
			iteration ++;
	}

	if(iteration >= CHUNK_SIZE*2){
		iteration = 0;
		//arm_cmplx_mag_f32(mic_cmplx_input, mic_amplitude, CHUNK_SIZE);
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

/**
* @brief Returns the index of the maximal value in an array of positive float in the range specified between min and max
*
* @return The uint_t of the index of the max
*/
float find_maximum_index(float* array_buffer, uint16_t min_range, uint16_t max_range){
	uint16_t max_index = 0;
	for(uint16_t i = min_range; i < max_range; i++){
		if(array_buffer[i] > array_buffer[max_index]){
			max_index = i;
		}
	}
	return max_index;
}

uint16_t get_music_tempo(){

}

//we take the maximum pitch between 125hz (3d key) and 4000hz on one side only which gives us the index 16 to 512 as it goes from 0 to 512 in incremented values of 7.8125hz
uint16_t get_music_pitch(){
	uint16_t min_range = 16;
	uint16_t max_range = 512;
	uint32_t frequency = find_maximum_index(mic_output, min_range, max_range) * 78125;
	return (uint16_t)(frequency/10000);
}

uint16_t get_amplitude(){

}
