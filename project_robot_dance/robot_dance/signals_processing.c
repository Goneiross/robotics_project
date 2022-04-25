#include <stdint.h>
#include <ch.h>
#include <hal.h>
#include <audio/microphone.h>
#include <chprintf.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#include "signals_processing.h"
#include "choreography.h"

//#define C_FFT
#define R_FFT

static float mic_output[CHUNK_SIZE/2];
static float mic_cmplx_input[2*CHUNK_SIZE];

#ifdef R_FFT
static float mic_input[CHUNK_SIZE];
static arm_rfft_fast_instance_f32 arm_rfft_fast_f32_len1024;
#endif

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
	#ifdef R_FFT
	arm_rfft_fast_init_f32(&arm_rfft_fast_f32_len1024, CHUNK_SIZE);
	#endif
}

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static int wait10 = 0;
	int time_to_wait = 100;
	static uint16_t nb_samples = 0;

	/*for(int i = 0; i <num_samples; i+=4){
		mic_cmplx_input[nb_samples] = data[i + MIC_BACK]

	}*/


	for(uint16_t i = 0 ; i < num_samples ; i+=4){
	#ifdef R_FFT
		mic_input[nb_samples] = (float)data[i + MIC_BACK];
		nb_samples++;
		if(nb_samples >= (CHUNK_SIZE)){
			break;
		}
	#endif

	#ifdef C_FFT
		mic_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		nb_samples++;
		mic_cmplx_input[nb_samples] = 0;
		nb_samples++;
		if(nb_samples >= (2 * CHUNK_SIZE)){
			break;
		}
	#endif
	}

	#ifdef C_FFT
	if(nb_samples >= (2 * CHUNK_SIZE)){
	#endif
	#ifdef R_FFT
	if(nb_samples >= (CHUNK_SIZE)){
	#endif
		nb_samples = 0;
		//arm_cmplx_mag_f32(mic_cmplx_input, mic_amplitude, CHUNK_SIZE);
		#ifdef R_FFT
		arm_rfft_fast_f32(&arm_rfft_fast_f32_len1024,mic_input,mic_cmplx_input,0);
		arm_cmplx_mag_f32(mic_cmplx_input, mic_output, CHUNK_SIZE/2);
		#endif
		#ifdef C_FFT
		doFFT_optimized(CHUNK_SIZE, mic_cmplx_input);
		arm_cmplx_mag_f32(mic_cmplx_input, mic_output, CHUNK_SIZE);
		#endif


		if(wait10 == time_to_wait){
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

//we take the maximum pitch between 125hz (3d key) and 4000hz on one side only which gives us the index 8 to 256 as it goes from 0 to 512 in incremented values of 15.625hz
uint16_t get_music_pitch(){
	uint16_t min_range = LOW_FILTER_INDEX;
	uint16_t max_range = HIGH_FILTER_INDEX;
	uint32_t frequency = find_maximum_index(mic_output, min_range, max_range) * 78125;
	//arm_max_f32  il est possible d'utiliser le dsp mais on aura pas de filtre à ce moment
	return (uint16_t)(frequency/10000);
}

uint16_t get_amplitude(){

}
