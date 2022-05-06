#include <stdint.h>
#include <ch.h>
#include <hal.h>
#include <audio/microphone.h>
#include <chprintf.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <msgbus/messagebus.h>

#include "signals_processing.h"

#define MIC_OUTPUT LEFT_OUTPUT
#define LOW_FILTER_PITCH_I 8
#define HIGH_FILTER_PITCH_I 256
#define LOW_FILTER_CORR_I 4
#define HIGH_FILTER_CORR_I 46
#define AUDIO_PROCESS_TIME 69
#define MINUTE_IN_MS 60000
#define ONSET_NB_SEND 5

uint16_t find_maximum_index(float* array_buffer, uint16_t min_range, uint16_t max_range);

static float mic_output[CHUNK_SIZE/2];
static float mic_cmplx_output[CHUNK_SIZE];

static float mic_input0[CHUNK_SIZE];
static float mic_input1[CHUNK_SIZE];
static arm_rfft_fast_instance_f32 arm_rfft_fast_f32_len1024;

static float rms_frequencies[WINDOW_SIZE];
static float rms_frequencies_shift[WINDOW_SIZE];
static float rms_frequency_old = 0;

static float auto_correlation[2*WINDOW_SIZE];

static BSEMAPHORE_DECL(onset_sem, TRUE);

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
	arm_rfft_fast_init_f32(&arm_rfft_fast_f32_len1024, CHUNK_SIZE);
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
	int time_to_wait = 5;
	static uint16_t mic_input_i = 0;
	static bool input_number = false;
	static bool buffer_full = false;
	static uint8_t rms_frequencies_i = 0;
	float abs_derivative = 0;
	float rms_frequency = 0;
	static float mean_rms_derivative_fft = 0;

	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		if(input_number == false){
			mic_input0[mic_input_i] = (float)data[i + MIC_BACK];
		}
		if(input_number == true){
			mic_input1[mic_input_i] = (float)data[i + MIC_BACK];
		}
		mic_input_i++;

		if(mic_input_i >= (CHUNK_SIZE)){
			input_number = !input_number;
			mic_input_i = 0;
			buffer_full = true;
		}
	}

	if(buffer_full){
		buffer_full = false;
		//arm_cmplx_mag_f32(mic_cmplx_output, mic_amplitude, CHUNK_SIZE);
		if(input_number == true){
			arm_rfft_fast_f32(&arm_rfft_fast_f32_len1024,mic_input0,mic_cmplx_output,0);
		} else {
			arm_rfft_fast_f32(&arm_rfft_fast_f32_len1024,mic_input1,mic_cmplx_output,0);
		}
		arm_cmplx_mag_f32(mic_cmplx_output, mic_output, CHUNK_SIZE/2);
		arm_rms_f32(&mic_output,CHUNK_SIZE/2,&rms_frequency);
		rms_frequencies[rms_frequencies_i] = rms_frequency;
		rms_frequencies_i ++;
		//chBSemSignal(&sendToComputer_sem);

		abs_derivative = fabs(rms_frequency - rms_frequency_old);
		//chprintf((BaseSequentialStream *)&SD3, "rms old: %f   ,  rms new: %f,  abs_derivative:%f \n", rms_frequency, rms_frequency_old, abs_derivative);
		rms_frequency_old = rms_frequency;

		if(rms_frequencies_i >= (WINDOW_SIZE)){
			rms_frequencies_i = 0;
			rms_frequencies_shift[0] = rms_frequencies[0];
			for(uint8_t i=0; i < WINDOW_SIZE-1; i++){
				rms_frequencies_shift[i+1] = rms_frequencies[i];
			}
			arm_sub_f32(&rms_frequencies,&rms_frequencies_shift,&rms_frequencies_shift,WINDOW_SIZE);
			arm_abs_f32(&rms_frequencies_shift,&rms_frequencies_shift,WINDOW_SIZE);
			arm_rms_f32(&rms_frequencies_shift,WINDOW_SIZE,&mean_rms_derivative_fft);
			for(uint8_t i=0; i < WINDOW_SIZE; i++){
				rms_frequencies_shift[i] = rms_frequencies_shift[i]-mean_rms_derivative_fft;
			}
			arm_correlate_f32(&rms_frequencies_shift,WINDOW_SIZE,&rms_frequencies_shift,WINDOW_SIZE,&auto_correlation);
		}
		if(mean_rms_derivative_fft <= abs_derivative){
			//chprintf((BaseSequentialStream *)&SD3, "value at this time: %f   ,    mean_derivative: %f \n", abs_derivative, mean_rms_derivative_fft);
			for(uint8_t i=0; i<ONSET_NB_SEND; i++){
				chBSemSignal(&onset_sem);
			}
		}

		if(wait10 == time_to_wait){
			wait10 = 0;
			//chBSemSignal(&sendToComputer_sem);
		} else {
			wait10 ++;
		}
	}
}

float* get_audio_buffer_ptr(void){
		return mic_output;
}

float* get_rms_frequencies(void){
	return rms_frequencies_shift;
}
float* get_auto_correlation(void){
	return auto_correlation;
}

/**
* @brief wait for the onset to be detected and sent on the semaphore
*/
void wait_onset(void){
	chBSemWait(&onset_sem);
}

/**
* @brief Returns the index of the maximal value in an array of positive float in the range specified between min and max
*
* @param array_buffer buffer from which the max value has to be found
* @param min_range lower bound index to start searching from
* @param max_range higher bound index to stop searching for it
*
* @return The uint_t of the index of the max
*/
uint16_t find_maximum_index(float* array_buffer, uint16_t min_range, uint16_t max_range){
	uint16_t max_index = 0;
	for(uint16_t i = min_range; i < max_range; i++){
		if(array_buffer[i] > array_buffer[max_index]){
			max_index = i;
		}
	}
	return max_index;
}

/**
* @brief get the tempo in bpm of what is recorded by the microphone
*
* @return tempo
*/
uint8_t get_music_tempo(void){
	uint16_t min_range = 4+WINDOW_SIZE;
	uint16_t max_range = 46+WINDOW_SIZE;
	uint16_t interval = get_music_interval();
	uint8_t tempo = (uint8_t)((float)MINUTE_IN_MS/(float)interval);
	return tempo;
}

/**
* @brief get the interval in time between two beats (periodicity of the music)
*
* @return interval in ms
*/
uint16_t get_music_interval(void){
	uint16_t min_range = LOW_FILTER_CORR_I+WINDOW_SIZE;
	uint16_t max_range = HIGH_FILTER_CORR_I+WINDOW_SIZE;
	uint16_t music_interval = AUDIO_PROCESS_TIME*(find_maximum_index(auto_correlation, min_range, max_range)-WINDOW_SIZE);
	if(music_interval > 2000){
		music_interval = 2000;
	}
	return music_interval;
}

//we take the maximum pitch between 125hz (3d key) and 4000hz on one side only which gives us the index 8 to 256 as it goes from 0 to 512 in incremented values of 15.625hz
uint16_t get_music_pitch(void){
	uint16_t min_range = LOW_FILTER_PITCH_I;
	uint16_t max_range = HIGH_FILTER_PITCH_I;
	uint32_t frequency = find_maximum_index(mic_output, min_range, max_range) * 15625;
	//arm_max_f32  il est possible d'utiliser le dsp mais on aura pas de filtre à ce moment
	return (uint16_t)(frequency/1000);
}

uint16_t get_music_amplitude(void){
	uint16_t amplitude = (uint16_t)rms_frequency_old;
	return amplitude;
}
