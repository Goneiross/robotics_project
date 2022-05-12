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
#define LOW_FILTER_PITCH_I 5
#define HIGH_FILTER_PITCH_I 256
#define LOW_FILTER_CORR_I 4
#define HIGH_FILTER_CORR_I 46
#define AUDIO_PROCESS_TIME 71
#define MINUTE_IN_MS 60000
#define ONSET_NB_SEND 4
#define ONSET_COEF 1
#define BIG_ONSET_COEF 3
#define COMPRESSION_LOG_FACTOR 100
#define COMPRESSION_SCALE 100

#ifdef DATA_TO_COMPUTER
#define TIME_TO_WAIT  10
#endif

uint16_t find_maximum_index(float* array_buffer, uint16_t min_range, uint16_t max_range);
bool chBSemGetState(binary_semaphore_t *bsp);
bool fill_mic_input_arrays(int16_t *data, uint16_t num_samples, bool* input_number);
void real_fft_magnitude(bool* input_number);
float derivate_frequencies(uint8_t *rms_frequencies_i);
void auto_corr_freq(uint8_t *rms_frequencies_i, float *mean_rms_derivative_fft);
void onset_detection(float *mean_rms_derivative_fft, float abs_freq_derivative);

static float mic_fft[CHUNK_SIZE/2];
static float mic_cmplx_fft[CHUNK_SIZE];

static float mic_input0[CHUNK_SIZE];
static float mic_input1[CHUNK_SIZE];
static arm_rfft_fast_instance_f32 arm_rfft_fast_f32_len1024;

static float rms_frequencies[WINDOW_SIZE];
static float rms_frequencies_w[WINDOW_SIZE];

static float auto_correlation[2*WINDOW_SIZE];

static BSEMAPHORE_DECL(onset_sem, TRUE);
static BSEMAPHORE_DECL(big_onset_sem, TRUE);
static BSEMAPHORE_DECL(tempo_update_sem, FALSE);

#ifdef DATA_TO_COMPUTER
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);
#endif

/**
* @brief initialise microphone start and fft parameters
*/
void signals_processing_init(void){
	mic_start(&processAudioData);
	arm_rfft_fast_init_f32(&arm_rfft_fast_f32_len1024, CHUNK_SIZE);
}

/**
* @brief Function called by the thread of the microphone to process the signal.
*
* @param data Data received from the microphone.
* @param num_samples Number of samples received by the microphone.
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	static bool input_number = 0;
	static uint8_t rms_frequencies_i = 0;
	static float mean_rms_derivative_fft = 0;

	bool buffer_full = false;
	buffer_full = fill_mic_input_arrays(data, num_samples, &input_number);

	if(buffer_full){
		float abs_freq_derivative = 0;

		real_fft_magnitude(&input_number);
		abs_freq_derivative = derivate_frequencies(&rms_frequencies_i);
		auto_corr_freq(&rms_frequencies_i, &mean_rms_derivative_fft);
		onset_detection(&mean_rms_derivative_fft, abs_freq_derivative);

		#ifdef DATA_TO_COMPUTER1
		static int wait = 0;
		if(wait == TIME_TO_WAIT){
			wait = 0;
			chBSemSignal(&sendToComputer_sem);
		} else {
			wait ++;
		}
		#endif
	}
}

/**
* @brief detect if the last change in mean frequency is big relative the mean and send a signal if it is.
*
* @param mean_rms_derivative_fft Pointer to the mean of the novelty function.
* @param abs_freq_derivative The absolute value of (the last change in mean(rms) frequency) * 2.
*/
void onset_detection(float *mean_rms_derivative_fft, float abs_freq_derivative){
	if((*mean_rms_derivative_fft)*BIG_ONSET_COEF <= abs_freq_derivative){
		chBSemSignal(&big_onset_sem);
	}
	if((*mean_rms_derivative_fft)*ONSET_COEF <= abs_freq_derivative){
		for(uint8_t i=0; i<ONSET_NB_SEND; i++){
			chBSemSignal(&onset_sem);
		}
		if(chBSemGetState(&tempo_update_sem)){
			for(uint8_t i=0; i<ONSET_NB_SEND; i++){
				chBSemSignal(&tempo_update_sem);
			}
		}
	}
}

/**
* @brief Compute the autocorrelation of the novelty function in order to retrieve periodicity.
*
* @param rms_frequencies_i Pointer to the index of the array of rms frequencies to fill.
* @param mean_rms_derivative_fft Pointer to the mean of the novelty function.
*/
void auto_corr_freq(uint8_t *rms_frequencies_i, float *mean_rms_derivative_fft){
	if((*rms_frequencies_i) >= (WINDOW_SIZE)){
		(*rms_frequencies_i) = 0;
		rms_frequencies_w[0] = rms_frequencies[0];
		for(uint8_t i=0; i < WINDOW_SIZE-1; i++){
			rms_frequencies_w[i+1] = rms_frequencies[i];
		}
		arm_sub_f32(rms_frequencies, rms_frequencies_w, rms_frequencies_w,WINDOW_SIZE);
		arm_abs_f32(rms_frequencies_w, rms_frequencies_w, WINDOW_SIZE);
		arm_rms_f32(rms_frequencies_w,WINDOW_SIZE,mean_rms_derivative_fft);
		for(uint8_t i=0; i < WINDOW_SIZE; i++){
			rms_frequencies_w[i] = rms_frequencies_w[i]-(*mean_rms_derivative_fft);
		}
		arm_correlate_f32(rms_frequencies_w, WINDOW_SIZE, rms_frequencies_w, WINDOW_SIZE, auto_correlation);
		chBSemReset(&tempo_update_sem,true);
		#ifdef DATA_TO_COMPUTER2
		chBSemSignal(&sendToComputer_sem);
		#endif
	}
}

/**
* @brief Compute the rms value of the last fft spectrum and stores it in rms_frequencies[] to get a novelty function.
*
* @param rms_frequencies_i Pointer to the index of the array of rms frequencies to fill.
*
* @return The absolute value of (the last change in mean(rms) frequency) * 2.
*/
float derivate_frequencies(uint8_t *rms_frequencies_i){
		static float rms_frequency_old;
		float abs_freq_derivative = 0;
		float rms_frequency = 0;
		arm_rms_f32(mic_fft,CHUNK_SIZE/2,&rms_frequency);
		rms_frequency = COMPRESSION_SCALE*log(1+ COMPRESSION_LOG_FACTOR * rms_frequency);
		rms_frequencies[*rms_frequencies_i] = rms_frequency;
		(*rms_frequencies_i) ++;
		abs_freq_derivative = fabs(rms_frequency - rms_frequency_old);
		rms_frequency_old = rms_frequency;

		return abs_freq_derivative;
}

/**
* @brief Perform the real fft of the last CHUNK_SIZE samples and stores the magnitude in mic_fft[].
*
* @param input_number Index of the array that was last filled.
*/
void real_fft_magnitude(bool *input_number){
	if(*input_number == 1){
		arm_rfft_fast_f32(&arm_rfft_fast_f32_len1024,mic_input0,mic_cmplx_fft,0);
	} else {
		arm_rfft_fast_f32(&arm_rfft_fast_f32_len1024,mic_input1,mic_cmplx_fft,0);
	}
	arm_cmplx_mag_f32(mic_cmplx_fft, mic_fft, CHUNK_SIZE/2);
}

#ifdef DATA_TO_COMPUTER
/**
* @brief Get the array of fft for debug.
*
* @return Array of fft data.
*/
float* get_audio_buffer_ptr(void){
		return mic_fft;
}

/**
* @brief Get the array of novelty function for debug.
*
* @return Array of the novelty function.
*/
float* get_rms_frequencies(void){
	return rms_frequencies_w;
}

/**
* @brief Get the array of auto correlation.
*
* @return Array of auto_correlation function.
*/
float* get_auto_correlation(void){
	return auto_correlation;
}
#endif

/**
* @brief Wait for the onset to be detected and sent on the semaphore.
*/
void wait_onset(void){
	chBSemWait(&onset_sem);
}

/**
* @brief Wait for a big onset to be detected and sent on the semaphore.
*/
void wait_big_onset(void){
	chBSemWait(&big_onset_sem);
}

/**
* @brief Wait for an onset to send a signal on the tempo_update_semaphore.
*/
void wait_tempo_update(void){
	chBSemWait(&tempo_update_sem);
}

/**
* @brief Retrieve the state of the tempo update
*
* @return Bool, true if a new tempo update is available, false otherwise.
*/
bool state_tempo_update(void){
	return chBSemGetState(&tempo_update_sem);
}

/**
* @brief Wait for the onset to be detected and sent on the semaphore.
*/
void reset_tempo_update(void){
	chBSemReset(&tempo_update_sem, FALSE);
}

/**
* @brief Completion of the library, normal version of the chBSemGetStateI function.
*
* @param bsp Pointer to the semaphore we want the state of.
*
* @return Bool, true if the semaphore is taken, false otherwise.
*/
bool chBSemGetState(binary_semaphore_t *bsp){
	bool state = 0;
	chSysLock();
	state = chBSemGetStateI(bsp);
	chSysUnlock();
	return state;
}

/**
* @brief Fill the arrays mic_input0 or mic_input1 that is not full yet with the good data.
*
* @param data Pointer to the data received from the microphone.
* @param num_samples Number of samples received from the microphone.
* @param input_number Pointer to the index of the input array to fill.
*
* @return Bool indicating if the buffer is full yet.
*/
bool fill_mic_input_arrays(int16_t *data, uint16_t num_samples, bool *input_number){
	static uint16_t mic_input_i = 0;
	bool buffer_full = false;
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		if((*input_number) == 0){
			mic_input0[mic_input_i] = (float)data[i + MIC_BACK];
		}
		if((*input_number) == 1){
			mic_input1[mic_input_i] = (float)data[i + MIC_BACK];
		}
		mic_input_i++;

		if(mic_input_i >= (CHUNK_SIZE)){
			(*input_number) = !(*input_number);
			mic_input_i = 0;
			buffer_full = true;
		}
	}
	return buffer_full;
}


/**
* @brief Returns the index of the maximal value in an array of positive float in the range specified between min and max.
*
* @param array_buffer Buffer from which the max value has to be found.
* @param min_range Lower bound index to start searching from.
* @param max_range Higher bound index to stop searching for it.
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
* @brief Get the tempo in bpm of what is recorded by the microphone.
*
* @return Tempo in bpm.
*/
uint8_t get_music_tempo(void){
	uint16_t interval = get_music_interval();
	uint8_t tempo = MINUTE_IN_MS/interval;
	return tempo;
}

/**
* @brief Get the interval in time between two beats (periodicity of the music).
*
* @return Interval in ms.
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

/**
* @brief Get the pitch of the last CHUNK_SIZE smaples of the microphone.
*
* @return Pitch in Hz.
*/
uint16_t get_music_pitch(void){
	uint16_t min_range = LOW_FILTER_PITCH_I;
	uint16_t max_range = HIGH_FILTER_PITCH_I;
	uint32_t frequency = find_maximum_index(mic_fft, min_range, max_range) * 15625;
	return (uint16_t)(frequency/1000);
}

/**
* @brief Get the amplitude of the sound.
*
* @return Amplitude of the last CHUNK_SIZE of the microphone in arbitrary unit.
*/
uint16_t get_music_amplitude(void){
	uint16_t min_range = LOW_FILTER_PITCH_I;
	uint16_t max_range = HIGH_FILTER_PITCH_I;
	uint16_t index_max = 0;
	index_max = find_maximum_index(mic_fft, min_range, max_range);
	uint16_t amplitude = mic_fft[index_max];
	return amplitude;
}

#ifdef DATA_TO_COMPUTER
/**
* @brief Send a signal on the semaphore when data are available for debug.
*/
void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}
#endif
