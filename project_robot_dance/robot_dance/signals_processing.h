#ifndef SIGNALS_PROCESSING_H
#define SIGNALS_PROCESSING_H

#define CHUNK_SIZE 	1024
#define WINDOW_SIZE 64
#define MIC_OUTPUT LEFT_OUTPUT
#define LOW_FILTER_INDEX 8
#define HIGH_FILTER_INDEX 256

void signals_processing_init(void);
void processAudioData(int16_t *data, uint16_t num_samples);
float* get_audio_buffer_ptr(void);
float* get_rms_frequencies(void);
float* get_auto_correlation(void);
void wait_send_to_computer(void);
uint16_t get_music_tempo(void);
uint16_t get_music_pitch(void);
uint16_t get_music_interval(void);

#endif
