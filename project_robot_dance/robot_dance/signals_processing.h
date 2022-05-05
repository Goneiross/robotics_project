#ifndef SIGNALS_PROCESSING_H
#define SIGNALS_PROCESSING_H

#define CHUNK_SIZE 	1024
#define WINDOW_SIZE 64

#include <hal.h>

typedef struct {
    bool onset;
} onset_msg_t;

void signals_processing_init(void);
void processAudioData(int16_t *data, uint16_t num_samples);
float* get_audio_buffer_ptr(void);
float* get_rms_frequencies(void);
float* get_auto_correlation(void);
void wait_onset(void);
uint8_t get_music_tempo(void);
float get_music_interval(void);
uint16_t get_music_pitch(void);
uint16_t get_music_amplitude(void);



#endif
