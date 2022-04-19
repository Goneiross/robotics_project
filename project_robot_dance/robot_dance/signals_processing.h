#ifndef SIGNALS_PROCESSING_H
#define SIGNALS_PROCESSING_H

#define CHUNK_SIZE 	1024

#define MIC_OUTPUT LEFT_OUTPUT

void signals_processing_init(void);
void processAudioData(int16_t *data, uint16_t num_samples);
float* get_audio_buffer_ptr();
void wait_send_to_computer(void);

#endif
