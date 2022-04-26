#include <stdint.h>
#include <ch.h>

#include "choreography.h"

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

#include "choreography.h"

uint16_t get_music_tempo(){

}

uint16_t get_music_pitch(){
    return 1000;
}