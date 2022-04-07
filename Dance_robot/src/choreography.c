#include <stdint.h>
#include <ch.h>

#include "IR_detection.h"
#include "signals_processing.h"

static THD_WORKING_AREA(waThdDance, 128);
static THD_FUNCTION(ThdDance, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void full_rotation(uint8_t iteration){

}

void turn_around(){

}

void move_forward(){

}

void move_backward(){

}

void blink_LED_1(int iteration, int delay1, int delay2){

}

void blink_LED_3(int iteration, int delay1, int delay2){

}

void blink_LED_5(int iteration, int delay1, int delay2){

}

void blink_LED_7(int iteration, int delay1, int delay2){

}

void blink_LED_FRONT(int iteration, int delay1, int delay2){

}

void blink_LED_BODY(int iteration, int delay1, int delay2){

}