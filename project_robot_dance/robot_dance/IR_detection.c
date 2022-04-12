#include <ch.h>
#include "hal.h"
//#include <sensors/proximity.h>
#include <chprintf.h>
#include "memory_protection.h"

#include "choreography.h"

#define LED_IR_nb 8

static int prox[8] = {0};

static THD_WORKING_AREA(waThdDetection, 128);
static THD_FUNCTION(ThdDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        for (int i = 0; i < LED_IR_nb; i++){
          //  prox[i] = get_prox(i);
        }
        debug_detection(3);
        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

/**
* @brief Initializes IR detection and calibrates it
*
* @return 0 if no error
*/
int detection_init(){
    //proximity_start();
    //calibrate_ir();
    chThdCreateStatic(waThdDetection, sizeof(waThdDetection), NORMALPRIO, ThdDetection, NULL);
    return 0;
}

/**
* @brief Returns the distance to the objetc]
*
* @return Distance to the object
*/
float get_obstacle_angle(){
    return ;
}

/**
* @brief Returns the distance to the obstacle
*
* @return Distance to the object
*/
float get_obstacle_distance(){
    return ;
}

/**
* @brief chprintf data to debug
*
* @param level 0 none, 1 basics, 2 medium, 3 all
*
* @return nothing
*/
void debug_detection(int level){
    if(level >= 1) {
       // chprintf((BaseSequentialStream *)&SD3, "Debug level %d : \n", level); 
    } 
    if(level >= 2) {
    }
    if (level >= 3){
        for (int i = 0; i < LED_IR_nb; i++){
            // chprintf((BaseSequentialStream *)&SD3, "Prox%d=%d ", i, prox[i]);
        }
        // chprintf((BaseSequentialStream *)&SD3, "\n");
    }
}
