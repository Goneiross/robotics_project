#include <ch.h>
#include "hal.h"
#include <messagebus.h>
//#include <sensors/proximity.h>
#include <chprintf.h>
#include "memory_protection.h"

#include "choreography.h"

#define DEBUG_LEVEL 3
#define LED_IR_nb 8
#define THRESHOLD_PROX_MIN 10 // MODIFIER CA !!! 
#define THRESHOLD_PROX_MAX 1000 // MODIFIER CA !!! 
#define THRESHOLD_DIST 80  
#define FACTOR 0.01 // MODIFIER CA !!!

extern messagebus_t bus;

static int prox[8] = {0};
static int obstacle_dist[8] = {0};

static THD_WORKING_AREA(waThdDetection, 128);
static THD_FUNCTION(ThdDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        for (int i = 0; i < LED_IR_nb; i++){
          prox[i] = get_prox(i);
        }
        compute_distance();
        debug_detection(DEBUG_LEVEL);
        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

/**
* @brief Compute distance in mm from the obstacle if between threshlod. The threshold is used to remove unwanted data / noise
*/
void compute_distance(){
    for (int i = 0; i < LED_IR_nb; i++){
        obstacle_dist[i] = 0;
        if ((prox[i] > THRESHOLD_PROX_MIN) && (prox[i] < THRESHOLD_PROX_MAX)) {
            obstacle_dist[i] = FACTOR * prox[i];
        }
    }
}

/**
* @brief Initializes IR detection and calibrates it
*
* @return 0 if no error
*/
int detection_init(){
    proximity_start();
    calibrate_ir();
    chThdCreateStatic(waThdDetection, sizeof(waThdDetection), NORMALPRIO, ThdDetection, NULL);
    return 0;
}

/**
* @brief Returns if a 8 boolean array for object detection
*
* @return Array of obstacle detection
*/
void update_obstacle_array(bool *obstacle){
    for (int i = 0; i < LED_IR_nb; i++){
        obstacle[i] = false;
        if (obstacle_dist[i] < THRESHOLD_DIST) {
            obstacle[i] = true;
        }
    }
}

/**
* @brief Returns true if there is an obstacle
*
* @return Bool true if obstacle
*/
void update_obstacle_array(){
    for (int i = 0; i < LED_IR_nb; i++){
        if (obstacle_dist[i] < THRESHOLD_DIST) {
            return true;
        }
    }
    return false;
}

// /**
// * @brief Returns the distance to the objetct
// *
// * @return Distance to the object
// */
// float get_obstacle_angle(){
//     return ;
// }

// /**
// * @brief Returns the distance to the obstacle
// *
// * @return Distance to the object
// */
// float get_obstacle_distance(){
//     return ;
// }

/**
* @brief chprintf data to debug
*
* @param level 0 none, 1 basics, 2 medium, 3 all
*/
void debug_detection(int level){
    if(level >= 1) {
       chprintf((BaseSequentialStream *)&SD3, "Debug level %d : \n", level); 
    } 
    if(level >= 2) {
    }
    if (level >= 3){
        for (int i = 0; i < LED_IR_nb; i++){
            chprintf((BaseSequentialStream *)&SD3, "Prox%d=%d Dist%d=%d ", i, prox[i], obstacle_dist[i], i);
        }
        chprintf((BaseSequentialStream *)&SD3, "\n");
    }
}
