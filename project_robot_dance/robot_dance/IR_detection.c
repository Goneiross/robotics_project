#include <stdio.h>
#include <stdlib.h>

#include <ch.h>
#include <hal.h>

#include <sensors/proximity.h>
#include <chprintf.h>
#include "memory_protection.h"

#include "choreography.h"
#include "IR_detection.h"

#define DEBUG_LEVEL 0
#define LED_IR_nb 8
#define THRESHOLD_PROX_MIN 10 // MODIFIER CA !!! 
#define THRESHOLD_PROX_MAX 1000 // MODIFIER CA !!! 
#define THRESHOLD_DIST 40
#define P1 0.00017303
#define P2 0.00054629

static int prox[8] = {0};
static int obstacle_dist[8] = {0};

void compute_distance(void);
void debug_detection(int level);
//int detection_init(void);
//bool is_obstacle(void);
//void update_obstacle_array(bool *obstacle);

static THD_WORKING_AREA(waThdDetection, 1024);
static THD_FUNCTION(ThdDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    calibrate_ir();

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
            obstacle_dist[i] = (1/(float)prox[i] + P2)/P1;
            // 1/prox = p1*distance - p2
            //distance = (1/prox+p2)/p1
            // p1 = 0.00017303
            //p2 = 0.00054629
        }
    }
}

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
            chprintf((BaseSequentialStream *)&SD3, "Prox%d=%d Dist%d=%d ", i, prox[i],i ,obstacle_dist[i]);
        }
        chprintf((BaseSequentialStream *)&SD3, "\n");
    }
}


/**
* @brief Initializes IR detection and calibrates it
*
* @return 0 if no error
*/
int detection_init(){
    proximity_start();
    chThdCreateStatic(waThdDetection, sizeof(waThdDetection), NORMALPRIO, ThdDetection, NULL);
    unsigned int rand_seed = get_prox(0)*get_prox(1)*get_prox(2)*get_prox(3)*get_prox(4)*get_prox(5)*get_prox(6)*get_prox(7);
    srand(rand_seed); 
    return 0;
}

/**
* @brief Returns true if there is an obstacle
*
* @return Bool true if obstacle
*/
bool is_obstacle(){
    for (int i = 0; i < LED_IR_nb; i++){
        if (obstacle_dist[i] < THRESHOLD_DIST) {
            return true;
        }
    }
    return false;
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
