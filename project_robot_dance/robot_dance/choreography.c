#include <stdint.h>
#include <ch.h>

#include "IR_detection.h"
#include "signals_processing.h"

#define MOVE_NB 10
#define DEFAULT_BLINK_DELAY_ON 50
#define DEFAULT_BLINK_DELAY_OFF 50
#define DEFAULT_BLINK_ITERATION 1
#define DEFAULT_ROTATION_ITERATION 1

static bool obstacle[8] = {false};

static THD_WORKING_AREA(waThdDance, 128);
static THD_FUNCTION(ThdDance, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;
    while(1){
        move(choose_move());
        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

/**
* @brief Initializes the choreography
*
* @return 0 if no error
*/
int choreography_init(){
    chThdCreateStatic(waThdDance, sizeof(waThdDance), NORMALPRIO, ThdDance, NULL);
    return 0;
}

/**
* @brief Choose the move to execute
*
* @return the value of the move
*/
int choose_move(){
    if (is_obstacle()){
        return 0;
    } else {
        return (1 + rand() % (MOVE_NB - 1));
    }
}

/**
* @brief Call the right functions to execute the chosen move
*
* @param move_chosen the ID of the move to execute
*/
void move(int move_chosen){
    switch (move_chosen)
    {
    case 0:
        escape_obstacle();
        break;
    case 1:
        move_forward();
    case 2:
        move_backward();
        break;
    case 3:
        full_rotation(DEFAULT_ROTATION_ITERATION);
        break;
    case 4:
        turn_around();
        break;
    case 5:
        blink_LED_1(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 6:
        blink_LED_3(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 7:
        blink_LED_5(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 8:
        blink_LED_7(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 9:
        blink_LED_FRONT(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 10:
        blink_LED_BODY(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    default:
        break;
    }
}

/**
* @brief Try to escape the nearest obstacle
*/
void escape_obstacle(){
    update_obstacle_array(obstacle);
}

/**
* @brief Move the epuck forward
*/
void move_forward(){

}

/**
* @brief Move the epuck backward
*/
void move_backward(){

}

/**
* @brief DO a full rotation of the epuck
*
* @param iterations number of iterations to execute 
*/
void full_rotation(uint8_t iterations){

}

/**
* @brief Make the epuck turn around
*/
void turn_around(){

}

/**
* @brief Blink LED_1
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_1(int iterations, int delay_on, int delay_off){

}

/**
* @brief Blink LED_3
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_3(int iterations, int delay_on, int delay_off){

}

/**
* @brief Blink LED_5
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_5(int iterations, int delay_on, int delay_off){

}

/**
* @brief Blink LED_7
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_7(int iterations, int delay_on, int delay_off){

}

/**
* @brief Blink FRONT LED
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_FRONT(int iterations, int delay_on, int delay_off){

}

/**
* @brief Blink BODY LED
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_BODY(int iterations, int delay_on, int delay_off){

}