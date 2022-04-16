#include <stdint.h>
#include <ch.h>
#include <string>

#include "IR_detection.h"
#include "signals_processing.h"

#define MOVE_NB 10
#define DEFAULT_BLINK_DELAY_ON 50
#define DEFAULT_BLINK_DELAY_OFF 50
#define DEFAULT_BLINK_ITERATION 1
#define DEFAULT_ROTATION_ITERATION 1

typedef struct thd_led_args
{
    string led;
    int iterations;
    int delay_on;
    int delay_off;
} thd_led_args;


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
        blink_LED1(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 6:
        blink_LED3(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 7:
        blink_LED5(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
        break;
    case 8:
        blink_LED7(DEFAULT_BLINK_ITERATION, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
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

// SI fonctionne pas cf https://www.chibios.org/dokuwiki/doku.php?id=chibios:documentation:books:rt:kernel_threading TO DO
static THD_WORKING_AREA(waThdLed, 128);
static THD_FUNCTION(ThdLed, arg) {
 chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_led_args *led_info = arg;
    static GPIO_TypeDef gpio = "";
    if (led_info->led == "LED1"){
        gpio = GPIOD;
    } else if (led_info->led == "LED3"){
        gpio = GPIOD;
    } else if (led_info->led == "LED5"){
        gpio = GPIOD;
    } else if (led_info->led == "LED7"){
        gpio = GPIOD;
    } else if (led_info->led == "FRONT_LED"){
        gpio = GPIOD;
    } else if (led_info->led == "BODY_LED"){
        gpio = GPIOB;
    } else {
        // PANIC TO DO
    }
    palWritePad(gpio, led_info->led, 0); // First set on of the LED, should it be first off ? TO DO
    for (int i = 0; i < led_info->iterations; i ++){ // int i or static int i ?? TO DO
        //palTogglePad(gpio, led_info->led);
        chThdSleepMilliseconds(led_info->delay_on);
        palWritePad(gpio, led_info->led, 1); 
        chThdSleepMilliseconds(led_info->delay_off);
        palWritePad(gpio, led_info->led, 0); 
    }
    chThdExit();
}

/**
* @brief Blink LED1
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED1(int iterations, int delay_on, int delay_off){
    thd_led_args th_args = {
                            .led = "LED1",
                            .iterations = iterations,
                            .delay_on = delay_on,
                            .delay_off = delay_off}
    chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, &th_args);
}

/**
* @brief Blink LED3
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED3(int iterations, int delay_on, int delay_off){
    thd_led_args th_args = {
                            .led = "LED3",
                            .iterations = iterations,
                            .delay_on = delay_on,
                            .delay_off = delay_off}
    chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, &th_args);
}

/**
* @brief Blink LED5
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED5(int iterations, int delay_on, int delay_off){
    thd_led_args th_args = {
                            .led = "LED5",
                            .iterations = iterations,
                            .delay_on = delay_on,
                            .delay_off = delay_off}
    chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, &th_args);
}

/**
* @brief Blink LED7
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED7(int iterations, int delay_on, int delay_off){
    thd_led_args th_args = {
                            .led = "LED7",
                            .iterations = iterations,
                            .delay_on = delay_on,
                            .delay_off = delay_off}
    chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, &th_args);
}

/**
* @brief Blink FRONT_LED
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_FRONT(int iterations, int delay_on, int delay_off){
    thd_led_args th_args = {
                            .led = "FRONT_LED",
                            .iterations = iterations,
                            .delay_on = delay_on,
                            .delay_off = delay_off}
    chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, &th_args);
}

/**
* @brief Blink BODY_LED
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_BODY(int iterations, int delay_on, int delay_off){
    thd_led_args th_args = {
                            .led = "BODY_LED",
                            .iterations = iterations,
                            .delay_on = delay_on,
                            .delay_off = delay_off}
    chThdCreateStatic(waThdLed, sizeof(waThdLed), NORMALPRIO, ThdLed, &th_args);
}