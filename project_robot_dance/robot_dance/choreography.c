#include <stdio.h>
#include <stdlib.h>

#include <stdint.h>
#include <ch.h>
#include <time.h>

#include <chprintf.h>

#include "IR_detection.h"
#include "signals_processing.h"

#define MOVE_NB 10
#define DEFAULT_BLINK_DELAY_ON 500
#define DEFAULT_BLINK_DELAY_OFF 500
#define DEFAULT_BLINK_ITERATION 10
#define DEFAULT_ROTATION_ITERATION 1
#define DEFAULT_MOVE_TIME_S 3
#define MOTOR_MAX_SPEED 2200
#define MOTOR_HIGH_SPEED 1800
#define MOTOR_MEDIUM_SPEED 1100
#define MOTOR_LOW_SPEED 500
#define MOTOR_TURTLE_SPEED 100

#define TEMPO_0 60
#define TEMPO_1 80
#define TEMPO_2 100
#define TEMPO_3 120
#define TEMPO_4 140
#define TEMPO_5 160

#define PITCH_0 80
#define PITCH_1 160
#define PITCH_2 320
#define PITCH_3 630
#define PITCH_4 1300
#define PITCH_5 2500

typedef enum {
	LED2,
	LED4,
	LED6,
	LED8,
	NUM_RGB_LED,
} rgb_led_name_t;

typedef struct thd_led_args
{
    int led;
    int iterations;
    int delay_on;
    int delay_off;
} thd_led_args;

typedef struct thd_motor_args
{
    uint8_t time_s;
    uint16_t speed_left;
    uint16_t speed_right;
} thd_motor_args;

int choose_move();
void move(int move_chosen);
void escape_obstacle();
void move_forward(uint8_t time_s, uint16_t speed);
void move_backward(uint8_t time_s, uint16_t speed);
void full_rotation(uint8_t iterations, uint16_t speed);
void turn_around();
void blink_LED1(int iterations, int delay_on, int delay_off);
void blink_LED3(int iterations, int delay_on, int delay_off);
void blink_LED5(int iterations, int delay_on, int delay_off);
void blink_LED7(int iterations, int delay_on, int delay_off);
void blink_LED_FRONT(int iterations, int delay_on, int delay_off);
void blink_LED_BODY(int iterations, int delay_on, int delay_off);

static bool obstacle[8] = {false};
static thd_motor_args motor_args;
static bool move_done = true;

static THD_WORKING_AREA(waThdDance, 256);
static THD_FUNCTION(ThdDance, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;
    while(1){
    	if(move_done == true){
    		move(choose_move());// pas random pour l'instant
    		move_done = false;
    	}

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

int choose_and_set_RGB(rgb_led_name_t led_number){
    uint16_t pitch = get_music_pitch();
    if (pitch < PITCH_0 ) {
        set_rgb_led(led_number, 255, 0 , 0);
    } else if (pitch < PITCH_1) {
        set_rgb_led(led_number, 255, 127, 0);
    } else if (pitch < PITCH_2) {
        set_rgb_led(led_number, 255, 255, 0);
    } else if (pitch < PITCH_3) {
        set_rgb_led(led_number, 0, 255, 0);
    } else if (pitch < PITCH_4) {
        set_rgb_led(led_number, 0, 0, 255);
    } else if (pitch < PITCH_5) {
        set_rgb_led(led_number, 75, 0, 130);
    } else {
        set_rgb_led(led_number, 148, 0, 211);
    }  
}

/**
* @brief Choose the move to execute
*
* @return the value of the move
*/
int choose_move(){
	if (is_obstacle() == true){
        return 0;
    } else {
        uint16_t tempo = get_music_tempo();
        uint16_t pitch = get_music_pitch();
    	uint8_t random = 1 + rand() % (MOVE_NB - 1);
    	//chprintf((BaseSequentialStream *)&SD3, " random: %d \n", random);
        return (random);
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
        move_forward(DEFAULT_MOVE_TIME_S, MOTOR_MEDIUM_SPEED);
        break;
    case 2:
        move_backward(DEFAULT_MOVE_TIME_S, MOTOR_MEDIUM_SPEED);
        break;
    case 3:
        full_rotation(DEFAULT_ROTATION_ITERATION, MOTOR_MEDIUM_SPEED);
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

// What to do if new thread started ????? TO DO
static THD_WORKING_AREA(waThdMotor, 256);
static THD_FUNCTION(ThdMotor, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_motor_args *motor_info = arg;
    //chprintf((BaseSequentialStream *)&SD3, "time: %d speed_right: %d speed_left: %d \n", motor_info->time_s, motor_info->speed_right, motor_info->speed_left);
    right_motor_set_speed(motor_info->speed_right);
    left_motor_set_speed(motor_info->speed_left);
    chThdSleepMilliseconds((motor_info->time_s) * 1000);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    move_done = true;
    chThdExit(0);
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
void move_forward(uint8_t time_s, uint16_t speed){
	motor_args.time_s = time_s;
	motor_args.speed_left = speed;
	motor_args.speed_right = speed;
    chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief Move the epuck backward
*/
void move_backward(uint8_t time_s, uint16_t speed){
	motor_args.time_s = time_s;
	motor_args.speed_left = -speed;
	motor_args.speed_right = -speed;
    chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief DO a full rotation of the epuck
*
* @param iterations number of iterations to execute 
*/
void full_rotation(uint8_t iterations, uint16_t speed){
    // TO DO
}

/**
* @brief Make the epuck turn around
*/
void turn_around(){
    // TO DO
}

// SI fonctionne pas cf https://www.chibios.org/dokuwiki/doku.php?id=chibios:documentation:books:rt:kernel_threading TO DO
static THD_WORKING_AREA(waThdLedLED1, 256);
static THD_WORKING_AREA(waThdLedLED3, 256);
static THD_WORKING_AREA(waThdLedLED5, 256);
static THD_WORKING_AREA(waThdLedLED7, 256);
static THD_WORKING_AREA(waThdLedFRONT_LED, 256);
static THD_WORKING_AREA(waThdLedBODY_LED, 256);
static THD_FUNCTION(ThdLed, arg) {
 chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_led_args *led_info = arg;
    stm32_gpio_t* gpio;
    int led = led_info->led;
    if (led_info->led == GPIOB_LED_BODY){
        gpio = GPIOB;
    } else {
        gpio = GPIOD;
    }
    palWritePad(gpio, led, 1); // First set on of the LED, should it be first off ? TO DO
    for (int i = 0; i < led_info->iterations; i ++){ // int i or static int i ?? TO DO
        //palTogglePad(gpio, led);
        chThdSleepMilliseconds(led_info->delay_off);
        palWritePad(gpio, led, 0);
        chThdSleepMilliseconds(led_info->delay_on);
        palWritePad(gpio, led, 1); 
    }
    move_done = true;
    chThdExit(0);
}

/**
* @brief Blink LED1
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED1(int iterations, int delay_on, int delay_off){
    static thd_led_args led_args;
    led_args.led = GPIOD_LED1;
    led_args.iterations = iterations;
    led_args.delay_on = delay_on;
    led_args.delay_off = delay_off;
    chThdCreateStatic(waThdLedLED1, sizeof(waThdLedLED1), NORMALPRIO, ThdLed, &led_args);
}

/**
* @brief Blink LED3
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED3(int iterations, int delay_on, int delay_off){
    static thd_led_args led_args;
    led_args.led = GPIOD_LED3;
    led_args.iterations = iterations;
    led_args.delay_on = delay_on;
    led_args.delay_off = delay_off;
    chThdCreateStatic(waThdLedLED3, sizeof(waThdLedLED3), NORMALPRIO, ThdLed, &led_args);
}

/**
* @brief Blink LED5
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED5(int iterations, int delay_on, int delay_off){
    static thd_led_args led_args;
    led_args.led = GPIOD_LED5;
    led_args.iterations = iterations;
    led_args.delay_on = delay_on;
    led_args.delay_off = delay_off;
    chThdCreateStatic(waThdLedLED5, sizeof(waThdLedLED5), NORMALPRIO, ThdLed, &led_args);
}

/**
* @brief Blink LED7
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED7(int iterations, int delay_on, int delay_off){
    static thd_led_args led_args;
    led_args.led = GPIOD_LED7;
    led_args.iterations = iterations;
    led_args.delay_on = delay_on;
    led_args.delay_off = delay_off;
    chThdCreateStatic(waThdLedLED7, sizeof(waThdLedLED7), NORMALPRIO, ThdLed, &led_args);
}

/**
* @brief Blink FRONT_LED
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_FRONT(int iterations, int delay_on, int delay_off){
    static thd_led_args led_args;
    led_args.led = GPIOD_LED_FRONT;
    led_args.iterations = iterations;
    led_args.delay_on = delay_on;
    led_args.delay_off = delay_off;
    chThdCreateStatic(waThdLedFRONT_LED, sizeof(waThdLedFRONT_LED), NORMALPRIO, ThdLed, &led_args);
}

/**
* @brief Blink BODY_LED
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED_BODY(int iterations, int delay_on, int delay_off){
    static thd_led_args led_args;
    led_args.led = GPIOB_LED_BODY;
    led_args.iterations = iterations;
    led_args.delay_on = delay_on;
    led_args.delay_off = delay_off;
    chThdCreateStatic(waThdLedBODY_LED, sizeof(waThdLedBODY_LED), NORMALPRIO, ThdLed, &led_args);
}
