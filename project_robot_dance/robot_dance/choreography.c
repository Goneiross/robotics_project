#include <stdio.h>
#include <stdlib.h>

#include <stdint.h>
#include <ch.h>
#include <hal.h>
#include <time.h>
#include "memory_protection.h"

#include <chprintf.h>
#include <leds.h>
#include <motors.h>
// #include "motor.h"
#include <spi_comm.h>
#include <msgbus/messagebus.h>

#include "choreography.h"
#include "IR_detection.h"
#include "signals_processing.h"

#define MOVE_NB 4
#define DEFAULT_BLINK_DELAY_ON 50
#define DEFAULT_BLINK_DELAY_OFF 50
#define DEFAULT_BLINK_ITERATION 10
#define DEFAULT_ROTATION_ITERATION 1
#define DEFAULT_MOVE_TIME_S 3
#define MOTOR_MAX_SPEED 2200
#define MOTOR_HIGH_SPEED 1800
#define MOTOR_MEDIUM_SPEED 1100
#define MOTOR_LOW_SPEED 500
#define MOTOR_TURTLE_SPEED 100

#define SOUND_AMP_MIN 60

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

#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1

#define COLOR_HZ_RANGE 2000

typedef enum {
	ESCAPE_OBSTACLE,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    FULL_ROTATION,
    TURN_AROUND,
    DO_NOTHING,
} move_type;

typedef enum {
	FOLLOW_PITCH,
    MANUAL,
} led_play_type;

typedef struct rgb
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb;

typedef struct thd_led_args
{
    int led;
    int iterations;
    int delay_on;
    int delay_off;
} thd_led_args;

typedef struct thd_rgb_led_args
{
    rgb_led_name_t led;
    int iterations;
    int delay_on;
    int delay_off;
    rgb colour;
    led_play_type led_play_type;
} thd_rgb_led_args;

typedef struct thd_motor_args
{
    uint8_t time_s;
    int16_t speed_left;
    int16_t speed_right;
} thd_motor_args;

typedef struct thd_motor_pos_args
{
    float position_r;
    float position_l;
    float speed_r;
    float speed_l;
} thd_motor_pos_args;

void blink_LED1(int iterations, int delay_on, int delay_off);
void blink_LED2(int iterations, int delay_on, int delay_off, rgb colour, int play_type);
void blink_LED3(int iterations, int delay_on, int delay_off);
void blink_LED4(int iterations, int delay_on, int delay_off, rgb colour, int play_type);
void blink_LED5(int iterations, int delay_on, int delay_off);
void blink_LED6(int iterations, int delay_on, int delay_off, rgb colour, int play_type);
void blink_LED7(int iterations, int delay_on, int delay_off);
void blink_LED_BODY(int iterations, int delay_on, int delay_off);
void blink_LED_FRONT(int iterations, int delay_on, int delay_off);
void choose_and_set_RGB(rgb_led_name_t led_number);
int choose_move(uint8_t old_move_nb);
void escape_obstacle(void);
void full_rotation(void);
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
void move(int move_chosen);
void move_backward(uint8_t time_s, int16_t speed);
void move_forward(uint8_t time_s, int16_t speed);
void start_leds(void);
void turn_around(void);
void do_nothing(uint8_t time_s);

static THD_WORKING_AREA(waThdDance, 256);
static THD_WORKING_AREA(waThdMotor, 256);
static THD_WORKING_AREA(waThdMotorPos, 1024);
static THD_WORKING_AREA(waThdLedLED1, 256);
static THD_WORKING_AREA(waThdLedLED2, 256);
static THD_WORKING_AREA(waThdLedLED3, 256);
static THD_WORKING_AREA(waThdLedLED4, 256);
static THD_WORKING_AREA(waThdLedLED5, 256);
static THD_WORKING_AREA(waThdLedLED6, 256);
static THD_WORKING_AREA(waThdLedLED7, 256);
static THD_WORKING_AREA(waThdLedLED8, 256);
static THD_WORKING_AREA(waThdLedFRONT_LED, 256);
static THD_WORKING_AREA(waThdLedBODY_LED, 256);

static bool obstacle[8] = {false};
static thd_motor_args motor_args;
static bool move_done = true;

static int16_t counter_step_right = 0;          // in [step]
static int16_t counter_step_left = 0; 		    // in [step]
static int16_t position_to_reach_right = 0;	    // in [step]
static int16_t position_to_reach_left = 0;	    // in [step]
static bool position_right_reached = 0;
static bool position_left_reached = 0;
static thd_motor_pos_args motor_pos_args;

extern messagebus_t bus;

static THD_FUNCTION(ThdDance, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    static uint8_t move_nb = 0;
    static uint8_t old_move_nb = 0;
    systime_t time;
    while(1){
    	if(move_done == true){
    		if ((move_nb == MOVE_BACKWARD) || (move_nb == MOVE_FORWARD)){
    			old_move_nb = move_nb;
    		}
            move_nb = choose_move(old_move_nb);
    		move_done = false;
    		move(move_nb);// pas random pour l'instant
    	}

        time = chVTGetSystemTime();
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

// What to do if new thread started ????? TO DO
static THD_FUNCTION(ThdMotor, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_motor_args *motor_info = arg;
    right_motor_set_speed(motor_info->speed_right);
    left_motor_set_speed(motor_info->speed_left);
    chThdSleepMilliseconds((motor_info->time_s) * 1000);
    right_motor_set_speed(0);
    left_motor_set_speed(0);
    move_done = true;
    chThdExit(0);
}


static THD_FUNCTION(ThdMotorPos, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_motor_pos_args *motor_pos_info = arg;
    motor_set_position(motor_pos_info->position_r, motor_pos_info->position_l, motor_pos_info->speed_r, motor_pos_info->speed_l);
    while ((position_right_reached == 0) || (position_left_reached == 0)){
        if (position_right_reached == 0){
            counter_step_right = right_motor_get_pos();
            if (abs(counter_step_right) >= abs(position_to_reach_right)){
                position_right_reached = 1;
                right_motor_set_speed(0);
            }
        }
        if (position_left_reached == 0){
            counter_step_left = left_motor_get_pos();
            if (abs(counter_step_left) >= abs(position_to_reach_left)){
                position_left_reached = 1;
                left_motor_set_speed(0);
            }
        }
        chThdSleepMilliseconds(10);
    }
    move_done = true;
    chThdExit(0);
}

// SI fonctionne pas cf https://www.chibios.org/dokuwiki/doku.php?id=chibios:documentation:books:rt:kernel_threading TO DO
static THD_FUNCTION(ThdLed, arg) {
 chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_led_args *led_info = arg;
    stm32_gpio_t* gpio;

    while(1){
    	int led = led_info->led;
		wait_onset();
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
    }
    chThdExit(0);
}

static THD_FUNCTION(ThdRGBLed, arg) {
 chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_rgb_led_args *led_info = arg;
    if (led_info->led_play_type == FOLLOW_PITCH){
        set_rgb_led(led_info->led, 0, 0 , 0);
        while (1) {
            chThdSleepMilliseconds(led_info->delay_off);
            choose_and_set_RGB(led_info->led);
            chThdSleepMilliseconds(led_info->delay_on);
            set_rgb_led(led_info->led, 0, 0 , 0);
        }
    } else {
        // TO DO 
    }
    // move_done = true;
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
* @brief Blink LED2
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
* @param colour RGB colour for the LED
* @param play_type Way to use the LED (FOLLOW_PITCH, MANUAL)
*/
void blink_LED2(int iterations, int delay_on, int delay_off, rgb colour, int play_type){
    static thd_rgb_led_args rgb_led_args;
    rgb_led_args.led = LED2;
    rgb_led_args.iterations = iterations;
    rgb_led_args.delay_on = delay_on;
    rgb_led_args.delay_off = delay_off;
    rgb_led_args.colour = colour;
    rgb_led_args.led_play_type = play_type;
    chThdCreateStatic(waThdLedLED2, sizeof(waThdLedLED2), NORMALPRIO, ThdRGBLed, &rgb_led_args);
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
* @brief Blink LED4
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
* @param colour RGB colour for the LED
* @param play_type Way to use the LED (FOLLOW_PITCH, MANUAL)
*/
void blink_LED4(int iterations, int delay_on, int delay_off, rgb colour, int play_type){
    static thd_rgb_led_args rgb_led_args;
    rgb_led_args.led = LED4;
    rgb_led_args.iterations = iterations;
    rgb_led_args.delay_on = delay_on;
    rgb_led_args.delay_off = delay_off;
    rgb_led_args.colour = colour;
    rgb_led_args.led_play_type = play_type;
    chThdCreateStatic(waThdLedLED4, sizeof(waThdLedLED4), NORMALPRIO, ThdRGBLed, &rgb_led_args);
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
* @brief Blink LED6
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
* @param colour RGB colour for the LED
* @param play_type Way to use the LED (FOLLOW_PITCH, MANUAL)
*/
void blink_LED6(int iterations, int delay_on, int delay_off, rgb colour, int play_type){
    static thd_rgb_led_args rgb_led_args;
    rgb_led_args.led = LED6;
    rgb_led_args.iterations = iterations;
    rgb_led_args.delay_on = delay_on;
    rgb_led_args.delay_off = delay_off;
    rgb_led_args.colour = colour;
    rgb_led_args.led_play_type = play_type;
    chThdCreateStatic(waThdLedLED6, sizeof(waThdLedLED6), NORMALPRIO, ThdRGBLed, &rgb_led_args);
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
* @brief Blink LED8
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
* @param colour RGB colour for the LED
* @param play_type Way to use the LED (FOLLOW_PITCH, MANUAL)
*/
void blink_LED8(int iterations, int delay_on, int delay_off, rgb colour, int play_type){
    static thd_rgb_led_args rgb_led_args;
    rgb_led_args.led = LED8;
    rgb_led_args.iterations = iterations;
    rgb_led_args.delay_on = delay_on;
    rgb_led_args.delay_off = delay_off;
    rgb_led_args.colour = colour;
    rgb_led_args.led_play_type = play_type;
    chThdCreateStatic(waThdLedLED8, sizeof(waThdLedLED8), NORMALPRIO, ThdRGBLed, &rgb_led_args);
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
* @brief Choose the colour to use for RGB leds depending on the pitch
*
* @param led_number The rgb_led_name_t of the led to use
*/
void choose_and_set_RGB(rgb_led_name_t led_number){
	if(get_music_amplitude()>SOUND_AMP_MIN){
		uint16_t pitch = get_music_pitch();
		uint8_t r = 255;
		uint8_t g = 255;
		uint8_t b = 255;
		/*if(pitch < COLOR_HZ_RANGE/6){
			g =0;
			b = (pitch * 6 * 255) / COLOR_HZ_RANGE;
		} else if(pitch < 2*COLOR_HZ_RANGE/6){
			r=0;
			g=0;
			b=0;

		} else if(pitch< 3*COLOR_HZ_RANGE/6){
			r = 0;
			g = (pitch * 6 * 255) / COLOR_HZ_RANGE - 2 * 255;
		} else if(pitch < 4*COLOR_HZ_RANGE/6){
			r=0;
			g=0;
			b=0;

		} else if(pitch < 5*COLOR_HZ_RANGE/6){
			r=0;
			g=0;
			b=0;

		} else if(pitch < COLOR_HZ_RANGE){
			r=0;
			g=0;
			b=0;
		}
		set_rgb_led(led_number, r, g , b);*/
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
}

/**
* @brief Choose the move to execute
*
* @param old_move_nb The old move number choosen
*
* @return the value of the move
*/
int choose_move(uint8_t old_move_nb){
	if (is_obstacle() == true){
        return ESCAPE_OBSTACLE;
    } else {
        uint16_t tempo = get_music_tempo();
        uint8_t move = 0;
    	//uint8_t random = 1 + rand() % (MOVE_NB - 1);
        uint8_t random = 1 + rand() % 99; // Get a random number between 1 and 100
        if (tempo < TEMPO_0) {
            if (random < 70) {
                move = FULL_ROTATION;
            } else if (random < 90) {
                move = TURN_AROUND;
            } else if (random < 95) {
                move = DO_NOTHING;
            } else {
                if (old_move_nb == MOVE_FORWARD){
                    move = MOVE_BACKWARD;
                } else {
                    move = MOVE_FORWARD;
                }
            }
        } else if (tempo < TEMPO_1) {
            if (random < 70) {
                move = FULL_ROTATION;
            } else if (random < 90) {
                move = TURN_AROUND;
            } else if (random < 95) {
                move = DO_NOTHING;
            } else {
                if (old_move_nb == MOVE_FORWARD){
                    move = MOVE_BACKWARD;
                } else {
                    move = MOVE_FORWARD;
                }
            }
        } else if (tempo < TEMPO_2) {
            if (random < 70) {
                move = FULL_ROTATION;
            } else if (random < 90) {
                move = TURN_AROUND;
            } else if (random < 95) {
                move = DO_NOTHING;
            } else {
                if (old_move_nb == MOVE_FORWARD){
                    move = MOVE_BACKWARD;
                } else {
                    move = MOVE_FORWARD;
                }
            }
        } else if (tempo < TEMPO_3) {
            if (random < 70) {
                move = TURN_AROUND;
            } else if (random < 90) {
                move = FULL_ROTATION;
            } else if (random < 95) {
                if (old_move_nb == MOVE_FORWARD){
                    move = MOVE_BACKWARD;
                } else {
                    move = MOVE_FORWARD;
                }
            } else {
                move = DO_NOTHING;
            }            
        } else if (tempo < TEMPO_4) {
            if (random < 70) {
                move = TURN_AROUND;
            } else if (random < 90) {
                move = FULL_ROTATION;
            } else if (random < 95) {
                if (old_move_nb == MOVE_FORWARD){
                    move = MOVE_BACKWARD;
                } else {
                    move = MOVE_FORWARD;
                }
            } else {
                move = DO_NOTHING;
            }
        } else if (tempo < TEMPO_5) {
            if (random < 70) {
                if (old_move_nb == MOVE_FORWARD){
                    move = MOVE_BACKWARD;
                } else {
                    move = MOVE_FORWARD;
                }
            } else if (random < 90) {
                move = TURN_AROUND;
            } else if (random < 95) {
                move = FULL_ROTATION;
            } else {
                move = DO_NOTHING;
            }            
        } else {
            if (random < 70) {
                if (old_move_nb == MOVE_FORWARD){
                    move = MOVE_BACKWARD;
                } else {
                    move = MOVE_FORWARD;
                }
            } else if (random < 90) {
                move = TURN_AROUND;
            } else if (random < 95) {
                move = DO_NOTHING;
            } else {
                move = FULL_ROTATION;
            }
        }
    	chprintf((BaseSequentialStream *)&SD3, " move: %d \n", move);
        return (move);
    }
}

/**
* @brief Initializes the choreography
*
* @return 0 if no error
*/
int choreography_init(){
    //chThdCreateStatic(waThdDance, sizeof(waThdDance), NORMALPRIO, ThdDance, NULL);
    //start_leds();
    motor_pos_args.position_r = PERIMETER_EPUCK/2;
    motor_pos_args.position_l = PERIMETER_EPUCK/2;
    motor_pos_args.speed_r = -MOTOR_MEDIUM_SPEED;
    motor_pos_args.speed_l = MOTOR_MEDIUM_SPEED;
    chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);

    spi_comm_start();
    start_leds();
    return 0;
}

/**
* @brief Try to escape the nearest obstacle
*/
void escape_obstacle(){
    update_obstacle_array(obstacle);
}

/**
* @brief DO a full rotation of the epuck
*/
void full_rotation(){
    motor_args.time_s = DEFAULT_MOVE_TIME_S;
	motor_args.speed_left = -MOTOR_MEDIUM_SPEED;
	motor_args.speed_right = +MOTOR_MEDIUM_SPEED;
    chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief Control motors in position
*
* @param position_r Position to go to for the right motor
* @param position_l Position to go to for the left motor
* @param speed_r Speed of the right motor
* @param speed_l Speed of the left motor
*/
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l){
	//reinit global variable
	counter_step_left = 0;
	counter_step_right = 0;

    position_right_reached = 0;
    position_left_reached = 0;

	//Set global variable with position to reach in step
	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;

    right_motor_set_speed(speed_r);
    left_motor_set_speed(speed_l);
}

/**
* @brief Call the right functions to execute the chosen move
*
* @param move_chosen the ID of the move to execute
*/
void move(int move_chosen){
    switch (move_chosen)
    {
    case ESCAPE_OBSTACLE:
        escape_obstacle();
        break;
    case MOVE_FORWARD:
        move_forward(DEFAULT_MOVE_TIME_S, MOTOR_MEDIUM_SPEED);
        break;
    case MOVE_BACKWARD:
        move_backward(DEFAULT_MOVE_TIME_S, MOTOR_MEDIUM_SPEED);
        break;
    case FULL_ROTATION:
        full_rotation();
        break;
    case TURN_AROUND:
        turn_around();
        break;
    case DO_NOTHING:
        do_nothing(DEFAULT_MOVE_TIME_S);
        break;
    default:
        break;
    }
}

/**
* @brief Move the epuck backward
*
* @param time_s Time in seconds to move backward
* @param speed Speed chosen to move
*/
void move_backward(uint8_t time_s, int16_t speed){
	motor_args.time_s = time_s;
	motor_args.speed_left = -speed;
	motor_args.speed_right = -speed;
    chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief Move the epuck forward
*
* @param time_s Time in seconds to move forward
* @param speed Speed chosen to move
*/
void move_forward(uint8_t time_s, int16_t speed){
	motor_args.time_s = time_s;
	motor_args.speed_left = speed;
	motor_args.speed_right = speed;
    chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief Start RGB leds blinking
*/
void start_leds(){
    rgb initial_rgb = {0, 0 ,0};
    blink_LED2(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF, initial_rgb, FOLLOW_PITCH);
    blink_LED4(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF, initial_rgb, FOLLOW_PITCH);
    blink_LED6(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF, initial_rgb, FOLLOW_PITCH);
    blink_LED8(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF, initial_rgb, FOLLOW_PITCH);

    blink_LED1(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
    blink_LED3(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
    blink_LED5(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
    blink_LED7(1, DEFAULT_BLINK_DELAY_ON, DEFAULT_BLINK_DELAY_OFF);
}

/**
* @brief Make the epuck turn around
*/
void turn_around(){
    motor_args.time_s = DEFAULT_MOVE_TIME_S / 2;
	motor_args.speed_left = -MOTOR_MEDIUM_SPEED;
	motor_args.speed_right = +MOTOR_MEDIUM_SPEED;
    chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief Make the epuck do nothing
*
* @param time_s Time to do nothing in seconds
*/
void do_nothing(uint8_t time_s){
	motor_args.time_s = time_s;
	motor_args.speed_left = 0;
	motor_args.speed_right = 0;
    chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}
