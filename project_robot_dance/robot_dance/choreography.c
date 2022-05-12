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
#define BODY_BLINK_DELAY 200
#define DEFAULT_BLINK_DELAY_ON 50
#define DEFAULT_BLINK_DELAY_OFF 50
#define DEFAULT_BLINK_ITERATION 10
#define DEFAULT_ROTATION_ITERATION 1
#define DEFAULT_MOVE_ESCAPE_TIME_MS 500
#define DEFAULT_MOVE_TIME_MS 1000
#define MOTOR_MAX_SPEED 2200
#define MOTOR_HIGH_SPEED 1800
#define MOTOR_MEDIUM_SPEED 1100
#define MOTOR_LOW_SPEED 500
#define MOTOR_TURTLE_SPEED 100
#define MOTOR_MIN_SPEED 100

#define SOUND_AMP_MIN 500

#define TEMPO_0 60
#define TEMPO_1 80
#define TEMPO_2 100
#define TEMPO_3 120
#define TEMPO_4 140
#define TEMPO_5 160

#define TEMPO_SPEED_COEF 10

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define POSITION_NOT_REACHED	0
#define POSITION_REACHED       	1

#define COLOR_HZ_RANGE 1000

/**
* @brief Structure définissant les différents move
*/
typedef enum {
	ESCAPE_OBSTACLE,
    MOVE_FORWARD,
    MOVE_FORWARD_IN_TEMPO,
    MOVE_BACKWARD,
    MOVE_BACKWARD_IN_TEMPO,
    FULL_ROTATION,
    TURN_AROUND,
    DO_NOTHING,
    HALF_MOON,
    FULL_MOON,
} move_type;

/**
* @brief Structure définissant les modes de gestion des LEDS
*/
typedef enum {
	FOLLOW_PITCH,
    MANUAL,
} led_play_type;

/**
* @brief Structure définissant une couleur en RGB
*/
typedef struct rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb;

/**
* @brief Structure pour passer en arguments au ThdLed
*/
typedef struct thd_led_args {
    uint8_t led;
    uint8_t iterations;
    uint16_t delay_on;
    uint16_t delay_off;
} thd_led_args;

/**
* @brief Structure pour passer en arguments au ThdRGBLed
*/
typedef struct thd_rgb_led_args {
    rgb_led_name_t led;
    uint8_t iterations;
    uint16_t delay_on;
    uint16_t delay_off;
    rgb colour;
    led_play_type led_play_type;
} thd_rgb_led_args;

/**
* @brief Structure pour passer en arguments au ThdMotor
*/
typedef struct thd_motor_args {
    uint16_t time_ms;
    int16_t speed_left;
    int16_t speed_right;
} thd_motor_args;

/**
* @brief Structure pour passer en arguments au ThdMotorPos
*/
typedef struct thd_motor_pos_args {
    float position_r;
    float position_l;
    int16_t speed_right;
    int16_t speed_left;
} thd_motor_pos_args;

void blink_LED1(uint8_t iterations, uint16_t delay_on, uint16_t delay_off);
void blink_LED2(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type);
void blink_LED3(uint8_t iterations, uint16_t delay_on, uint16_t delay_off);
void blink_LED4(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type);
void blink_LED5(uint8_t iterations, uint16_t delay_on, uint16_t delay_off);
void blink_LED6(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type);
void blink_LED7(uint8_t iterations, uint16_t delay_on, uint16_t delay_off);
void blink_LED8(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type);
void blink_LED_BODY(uint8_t iterations, uint16_t delay_on, uint16_t delay_off);
void blink_LED_FRONT(uint8_t iterations, uint16_t delay_on, uint16_t delay_off);
void cancel_moves(void);
void choose_and_set_RGB(rgb_led_name_t *led_number);
uint16_t choose_motor_speed(void);
int choose_move(uint8_t old_move_nb);
void do_nothing(uint16_t time_ms);
void escape_obstacle(void);
void full_rotation(void);
void motor_set_position(float position_r, float position_l, int16_t speed_right, int16_t speed_left);
void move(int move_chosen);
void move_backward(uint16_t time_ms, int16_t speed);
void move_backward_in_tempo(int16_t speed);
void move_forward(uint16_t time_ms, int16_t speed);
void move_forward_in_tempo(int16_t speed);
void move_full_moon();
void move_half_moon();
void start_leds(void);
void turn_around(void);
void update_RGB_delay(uint16_t *delay_on, uint16_t *delay_off);

static THD_WORKING_AREA(waThdDance, 1024);
static THD_WORKING_AREA(waThdEscape, 2048);
static THD_WORKING_AREA(waThdMotor, 1024);
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

static int32_t counter_step_right = 0;          // in [step]
static int32_t counter_step_left = 0; 		    // in [step]
static int32_t position_to_reach_right = 0;	    // in [step]
static int32_t position_to_reach_left = 0;	    // in [step]
static bool position_right_reached = 0;
static bool position_left_reached = 0;
static thd_motor_pos_args motor_pos_args;

static thread_t* pointer_thread_motor_pos = NULL;
static thread_t* pointer_thread_motor = NULL;
static bool is_escaping = false;

/**
* @brief Main thread for the choreography: chooses move, execute the move and check if obstacle
**/
static THD_FUNCTION(ThdDance, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    static uint8_t move_nb = 0;
    static uint8_t old_move_nb = 0;
    while(1){
    	if((move_done == true) && (is_escaping == false)){
    		pointer_thread_motor_pos = NULL;
    		pointer_thread_motor = NULL;
            if ((move_nb == MOVE_BACKWARD) || (move_nb == MOVE_FORWARD) || (move_nb == MOVE_BACKWARD_IN_TEMPO) || (move_nb == MOVE_FORWARD_IN_TEMPO)){
            	old_move_nb = move_nb;
            }
            move_nb = choose_move(old_move_nb);
            // chprintf((BaseSequentialStream *)&SD3, "move nb: %d\n", move_nb);
            move_done = false;
            move(move_nb);
    	}
        chThdSleepMilliseconds(10);
    }
}

/**
* @brief Thread to escape obstacle
**/
static THD_FUNCTION(ThdEscape, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t motor_speed;
    chThdSleepMilliseconds(2000);
    while (1) {
        if (is_obstacle() == true) {
            is_escaping = true;
            move_done = false;
            cancel_moves();
            chThdSleepMilliseconds(50);
            chprintf((BaseSequentialStream *)&SD3, "in the if of escape\n");
            do {
                escape_obstacle();
                while (move_done == false) {
                    chThdSleepMilliseconds(10);
                }
                motor_speed = choose_motor_speed();
                move_done = false;
                move_forward(DEFAULT_MOVE_ESCAPE_TIME_MS, motor_speed);
                while (move_done == false) {
                	chThdSleepMilliseconds(10);
                }
                pointer_thread_motor_pos = NULL;
                pointer_thread_motor = NULL;
            } while (is_obstacle() == true);
            move_done = true;
            is_escaping = false;
        }
        chThdSleepMilliseconds(20);
    }
}

/**
* @brief Thread for speed controled motor actions
**/
static THD_FUNCTION(ThdMotor, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_motor_args *motor_info = arg;
    right_motor_set_speed(motor_info->speed_right);
    left_motor_set_speed(motor_info->speed_left);
    if (motor_info->time_ms > 1000) {
        uint16_t sleep_time = (motor_info->time_ms) / 100;
        uint8_t i = 0;
        while(i < 100){
            if(chThdShouldTerminateX()){
                chThdExit(0);
            } else {
                chThdSleepMilliseconds(sleep_time);
            }
            i++;
        }
    } else {
        uint16_t sleep_time = (motor_info->time_ms) / 10;
        uint8_t i = 0;
        while(i < 10){
            if(chThdShouldTerminateX()){
                chThdExit(0);
            } else {
                chThdSleepMilliseconds(sleep_time);
            }
            i++;
        }
    }

    right_motor_set_speed(0);
    left_motor_set_speed(0);
    move_done = true;
    chThdExit(0);
}
/**
* @brief Thread for position controled motor actions
**/
static THD_FUNCTION(ThdMotorPos, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_motor_pos_args *motor_pos_info = arg;
    motor_set_position(motor_pos_info->position_r, motor_pos_info->position_l, motor_pos_info->speed_right, motor_pos_info->speed_left);
    while ((position_right_reached == false) || (position_left_reached == false)){
    	if(chThdShouldTerminateX()){
    		chThdExit(0);
    	} else {
			if (position_right_reached == false){
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
    	}
        chThdSleepMilliseconds(20);
    }
    move_done = true;
    chThdExit(0);
}

/**
* @brief Thread to control a normal LED
**/
static THD_FUNCTION(ThdLed, arg) {
	chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_led_args *led_info = arg;
    stm32_gpio_t* gpio;
    uint8_t led = led_info->led;
    uint16_t delay_off = led_info->delay_off;
    uint16_t delay_on = led_info->delay_on;
    uint8_t iterations = led_info->iterations;
    if (led == GPIOB_LED_BODY){
    	gpio = GPIOB;
    } else {
    	gpio = GPIOD;
    }
    while(1){
    	bool on = 0;
    	if(gpio == GPIOB){
    		on = 1;
    		wait_big_onset();
    	} else{
    		wait_onset();
    	}
    	palWritePad(gpio, led, !on);
		for (int i = 0; i < iterations; i ++){
			palWritePad(gpio, led, on);
			chThdSleepMilliseconds(delay_on);
			palWritePad(gpio, led, !on);
			chThdSleepMilliseconds(delay_off);
		}
    }
    chThdExit(0);
}

/**
* @brief Thread to control a RGB LED
**/
static THD_FUNCTION(ThdRGBLed, arg) {
 chRegSetThreadName(__FUNCTION__);
    (void)arg;
    thd_rgb_led_args *led_info = arg;
    uint16_t delay_off = led_info->delay_off;
    uint16_t delay_on = led_info->delay_on;
    rgb_led_name_t led = led_info->led;
    uint8_t iterations = led_info->iterations;
    if (led_info->led_play_type == FOLLOW_PITCH){
        set_rgb_led(led, 0, 0 , 0);
        while (1) {
        	if(state_tempo_update()){
        		wait_tempo_update();
				reset_tempo_update();
        	}
            update_RGB_delay(&delay_on, &delay_off);
            //chprintf((BaseSequentialStream *)&SD3, " delayoff: %d ms: %d\n", delay_off, get_music_interval());
            chThdSleepMilliseconds(delay_off);
            choose_and_set_RGB(&led);
            chThdSleepMilliseconds(delay_on);
            set_rgb_led(led, 0, 0 , 0);
        }
    } else {
        set_rgb_led(led, 0, 0 , 0);
        for (int i = 0; i < iterations; i++) {
            chThdSleepMilliseconds(delay_off);
            choose_and_set_RGB(&led);
            chThdSleepMilliseconds(delay_on);
            set_rgb_led(led, 0, 0 , 0);
        }
    }
    move_done = true;
    chThdExit(0);
}

// Those LED functions are needed in order to avoid issues of changed pointers values before value storage in the thread
/**
* @brief Blink LED1
*
* @param iterations number of iterations to execute 
* @param delay_on Number of ms to set the LED on
* @param delay_off Number of ms to set the LED off
*/
void blink_LED1(uint8_t iterations, uint16_t delay_on, uint16_t delay_off){
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
void blink_LED2(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type){
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
void blink_LED3(uint8_t iterations, uint16_t delay_on, uint16_t delay_off){
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
void blink_LED4(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type){
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
void blink_LED5(uint8_t iterations, uint16_t delay_on, uint16_t delay_off){
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
void blink_LED6(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type){
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
void blink_LED7(uint8_t iterations, uint16_t delay_on, uint16_t delay_off){
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
void blink_LED8(uint8_t iterations, uint16_t delay_on, uint16_t delay_off, rgb colour, int play_type){
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
void blink_LED_BODY(uint8_t iterations, uint16_t delay_on, uint16_t delay_off){
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
void blink_LED_FRONT(uint8_t iterations, uint16_t delay_on, uint16_t delay_off){
    static thd_led_args led_args;
    led_args.led = GPIOD_LED_FRONT;
    led_args.iterations = iterations;
    led_args.delay_on = delay_on;
    led_args.delay_off = delay_off;
    chThdCreateStatic(waThdLedFRONT_LED, sizeof(waThdLedFRONT_LED), NORMALPRIO, ThdLed, &led_args);
}

/**
* @brief If a motor thread exists, kills it
*/
void cancel_moves(){
    if (pointer_thread_motor_pos != NULL){
		chThdTerminate(pointer_thread_motor_pos);
		pointer_thread_motor_pos = NULL;
		chprintf((BaseSequentialStream *)&SD3, "terminate_pose \n");
	}
	if (pointer_thread_motor != NULL){
		chThdTerminate(pointer_thread_motor);
		pointer_thread_motor = NULL;
		chprintf((BaseSequentialStream *)&SD3, "ex-terminate \n");
	}
}

/**
* @brief Choose the colour to use for RGB leds depending on the pitch
*
* @param led_number The rgb_led_name_t of the led to use
*/
void choose_and_set_RGB(rgb_led_name_t *led_number){
	if(get_music_amplitude()>SOUND_AMP_MIN){
		uint16_t pitch = get_music_pitch();
		uint8_t r = 255;
		uint8_t g = 255;
		uint8_t b = 255;
		if(pitch < COLOR_HZ_RANGE/6){
			b =0;
			g = (pitch * 6 * 255)/COLOR_HZ_RANGE;
		} else if(pitch < 2 * COLOR_HZ_RANGE/6){
			r=-(pitch * 6 * 255)/COLOR_HZ_RANGE + 2 * 255;
			b=0;

		} else if(pitch < 3 * COLOR_HZ_RANGE/6){
			r = 0;
			b = (pitch * 6 * 255)/COLOR_HZ_RANGE - 2 * 255;
		} else if(pitch < 4 * COLOR_HZ_RANGE/6){
			r=0;
			g=-(pitch * 6 * 255)/COLOR_HZ_RANGE + 4 * 255;

		} else if(pitch < 5*COLOR_HZ_RANGE/6){
			r=(pitch * 6 * 255) / COLOR_HZ_RANGE - 4 * 255;
			g=0;

		} else if(pitch < COLOR_HZ_RANGE){
			g=0;
			b=-(pitch * 6 * 255)/COLOR_HZ_RANGE + 6 * 255;
		}
		//chprintf((BaseSequentialStream *)&SD3, " r:%d, g: %d, b: %d\n", r, g, b);
		set_rgb_led(*led_number, r, g , b);
	}
}

/**
* @brief Choose motor speed according to tempo
*
* @return the speed chosen for the motors
*/
uint16_t choose_motor_speed(){
    uint8_t tempo = get_music_tempo();
    uint16_t speed = tempo * TEMPO_SPEED_COEF;
    if (speed > MOTOR_MAX_SPEED){
        speed = MOTOR_MAX_SPEED;
    } if (speed < MOTOR_MIN_SPEED){
        speed   = MOTOR_MIN_SPEED;
    }
    return speed;
}

/**
* @brief Choose the move to execute
*
* @param old_move_nb The old move number choosen
*
* @return the value of the move
*/
int choose_move(uint8_t old_move_nb){
        uint8_t tempo = get_music_tempo();
        uint8_t move = 0;
        uint8_t random = 1 + rand() % 99;
        if (tempo < TEMPO_0) {
            if (random < 70) {
                move = FULL_MOON;
            } else if (random < 90) {
                move = HALF_MOON;
            } else if (random < 95) {
                move = FULL_ROTATION;
            } else {
                if (old_move_nb == MOVE_FORWARD_IN_TEMPO){
                    move = MOVE_BACKWARD_IN_TEMPO;
                } else {
                    move = MOVE_FORWARD_IN_TEMPO;
                }
            }
        } else if (tempo < TEMPO_1) {
            if (random < 70) {
                move = FULL_MOON;
            } else if (random < 90) {
                move = HALF_MOON;
            } else if (random < 95) {
                move = FULL_MOON;
            } else {
                if (old_move_nb == MOVE_FORWARD_IN_TEMPO){
                    move = MOVE_BACKWARD_IN_TEMPO;
                } else {
                    move = MOVE_FORWARD_IN_TEMPO;
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
                if (old_move_nb == MOVE_FORWARD_IN_TEMPO){
                    move = MOVE_BACKWARD_IN_TEMPO;
                } else {
                    move = MOVE_FORWARD_IN_TEMPO;
                }
            }
        } else if (tempo < TEMPO_3) {
            if (random < 70) {
                move = TURN_AROUND;
            } else if (random < 90) {
                move = FULL_ROTATION;
            } else if (random < 95) {
                if (old_move_nb == MOVE_FORWARD_IN_TEMPO){
                    move = MOVE_BACKWARD_IN_TEMPO;
                } else {
                    move = MOVE_FORWARD_IN_TEMPO;
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
                if (old_move_nb == MOVE_FORWARD_IN_TEMPO){
                    move = MOVE_BACKWARD_IN_TEMPO;
                } else {
                    move = MOVE_FORWARD_IN_TEMPO;
                }
            } else {
                move = DO_NOTHING;
            }
        } else if (tempo < TEMPO_5) {
            if (random < 70) {
                if (old_move_nb == MOVE_FORWARD_IN_TEMPO){
                    move = MOVE_BACKWARD_IN_TEMPO;
                } else {
                    move = MOVE_FORWARD_IN_TEMPO;
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
                if (old_move_nb == MOVE_FORWARD_IN_TEMPO){
                    move = MOVE_BACKWARD_IN_TEMPO;
                } else {
                    move = MOVE_FORWARD_IN_TEMPO;
                }
            } else if (random < 90) {
                move = TURN_AROUND;
            } else if (random < 95) {
                move = DO_NOTHING;
            } else {
                move = FULL_ROTATION;
            }
        }
        return (move);
}

/**
* @brief Initializes the choreography
*
* @return 0 if no error
*/
int choreography_init(){
    motors_init();
	detection_init();
	signals_processing_init();
    //chThdCreateStatic(waThdDance, sizeof(waThdDance), NORMALPRIO, ThdDance, NULL);
    //chThdCreateStatic(waThdEscape, sizeof(waThdEscape), NORMALPRIO+2, ThdEscape, NULL);
    spi_comm_start();
    start_leds();
    return 0;
}

/**
* @brief Make the epuck do nothing
*
* @param time_ms Time to do nothing in milliseconds
*/
void do_nothing(uint16_t time_ms){
	motor_args.time_ms = time_ms;
	motor_args.speed_left = 0;
	motor_args.speed_right = 0;
	pointer_thread_motor = chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief Try to escape the nearest obstacle
*/
void escape_obstacle(){
    move_done = false; // Nécéssaire ? 
	uint16_t motor_speed = choose_motor_speed();
    update_obstacle_array(obstacle);;
    if (obstacle[0] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8 + PERIMETER_EPUCK/32;
        motor_pos_args.position_l = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8 + PERIMETER_EPUCK/32;
        motor_pos_args.speed_right = motor_speed;
        motor_pos_args.speed_left = -motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else if (obstacle[1] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8;
        motor_pos_args.position_l = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8;
        motor_pos_args.speed_right = motor_speed;
        motor_pos_args.speed_left = -motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else if (obstacle[2] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/4;
        motor_pos_args.position_l = PERIMETER_EPUCK/4;
        motor_pos_args.speed_right = motor_speed;
        motor_pos_args.speed_left = -motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else if (obstacle[3] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/16;
        motor_pos_args.position_l = PERIMETER_EPUCK/16;
        motor_pos_args.speed_right = motor_speed;
        motor_pos_args.speed_left = -motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else if (obstacle[4] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/16;
        motor_pos_args.position_l = PERIMETER_EPUCK/16;
        motor_pos_args.speed_right = -motor_speed;
        motor_pos_args.speed_left = motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else if (obstacle[5] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/4;
        motor_pos_args.position_l = PERIMETER_EPUCK/4;
        motor_pos_args.speed_right = -motor_speed;
        motor_pos_args.speed_left = motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else if (obstacle[6] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8;
        motor_pos_args.position_l = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8;
        motor_pos_args.speed_right = -motor_speed;
        motor_pos_args.speed_left = motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else if (obstacle[7] == true){
        motor_pos_args.position_r = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8 + PERIMETER_EPUCK/32;
        motor_pos_args.position_l = PERIMETER_EPUCK/4 + PERIMETER_EPUCK/8 + PERIMETER_EPUCK/32;
        motor_pos_args.speed_right = -motor_speed;
        motor_pos_args.speed_left = motor_speed;
        pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
    } else {
    	chprintf((BaseSequentialStream *)&SD3, "erreur no obstacle\n");
    	move_done = true;
    }
}

/**
* @brief DO a full rotation of the epuck
*/
void full_rotation(){
    uint16_t motor_speed = choose_motor_speed();
    motor_pos_args.position_r = PERIMETER_EPUCK;
    motor_pos_args.position_l = PERIMETER_EPUCK;
	motor_pos_args.speed_left = -motor_speed;
	motor_pos_args.speed_right = motor_speed;
	pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
}

/**
* @brief Control motors in position
*
* @param position_r Position to go to for the right motor
* @param position_l Position to go to for the left motor
* @param speed_right Speed of the right motor
* @param speed_left Speed of the left motor
*/
void motor_set_position(float position_r, float position_l, int16_t speed_right, int16_t speed_left){
	right_motor_set_pos(0);
	left_motor_set_pos(0);
	counter_step_left = 0;
	counter_step_right = 0;
    position_right_reached = 0;
    position_left_reached = 0;
	position_to_reach_left = position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	position_to_reach_right = position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;
    right_motor_set_speed(speed_right);
    left_motor_set_speed(speed_left);
}

/**
* @brief Call the right functions to execute the chosen move
*
* @param move_chosen the ID of the move to execute
*/
void move(int move_chosen){
    uint16_t motor_speed = choose_motor_speed();
    switch (move_chosen)
    {
    case MOVE_FORWARD:
        move_forward(DEFAULT_MOVE_TIME_MS, motor_speed);
        break;
    case MOVE_BACKWARD:
        move_backward(DEFAULT_MOVE_TIME_MS, motor_speed);
        break;
    case FULL_ROTATION:
        full_rotation();
        break;
    case TURN_AROUND:
        turn_around();
        break;
    case DO_NOTHING:
        do_nothing(DEFAULT_MOVE_TIME_MS);
        break;
    case HALF_MOON:
        move_half_moon();
    case FULL_MOON:
        move_full_moon(); 
    case MOVE_BACKWARD_IN_TEMPO:
        move_backward_in_tempo(motor_speed);
    case MOVE_FORWARD_IN_TEMPO :
        move_forward_in_tempo(motor_speed);
    default:
        break;
    }
}

/**
* @brief Move the epuck backward
*
* @param time_ms Time in milliseconds to move backward
* @param speed Speed chosen to move
*/
void move_backward(uint16_t time_ms, int16_t speed){
	motor_args.time_ms = time_ms;
	motor_args.speed_left = -speed;
	motor_args.speed_right = -speed;
	pointer_thread_motor = chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

void move_backward_in_tempo(int16_t speed){
    int16_t time_ms = get_music_interval();
    motor_args.time_ms = time_ms;
	motor_args.speed_left = speed;
	motor_args.speed_right = speed;
	pointer_thread_motor =chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

/**
* @brief Move the epuck forward
*
* @param time_ms Time in milliseconds to move forward
* @param speed Speed chosen to move
*/
void move_forward(uint16_t time_ms, int16_t speed){
	motor_args.time_ms = time_ms;
	motor_args.speed_left = speed;
	motor_args.speed_right = speed;
	pointer_thread_motor =chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

void move_forward_in_tempo(int16_t speed){
    int16_t time_ms = get_music_interval();
    motor_args.time_ms = time_ms;
	motor_args.speed_left = speed;
	motor_args.speed_right = speed;
	pointer_thread_motor =chThdCreateStatic(waThdMotor, sizeof(waThdMotor), NORMALPRIO, ThdMotor, &motor_args);
}

void move_full_moon(){
    uint16_t motor_speed = choose_motor_speed();
    motor_pos_args.position_r = PERIMETER_EPUCK * 4;
    motor_pos_args.position_l = PERIMETER_EPUCK * 2;
	motor_pos_args.speed_left = motor_speed / 2;
	motor_pos_args.speed_right = motor_speed;
	pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
}

void move_half_moon(){
    uint16_t motor_speed = choose_motor_speed();
    motor_pos_args.position_r = PERIMETER_EPUCK * 2;
    motor_pos_args.position_l = PERIMETER_EPUCK;
	motor_pos_args.speed_left = motor_speed / 2;
	motor_pos_args.speed_right = motor_speed;
	pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
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
    blink_LED_BODY(1, BODY_BLINK_DELAY, BODY_BLINK_DELAY);
}

/**
* @brief Make the epuck turn around
*/
void turn_around(){
    uint16_t motor_speed = choose_motor_speed();
    motor_pos_args.position_r = PERIMETER_EPUCK/2;
    motor_pos_args.position_l = PERIMETER_EPUCK/2;
	motor_pos_args.speed_left = -motor_speed;
	motor_pos_args.speed_right = motor_speed;
	pointer_thread_motor_pos = chThdCreateStatic(waThdMotorPos, sizeof(waThdMotorPos), NORMALPRIO, ThdMotorPos, &motor_pos_args);
}

/**
* @brief Update the interval between LED blinks depending on music tempo
*
* @param delay_on Delay, for the LED to be on, to update
* @param delay_ff Delay, for the LED to be off, to update
*/
void update_RGB_delay(uint16_t *delay_on, uint16_t *delay_off){
    uint16_t delay = get_music_interval();
    *delay_on = delay/2;
    *delay_off = delay/2;
}
