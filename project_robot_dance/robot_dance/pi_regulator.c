#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_limit = 1100;

    float commande = 0;
    float erreur = 0;
    float consigne = 10;
    float Kp = 100;
    float Ki = 0.00;
    float somme_erreurs = 0;
    float mesure = 0;
    float somme_erreurs_max = 5000;
    float old_erreur;
    //float commande = Kp * erreur + Ki * somme_erreur;

    while(1){
        time = chVTGetSystemTime();

        /*
		*	To complete
		*/
        
        mesure = get_distance_cm();
        erreur = consigne - mesure;
        somme_erreurs += erreur;
        if (somme_erreurs > somme_erreurs_max){
        	somme_erreurs = somme_erreurs_max;
        } else if (somme_erreurs < -somme_erreurs_max){
        	somme_erreurs = -somme_erreurs_max;
        }
        commande = Kp * erreur + Ki * somme_erreurs;

        if (erreur*erreur < 0.2){
        	commande = 0;
        }

        chprintf((BaseSequentialStream *)& SD3,"mesure= %.2f, somme des erreurs %.2f, commande %.2f \n", mesure, somme_erreurs, commande );

        //applies the speed from the PI regulator
        if(commande > speed_limit){
        	speed = -speed_limit;
        } else if(commande < -speed_limit){
        	speed = speed_limit;
        } else {
        	speed = -commande;
        }

        right_motor_set_speed(speed);
        left_motor_set_speed(speed);
        //50Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
