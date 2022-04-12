#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {



    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	systime_t time = 0;
    while(1){
    	time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
		chThdSleepMilliseconds(12);
		time = (int)chVTGetSystemTime() - time;
		//chprintf((BaseSequentialStream *)& SD3,"time= %ld\n" , time);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	int k =0;
	int length_px;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
        img_buff_ptr = dcmi_get_last_image_ptr();
		/*1
		*	To complete
		*/
		for(int i=0; i<IMAGE_BUFFER_SIZE ; i++){

			image[i] = ((*(img_buff_ptr + 2*i)) & 0b11111000);
		}
		if (k==4){
			//SendUint8ToComputer(image, IMAGE_BUFFER_SIZE); //show image
			length_px = line_detection(image);
			distance_cm = (1/(float)length_px-0.0009153)/0.00066;
			//chprintf((BaseSequentialStream *)& SD3,"\n distance= %.2f", distance_cm);
			k=0;
		}
		k++;
    }
}
int line_detection(uint8_t image[IMAGE_BUFFER_SIZE]){
	int x0 = -1;
	int x1 = -1;
	int length = 0;
	uint8_t threshold = 0;
	uint16_t image_sum = 0;
	uint8_t average = 0;

	for(int i=0; i<IMAGE_BUFFER_SIZE ; i++){
		image_sum += image[i];
	}
	average = image_sum/IMAGE_BUFFER_SIZE;

	threshold = average - 0.42*(average);

	//chprintf((BaseSequentialStream *)& SD3,"average: %u \n" , );

	//chprintf((BaseSequentialStream *)& SD3,"Image: ");
	for(int i=0; i<IMAGE_BUFFER_SIZE ; i++){
		//chprintf((BaseSequentialStream *)& SD3,"%d " ,image[i]);
		if(image[i] < threshold && x0 == -1){
			x0 = i;
		} else if (image[i] > threshold && x0 != -1){
			x1 = i;
			break;
		}
	}
	if (x0 != -1 && x1 != -1){
		length = x1- x0;
	}
	chprintf((BaseSequentialStream *)& SD3,"\n threshold= %u, x0= %d, x1= %d, length= %d\n" ,threshold , x0, x1, length);
	return length;
}

float get_distance_cm(void){

	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
