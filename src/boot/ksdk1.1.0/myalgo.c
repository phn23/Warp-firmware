#include <stdio.h>
#include <math.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "warp.h"
#include "myalgo.h"
#include "devMMA8451Q.h"

int16_t x_acceleration;
int16_t y_acceleration;
int16_t z_acceleration;


// TODO: NOT ENTIRELY SURE WHETHER THIS SHOULD BE HERE
extern volatile WarpI2CDeviceState	deviceMMA8451QState; 


/****************************************************************
See flowchart for logics
****************************************************************/
// defined iin .h
// #define threshold_tilt_angle 3000 // 3000 / 100 = 30 degrees

bool tilt_angle_trigger(){

    get_acceleration(&x_acceleration, &y_acceleration, &z_acceleration);

    int tilt_front; // x, z
    int tilt_side; // y, z

    tilt_front = abs(atan2(z_acceleration, x_acceleration) * 18000 / M_PI);
    tilt_side = abs(atan2(z_acceleration, y_acceleration) * 18000 / M_PI);

    
    return (tilt_front > threshold_tilt_angle || tilt_side > threshold_tilt_angle);
}

bool flip_classifier(int tilt_front_count,int tilt_side_count, int tilt_count_threshold){
    return (tilt_front_count > tilt_count_threshold || tilt_side_count > tilt_count_threshold);        
}


/************************************************************************
READ FROM ACCELEROMETER
************************************************************************/
/* Data registers: 0x01 OUT_X_MSB, 0x02 OUT_X_LSB, 0x03 OUT_Y_MSB, 0x04 OUT_Y_LSB, 0x05 OUT_Z_MSB, 0x06 OUT_Z_LSB
These registers contain the X-axis, Y-axis, and Z-axis, and 14-bit output sample data expressed as 2's complement numbers. 
inputs are pointers as these values are to be modified */
void get_acceleration(int16_t* x_acc, int16_t* y_acc, int16_t* z_acc){
    int16_t readSensorRegisterValueCombined_x;
    int16_t readSensorRegisterValueCombined_y;
    int16_t readSensorRegisterValueCombined_z;

    // all 3 dim
    readSensorRegisterMMA8451Q(0x01, 6);

    // x
	readSensorRegisterValueCombined_x = (( deviceMMA8451QState.i2cBuffer[0] & 0xFF) << 6) | ( deviceMMA8451QState.i2cBuffer[1] >> 2);
	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	*x_acc = (readSensorRegisterValueCombined_x ^ (1 << 13)) - (1 << 13);
    

    // y
	readSensorRegisterValueCombined_y = (( deviceMMA8451QState.i2cBuffer[2] & 0xFF) << 6) | ( deviceMMA8451QState.i2cBuffer[3] >> 2);
	*y_acc = (readSensorRegisterValueCombined_y ^ (1 << 13)) - (1 << 13);
    
    // z
    readSensorRegisterValueCombined_z = (( deviceMMA8451QState.i2cBuffer[4] & 0xFF) << 6) | ( deviceMMA8451QState.i2cBuffer[5] >> 2);
	*z_acc = (readSensorRegisterValueCombined_z ^ (1 << 13)) - (1 << 13);
}
/************************************************************************
DONE READING FROM ACCELEROMETER
************************************************************************/
	


/************************************************************************
THE BIG WORKING LOOP
************************************************************************/

int normal_loop(){
	get_acceleration(&x_acceleration, &y_acceleration, &z_acceleration);
    // while(true){

    //     // test whether is triggered
    //     // get acceleration  first

    //     bool trigger = false;
    //     int tilt_front = 0;
    //     int tilt_side = 0;
    //     int tilt_front_count = 0;
    //     int tilt_side_count = 0;
    //     bool flip = false;

    //     trigger = tilt_angle_trigger();

    //     // if there is a suspicion of flip
    //     if (trigger == 1){
    //         // record data for 5 sec
    //         // loop for 1000:

    //         int window_len = 2;
    //         int tilt_count_threshold = window_len / 2; 
            
    //         for (int i=0; i < window_len; i++){

    //             get_acceleration(&x_acceleration, &y_acceleration, &z_acceleration);

    //             tilt_front = abs(atan2(z_acceleration, x_acceleration) * 18000 / M_PI);
    //             tilt_side = abs(atan2(z_acceleration, y_acceleration) * 18000 / M_PI);

    //             if (tilt_front > threshold_tilt_angle){
    //                 tilt_front_count += 1;
    //             }

    //             if (tilt_side > threshold_tilt_angle){
    //                 tilt_side_count += 1;
    //             }
    //         }

    //         // after the entire window, do classification
    //         flip = flip_classifier(tilt_front_count, tilt_side_count, tilt_count_threshold);

    //         if (flip == 1){
    //             // print a statement or make a light bulb switch
    //         }

    //         else{
    //             // print statement: SAFE
    //         }
    //     }    
    // }
}
