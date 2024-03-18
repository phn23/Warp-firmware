#include <math.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "warp.h"
#include "myalgo.h"
/****************************************************************
The logic is as follows；
1. Get data
2. Calculate the 
****************************************************************/

#define threshold_tilt_angle 3000 // 3000 / 100 = 30 degrees

bool tilt_angle_trigger(){

    int x_acc;
    int y_acc;
    int z_acc;

    int tilt_front; // x, z
    int tilt_side; // y, z

    tilt_front = abs(atan2(z_acc, x_acc) * 18000 / M_PI);
    tilt_side = abs(atan2(z_acc, y_acc) * 18000 / M_PI);

    
    return (tilt_front > threshold_tilt_angle || tilt_side > threshold_tilt_angle);
}

bool flip_classifier(tilt_front_count, tilt_side_count){
    return (tilt_front_count > tilt_count_threshold || tilt_side_count > tilt_count_threshold);        
}

// int tilt_angle_cal(z_acc, n_acc){
//     int tilt_angle_cal = abs(atan2(z_acc, n_acc) * 18000 / M_PI);
// }





int normal_loop(){
    while(true){

        // test whether is triggered

        // get acceleration  first

        bool trigger = false;
        int tilt_front = 0;
        int tilt_side = 0;
        int tilt_front_count = 0;
        int tilt_side_count = 0;
        bool flip = false;

        trigger = tilt_angle_trigger();

        // if there is a suspicion of flip
        if (trigger == 1){
            // record data for 5 sec
            
            // run the programme
            // tilt_angle_count(); //  get tilt_front_count, tilt_side_count

            // loop for 1000:

            int window_len = 1000;
            
            for (int i=0; i < window_len; i++){

                tilt_front = abs(atan2(z_acc, x_acc) * 18000 / M_PI);
                tilt_side = abs(atan2(z_acc, y_acc) * 18000 / M_PI);

                if (tilt_front > threshold_tilt_angle){
                    tilt_front_count += 1;
                }

                if (tilt_side > threshold_tilt_angle){
                    tilt_side_count += 1;
                }
            }

            // after the entire window, do classification
            flip = flip_classifier(tilt_front_count, tilt_side_count);

            if (flip == 1){
                // print a statement or make a light bulb switch
            }

            else{
                // print statement: SAFE
            }
        }    
    }
}
