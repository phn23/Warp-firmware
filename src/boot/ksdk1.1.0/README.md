# Summary

A Swiss rehabilitation centre is designing some tasks to assess the recovery progress of their patients. One of the tasks is to measure the acceleration control of the arm in one particular direction. For the task to be marked as successful (good recovery progress), the movement of the patient's arm should be in a straight line and satisfies these inequalities:


1.   X_count > count_threshold, Y_count < count_threshold, Z_count < count_threshold
2.   X_count + Y_count + Z_count = total_count < total_count _threshold


There should be an indicator for successful or unsuccessful tasks and a display to show the uncertainty of the test.

An additional request is that the device should start once it senses the movement to accommodate for different response speeds of the patients.



# Instructions


1.   Once the system is rebooted, OLED lights up green for 3s to indicate it is working.
2. OLED turns off to indicate test reading stage. 
3.   2 test acceleration values (each with a gap of 0.5s) will be read to ensure the accelerometer is reading and JLink Client is displaying properly.
4.   OLED lights up green  to indicate new stage.
5. OLED turns off to indicate start of the main loop (See Figure 1 in the report or the flowchart below).
6. The accelerometer has to move in order to trigger the 5s period.
7. After 5s, the loop returns the classification results.
8. For successful results, the OLED blinks for 10 times in GREEN; for unsuccessful results, the OLED blinks for 10 times in RED.

# Files changed

### 1. Warp-firmware

##### `Makefile`


* cp src/boot/ksdk1.1.0/devSSD1331.*		$~~~~~~~~~~~$ 	build/ksdk1.1/work/demos/Warp/src/





### 2. Warp-firmware/src/boot/ksdk1.1.0：

##### `CMakeLists-FRDMKL03.txt`
* In ADD_EXECUTABLE:
* Added "{ProjDirPath}/../../src/devSSD1331.c"
* Added  "{ProjDirPath}/../../src/devSSD1331.h"


##### `boot.c`
* Algorithm implemented at print before main loop (after the line below)

 #if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && WARP_BUILD_BOOT_TO_CSVSTREAM)


* Removed printing out initialisation menu to avoid overflow
	

##### `config.h`
* Changed kWarpSizesI2cBufferBytes from 4 to 6 

##### `devSSD1331.c`
* Amended int devSSD1331init(void) for drawing rectangles at the brightest OLED level and changed from PTA12 to PTA6
* Defined void devSSD1331_set_up(void) (Note: this is for setting up blinks )
* Defined void devSSD1331_blink_green(void)
* Defined void devSSD1331_blink_red(void)

##### `devSSD1331.h`
* Declared void devSSD1331_set_up(void) (Note: this is for setting up blinks )
* Declared void devSSD1331_blink_green(void)
* Declared void devSSD1331_blink_red(void)


##### `devMMA8451Q.c`
* Defined void get_acceleration(int16_t* x_acc, int16_t* y_acc, int16_t* z_acc)
* Defined bool normal_loop()

##### `devMMA8451Q.h`
* Declared void get_acceleration(int16_t* x_acc, int16_t* y_acc, int16_t* z_acc)
* Declared bool normal_loop()



##### `gpio_pins.c`
* Enabled 5 pins in #if (WARP_BUILD_ENABLE_FRDMKL03)

##### `gpio_pins.h`


*   Enabled 5 pins in #if (WARP_BUILD_ENABLE_FRDMKL03)




