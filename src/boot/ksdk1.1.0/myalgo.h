bool tilt_angle_trigger();
void get_acceleration(int16_t* x_acc, int16_t* y_acc, int16_t* z_acc);
int normal_loop();

#define threshold_tilt_angle 3000 // 3000 / 100 = 30 degrees
