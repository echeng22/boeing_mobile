#ifndef OMNI_ROBOT_UTIL
#define OMNI_ROBOT_UTIL



/*
    Defining length, width and height of the robot, and radius of the wheels.
        -Wheel to Wheel Width measurement: 19.71 inches. 
        -Thickness of wheels 3 inches.
        -Wheel to Wheel Length measurement: 16.25 inches. 
        -Radius of Wheels: 4 inches.
        -Not sure if height is needed...commented out for now.
*/

#define LENGTH 0.2064
#define RADIUS 0.1016
#define WIDTH  0.2032
//#define HEIGHT 0.1

// Kangaroo Unit Nominal Conversion Values. Sending these values to each respective wheel results in about 60 RPM +/- .5. May need to adjust when tuning the robot on the ground...
// Conversion to KU units is #KU_CONSTANT/CONV_VAL and vice versa.

#define CONV_VAL 6.2831

#define KU1_1 355
#define KU1_2 330
#define KU2_1 340
#define KU2_2 370
#define KU_avg 349

void getAngVelFourWheels(double vx, double vy, double wz, int *u);
void getTwistFourWheels(int u1, int u2, int u3, int u4, double *twist);

#endif
