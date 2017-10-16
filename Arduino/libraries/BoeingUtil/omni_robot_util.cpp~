#include "omni_robot_util.h"
//#include <iostream>

/*
*   KU1_2 = Wheel 1
*   KU1_1 = Wheel 2
*   KU2_1 = Wheel 3
*   KU2_2 = Wheel 4
*/

int wheels_KU[] = {KU1_2, KU1_1, KU2_1, KU2_2};

/*
* getAngVelFourWheels: Converts twist values into four wheel velocities in Kangaroo Units.
*   - vx, vy: Velocity in X and Y direction. Units are in meters/sec.
*   - wz: Rotational velocity around the z-axis. Units are in radians/sec.
*   - *u: Array of int with length 4 used to hold the resulting wheel velocities. Values are in Kangaroo Units
*/

void getAngVelFourWheels(double vx, double vy, double wz, int *u)
{
    int i;
    double H[4][3] = {{-LENGTH - WIDTH, 1, -1}, {LENGTH + WIDTH, 1, 1}, {LENGTH + WIDTH, 1, -1}, {-LENGTH - WIDTH, 1, 1}};
    double twist[3] = {wz, vx, vy};
    // matrix calculation
    // Adding .5 before converting to int does rounding process without need for adding another function.
    for (i = 0; i < 4; i++)
    {
        u[i] = (int)(((H[i][0] * twist[0] + H[i][1] * twist[1] + H[i][2] * twist[2]) / RADIUS) * (wheels_KU[i]/CONV_VAL) + .5);
    }
}

/*
* getTwistFourWheels: Converts four Kangaroo Unit values to four wheel velocities into twist values.
*   - u1, u2, u3, u4: Velocities of each wheel. Units are in Kangaroo Units.
*   - *twist: Array of doubles with length 3 used to hold resulting twist values. Values are in meters/sec, meters/sec, and radians/sec respectively.
*/

void getTwistFourWheels(int u1, int u2, int u3, int u4, double *twist)
{
    double v1 = u1 * CONV_VAL/wheels_KU[0];
    double v2 = u2 * CONV_VAL/wheels_KU[1];
    double v3 = u3 * CONV_VAL/wheels_KU[2];
    double v4 = u4 * CONV_VAL/wheels_KU[3];

    double Vx = (v1 + v2) * RADIUS / 2;
    double Vy = RADIUS / 2 * (v2 + v4 - 2 * (Vx / RADIUS));
    double Wz = (v1 - (Vx / RADIUS) + (Vy / RADIUS)) * RADIUS / (-LENGTH - WIDTH);
    twist[0] = Vx;
    twist[1] = Vy;
    twist[2] = Wz;
}

/*
Some error exists due to floating number error. Values when converting back and forth are accurate to each other.

int main()
{
    int testVel[4];
    getAngVelFourWheels(.5, .5, 1, testVel);
    std::cout << testVel[0] << ", " << testVel[1] << ", " << testVel[2] << ", " << testVel[3] << std::endl;
    double testTwist[3];
    getTwistFourWheels(testVel[0], testVel[1], testVel[2], testVel[3], testTwist);
    std::cout << testTwist[0] << ", " << testTwist[1] << ", " << testTwist[2] << std::endl;
}
*/
