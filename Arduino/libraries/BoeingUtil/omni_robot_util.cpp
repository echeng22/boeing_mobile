#include "omni_robot_util.h"

/*
*   KU1_2 = Wheel 1
*   KU1_1 = Wheel 2
*   KU2_1 = Wheel 3
*   KU2_2 = Wheel 4
*/

const int wheels_KU[] = {KU1_2, KU1_1, KU2_1, KU2_2};

/// @brief Converts twist values into four wheel velocities in Kangaroo Units.
/// Core equations used to calculate this can be found on pg. 519 of Professor Lynch's Robotic Manipulation book (Eq 13.10).
/// @param vx [in] A double representing the linear velocity of the body of the robot in meters/sec in the x direction.
/// @param vy [in] A double representing the linear velocity of the body of the robot in meters/sec in the y direction.
/// @param wz [in] A double representing the angular velocity of the body of the robot in rad/sec around the z axis.
/// @param *u [in/out] An integer array that stores the resulting wheel velocities needed to move the robot at the desired twist.
///@post Values inside *u will be in Kangaroo Units (KU) and represent the desired wheel velocties for the robot.
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

/// @brief Converts four Kangaroo Unit values to body twist values.
/// Used the inverse of the equation found in pg. 519 of Professor Lynch's Robotic Manipulation book (Eq 13.10).
/// @param u1 [in] An integer that represents wheel velocity of wheel 1. Units are in KU.
/// @param u2 [in] An integer that represents wheel velocity of wheel 2. Units are in KU.
/// @param u3 [in] An integer that represents wheel velocity of wheel 3. Units are in KU.
/// @param u4 [in] An integer that represents wheel velocity of wheel 4. Units are in KU.
/// @param *twist [in/out] A double array that stores the resulting twist values with the wheel velocity inputs.
///@post Values inside *twist will represent, in the order of, Vx, Vy, Wz, where Vx and Vy represent linear velocities in the x and y direction (meters/sec), and Wz represent angular velocities around the z axis (rad/sec).
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
