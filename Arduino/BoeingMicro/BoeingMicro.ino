// Files needed to run ROS on Arduino
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

//Custom Velocity Conversion Library
#include <omni_robot_util.h>

// Kangaroo Motion Controller Library and Definitions
//   - Independent mode channels on Kangaroo are, by default, '1' and '2'.
//   - Serial1 Pins: 18 - TX (White), 19 - RX (Red)
//   - Serial2 Pins: 16 - TX (White), 17 - RX (Red)
#include <Kangaroo.h>


// Initialize Kangaroo Serial objects
KangarooSerial  K1(Serial1);
KangarooChannel K1_1(K1, '1');
KangarooChannel K1_2(K1, '2');

KangarooSerial  K2(Serial2);
KangarooChannel K2_1(K2, '1');
KangarooChannel K2_2(K2, '2');

// Defining constants ie. frequency values, wheel conversion values, etc.
//#define INT_FREQ 1000000
//const double dt = INT_FREQ *.000001; 
int wheel_vals[4];
double twist_vals[3];

// Variables that will hold prev encoder position of motors 
int k1_1p, k1_2p, k2_1p, k2_2p = 0;
// Variables that will hold current encoder position of motors 
int k1_1c, k1_2c, k2_1c, k2_2c = 0;
// Variables that will keep track of x, y and theta positions
double x_pos, y_pos, theta_pos = 0;
// Variables that will keep track of individual wheel velocities
int u1, u2, u3, u4;
// Variables that will keep track of individual wheel velocities
unsigned long prev_t, curr_t;
// Variables that will keep track of individual wheel velocities
double dt;

////  Initializing ROS publishers/subscribers. Also setting up callback function for ROS subscriber.
ros::NodeHandle nh;
geometry_msgs::Pose odom;

void updateWheelVel(const geometry_msgs::Twist& twist){
  getAngVelFourWheels(twist.linear.x, twist.linear.y, twist.angular.z, wheel_vals);
  
  /* 
   *  K1 wheel velocities are flipped due to how the reference frame used to convert twist to wheel velocities.
   *  This is done to match the setup used in Professor Lynch's Robotic Manipulation book (pg. 519).
   */
  K1_1.s(wheel_vals[1]);
  K1_2.s(wheel_vals[0]);      
  K2_1.s(wheel_vals[2]);
  K2_2.s(wheel_vals[3]);   
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("/vel", updateWheelVel);
ros::Publisher odom_pub("/odom", &odom); 


/*
 * Arduino Code Setup
 */

void setup() {
  // Setup communication to Kangaroo Motion Controller  
  Serial1.begin(115200);
  K1_1.start();
  K1_1.home().wait();
  K1_2.start();
  K1_2.home().wait();
  
  Serial2.begin(115200);
  K2_1.start();
  K2_1.home().wait();
  K2_2.start();
  K2_2.home().wait();

  // Set streaming on Kangaroo to true. This means that there is no verification to see if the command was received or not. This improves command rate at the cost of reduced reliability.
  // Noticed that when streaming is false, odom messages are not reliably being published. May be due to delays in verifying commands.
  K1_1.streaming(true);
  K1_2.streaming(true);
  K2_1.streaming(true);
  K2_2.streaming(true);
  
  // Initialize ROS Nodes
  nh.initNode();
  nh.subscribe(twist_sub);
  nh.advertise(odom_pub);
}


// spinOnce is used to keep ROS on Arduino and on the Linux computer in sync. If not called enough or there is a function that takes a significant amount of time to run, loss of sync error will occur and cause a drop in messages being published
// by the Arduino. To avoid this, spinOnce is added in locations where delays may occur in the code.
void loop() {
  nh.spinOnce();  
  integrateOdom();
  nh.spinOnce();
  pubOdom();
  nh.spinOnce();
  odom_pub.publish(&odom);
  nh.spinOnce();  
}

// Function that updates the previous and current KU values for each wheel. Also calculates the difference in time between function calls.
void integrateOdom(){
  // Update previous values
  k1_1p = k1_1c;
  k1_2p = k1_2c;
  k2_1p = k2_1c;
  k2_2p = k2_2c;
  
  // Make call to kangaroo to get current encoder position values  
  curr_t = millis();
  k1_1c = K1_1.getP().value();
  nh.spinOnce();
  k1_2c = K1_2.getP().value();
  nh.spinOnce();
  k2_1c = K2_1.getP().value();
  nh.spinOnce();
  k2_2c = K2_2.getP().value();
  nh.spinOnce();
  dt = (curr_t - prev_t)*.001;
  prev_t = curr_t;
}

// Function that calculates the odometry of the robot and builds the odom message to be published.
void pubOdom(){    
  //Calculate wheel speeds in KU units
  u1 = (int)(((k1_2c - k1_2p)/dt) + .5);
  u2 = (int)(((k1_1c - k1_1p)/dt) + .5);
  u3 = (int)(((k2_1c - k2_1p)/dt) + .5);
  u4 = (int)(((k2_2c - k2_2p)/dt) + .5);
  
  // Convert wheel speeds to twist and store in array
  getTwistFourWheels(u1, u2, u3, u4, twist_vals);  

  // Integrate twist to get estimated position
  x_pos += twist_vals[0] * dt;
  y_pos += twist_vals[1] * dt;
  theta_pos += twist_vals[2] * dt;

  // Build odom message    
  odom.position.x = x_pos;
  odom.position.y = y_pos;
  odom.orientation = tf::createQuaternionFromYaw(theta_pos);
}

