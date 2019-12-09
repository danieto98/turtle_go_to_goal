#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include<math.h>

#define PI 3.14159265

using namespace std;

// Global variables for the current position of the robot
float x_real = 0;
float y_real = 0;
float theta = 0;

// Get the euclidean distance between the real pose and a goal pose
float EuclidianDistance(float x_g, float y_g, float x_r, float y_r) {
    return sqrt(pow((x_g - x_r),2) + pow((y_g - y_r),2));
}

// Callback for the pose, set all global variables
void poseCallback(const turtlesim::Pose::ConstPtr& pose) {
    x_real = pose->x;
    y_real = pose->y;
    theta = pose->theta;
    // Rescale the value of theta such that it is in [0, 2PI]
    if(theta < 0) {
        theta = 2*PI + theta;
    }
}

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "turtle_go_to_goal");
    ros::NodeHandle n;

    // Subscribe to the turtle's pose and publish its velocities
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    ros::Subscriber sub = n.subscribe("turtle1/pose", 1, poseCallback);

    // Set variables needed to publish data
    float x_goal = 0;
    float y_goal = 0;
    geometry_msgs::Vector3 lin_vec;
    lin_vec.y = 0;
    lin_vec.z = 0;
    geometry_msgs::Vector3 ang_vec;
    ang_vec.x = 0;
    ang_vec.y = 0;
    geometry_msgs::Twist pub_twist;

    // Set a sleeping rate for the end of each loop
    ros::Rate loop_rate(10);

    while(1) {
        // Get coordinates to reach (exit if both zero)
        cout << "Insert X and Y coordinates that the robot should reach (type 0 0 for exit)" <<endl;
        cin >> x_goal >> y_goal;
        if(x_goal == 0 && y_goal == 0) {
            break;
        }

        //Set initial conditions
        int counter = 0;
        lin_vec.x = 0;
        double ang_val, desired;
        int direction = 1;
        float dist = 0.0;
        bool angle_done = false;
        bool br = false;
        float prev_dist = 0.0;

        // Move turtle towards goal
        while (ros::ok()) {
            // Get distance from current pose to desired one
            dist = EuclidianDistance(x_goal,y_goal,x_real,y_real);
            // Get desired angle (scale in [0, 2PI])
            desired = atan2(y_goal - y_real, x_goal - x_real);
            if(desired < 0) {
                desired = 2*PI + desired;
            }
            // Get the delta of the current and desired angle as an absolute value
            ang_val = theta - desired;
            if(ang_val < 0) {
                ang_val = -ang_val;
            }
            // Find out in which direction to move
            if(desired < theta) {
                if(2*PI - theta + desired < theta - desired) {
                    direction = 1;
                } else {
                    direction = -1;
                }
            } else {
                if(2*PI - desired + theta < desired - theta) {
                    direction = -1;
                } else {
                    direction = 1;
                }
            }
            // If already satisfied with orientation, start translating towards goal
            if(ang_val < 0.03 || angle_done) {
                if(dist < prev_dist || !angle_done) {
                    lin_vec.x = dist;
                } else { // Stop moving if there's an overshoot
                    lin_vec.x = 0;
                    br = true;
                }
                prev_dist = dist;
                ang_vec.z = 0;
                angle_done = true;
            } else { // Keep rotating if orientation is not good yet
                lin_vec.x = 0;
                ang_vec.z = ((float)direction)*ang_val;
            }
            // Publish all commands to the turtle
            pub_twist.linear = lin_vec;
            pub_twist.angular = ang_vec;
            vel_pub.publish(pub_twist);
            if(br) {
                break;
            }

            // Exit conditions (too many iterations or close enough)
            counter++;
            if(counter >= 900) {
              break;
            }
            if(dist < 0.25) {
                ang_vec.z = 0;
                lin_vec.x = 0;
                pub_twist.linear = lin_vec;
                pub_twist.angular = ang_vec;
                vel_pub.publish(pub_twist);
                break;
            }

            // Sleep for the assigned rate
            loop_rate.sleep();
            // Call the callback if needed
            ros::spinOnce();
        }
        // Output the results to the screen
        cout << "The process has finished in " << counter << " iterations" << endl;
        cout << "The position error is " << dist << endl << endl;
    }
}
