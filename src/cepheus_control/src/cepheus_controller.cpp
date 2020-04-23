#include <eigen3/Eigen/Dense>
#include <sstream>
#include <cmath>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "define.h"


#define DESIRED_VEL 40
#define NUM_OF_MEASUREMENTS 1000

bool reachedVel = false;

void velocityCheckCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    // ROS_INFO("I heard: [%f]", msg->velocity[2]);
    ROS_INFO("reachedVel: %s | RW_vel: %.2f", reachedVel ? "true" : "false", msg->velocity[2]);

    if (msg->velocity[2] >= DESIRED_VEL) {
        reachedVel = true;
    }
}

int main(int argc, char **argv) {

    // calculations
    double EulerConstant = std::exp(1.0);

    double M   = M0+M1+M2;
    double I0  = ((double)1/2)*(M0*pow(RH, (double)2));
    double I1  = 3.46*pow(EulerConstant, (double)-4);
    double I2  = 3.46*pow(EulerConstant, (double)-4);
    double R0X = 0.1954*cos(27.9*PI/180);
    double R0Y = 0.1954*sin(27.9*PI/180);

    std::cout << "M0 = " << M0 << std::endl\
         << "M1 = " << M1 << std::endl\
         << "M2 = " << M2 << std::endl\
         << "M = " << M << std::endl\
         << "RH = " << RH << std::endl\
         << "I0 = " << I0 << std::endl\
         << "I1 = " << I1 << std::endl\
         << "I2 = " << I2 << std::endl\
         << "R0X = " << R0X << std::endl\
         << "R0Y = " << R0Y << std::endl\
         << "L1 = " << L1 << std::endl\
         << "R1 = " << R1 << std::endl\
         << "L2 = " << L2 << std::endl;
    // Eigen Matrix
    Eigen::Matrix<float, NUM_OF_MEASUREMENTS, 8> Y;


    // ros init
    ros::init(argc, argv, "cepheus_controller_node");
    ros::NodeHandle n;

    // publishers
    ros::Publisher RW_velocity_pub = n.advertise<std_msgs::Float64>("/cepheus/reaction_wheel_velocity_controller/command", 1000);
    ros::Publisher LE_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_elbow_position_controller/command", 1000);
    ros::Publisher LS_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_shoulder_position_controller/command", 1000);

    // messages to publish
    std_msgs::Float64 msg_RW;
    std_msgs::Float64 msg_LE;
    std_msgs::Float64 msg_LS;
    // init messages 
    msg_RW.data = 0.1;
    // msg_LE.data = 0.1;
    // msg_LS.data = 0.1;


    // subscribers
    ros::Subscriber RW_velocity_sub = n.subscribe("/cepheus/joint_states", 1000, velocityCheckCallback);

    
    ros::Rate loop_rate(1000);

    int currentMeasurement = 0;

    while (ros::ok()) {

        // ROS_INFO("%f", msg_RW.data);
        RW_velocity_pub.publish(msg_RW);
        // LE_position_pub.publish(msg_LE);
        // LS_position_pub.publish(msg_LS);

        // ROS_INFO("reachedVel: %s", reachedVel ? "true" : "false");
        
        ros::spinOnce();

        if (reachedVel && (currentMeasurement < NUM_OF_MEASUREMENTS)) {
            
            RW_velocity_sub.shutdown();
            // ROS_INFO("reachedVel: %s", reachedVel ? "true" : "false");
            currentMeasurement++;
        }
        else if (currentMeasurement >= NUM_OF_MEASUREMENTS) {

        }
        else {
            msg_RW.data += 0.001;
            msg_LE.data += 0.001;
            msg_LS.data += 0.001;
        }

        loop_rate.sleep();
    }


    return 0;
}
