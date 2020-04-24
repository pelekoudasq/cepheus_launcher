#include <eigen3/Eigen/Dense>
#include <sstream>
#include <cmath>
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "define.h"


#define DESIRED_VEL 40
#define NUM_OF_MEASUREMENTS 1000

typedef Eigen::Matrix<float, NUM_OF_MEASUREMENTS, 8> Matrix;

bool reachedVel = false;



void velocityCheckCallback(const sensor_msgs::JointState::ConstPtr& msg, Matrix *Y, int *measurement) {
    
    float q1dot = msg->velocity[0];
    float q2dot = msg->velocity[1];
    ROS_INFO("RW_vel: %.2f | q1dot: %.5f | q2dot: %.5f", msg->velocity[2], q1dot, q2dot);

    if (!reachedVel && msg->velocity[2] >= DESIRED_VEL) {
        reachedVel = true;
    }
    else if (reachedVel) {

        ROS_INFO("Measurement number: %d | q1dot: %.5f, q2dot: %.5f", *measurement, q1dot, q2dot);
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

    double frequency = (float)1/DT;

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
         << "L2 = " << L2 << std::endl\
         << "frequency = " << frequency << std::endl;


    // Eigen Matrix
    Matrix Y;


    // ros init
    ros::init(argc, argv, "cepheus_controller_node");
    ros::NodeHandle n;

    // publishers
    ros::Publisher RW_velocity_pub = n.advertise<std_msgs::Float64>("/cepheus/reaction_wheel_velocity_controller/command", 1);
    ros::Publisher LE_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_elbow_position_controller/command", 1);
    ros::Publisher LS_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_shoulder_position_controller/command", 1);

    // messages to publish
    std_msgs::Float64 msg_RW;
    std_msgs::Float64 msg_LE;
    std_msgs::Float64 msg_LS;
    // init messages 
    msg_RW.data = 0.1;
    // msg_LE.data = 0.1;
    // msg_LS.data = 0.1;

    int currentMeasurement = 0;

    // subscribers
    ros::Subscriber RW_velocity_sub = n.subscribe<sensor_msgs::JointState>("/cepheus/joint_states", 1, boost::bind(&velocityCheckCallback, _1, &Y, &currentMeasurement));

    
    ros::Rate loop_rate(frequency);


    while (ros::ok()) {

        RW_velocity_pub.publish(msg_RW);
        // LE_position_pub.publish(msg_LE);
        // LS_position_pub.publish(msg_LS);
        
        ros::spinOnce();

        if (reachedVel && (currentMeasurement < NUM_OF_MEASUREMENTS)) {
            
            // keep RW desired velocity
            msg_RW.data = DESIRED_VEL;
            
            ROS_INFO("------------------------");
            ROS_INFO("current measurement number: %d", currentMeasurement+1);

            currentMeasurement++;
        }
        else if (currentMeasurement >= NUM_OF_MEASUREMENTS) {

            RW_velocity_sub.shutdown();
        }
        else {
            msg_RW.data += 0.1;
            msg_LE.data += 0.01;
            msg_LS.data += 0.01;
        }

        loop_rate.sleep();
    }


    return 0;
}
