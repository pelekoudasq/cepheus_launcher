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
#include "gazebo_msgs/ModelStates.h"
#include "define.h"

#define DESIRED_VEL 20  // RW_qdot_des [rad/s]
#define NUM_OF_MEASUREMENTS 1000

typedef Eigen::Matrix<float, NUM_OF_MEASUREMENTS, 8> Matrix;

bool reachedVel = false;

float q1;       // angle of first joint [rad]
float q2;       // angle of second joint [rad]
float q1dot;    // rate of first joint [rad/s]
float q2dot;    // rate of second joint [rad/s]
float omega0;   // base angular velocity [rad/s]
float RW_vel;   // reaction wheel velocity [rad/s]

void velocityCheckCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    q1      = msg->position[0];
    q2      = msg->position[1];
    q1dot   = msg->velocity[0];
    q2dot   = msg->velocity[1];
    RW_vel  = msg->velocity[4];
    
    // ROS_INFO("RW_vel: %.5f | q1: %.5f | q2: %.5f | q1dot: %.5f | q2dot: %.5f", RW_vel, q1, q2, q1dot, q2dot);

    if (!reachedVel && RW_vel >= DESIRED_VEL)
        reachedVel = true;
    else if (reachedVel) {
        // ROS_INFO("Measurement number: %d");
    }
}

void positionCheckCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {

    omega0 = msg->twist[1].angular.y;
    // ROS_INFO("angular twist y: %.5f", omega0);
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

    double hrw = Irw * DESIRED_VEL;

    /* print defined values */

    // std::cout << "M0 = " << M0 << std::endl\
    //      << "M1 = " << M1 << std::endl\
    //      << "M2 = " << M2 << std::endl\
    //      << "M = " << M << std::endl\
    //      << "RH = " << RH << std::endl\
    //      << "I0 = " << I0 << std::endl\
    //      << "I1 = " << I1 << std::endl\
    //      << "I2 = " << I2 << std::endl\
    //      << "R0X = " << R0X << std::endl\
    //      << "R0Y = " << R0Y << std::endl\
    //      << "L1 = " << L1 << std::endl\
    //      << "R1 = " << R1 << std::endl\
    //      << "L2 = " << L2 << std::endl\
    //      << "frequency = " << frequency << std::endl;


    q1 = q2 = q1dot = q2dot = omega0 = 0.0;
    
    /* Eigen Matrix */
    Eigen::Matrix<float, NUM_OF_MEASUREMENTS, 8> Y;

    /* Define Hrw matrix as a Nx1 column vector and all components equal to hrw */
    Eigen::Matrix<float, NUM_OF_MEASUREMENTS, 1> Hcm;
    for (int i = 0; i < NUM_OF_MEASUREMENTS; ++i)
        Hcm(i, 0) = hrw;

    /* ros init */
    ros::init(argc, argv, "cepheus_controller_node");
    ros::NodeHandle n;

    /* Create publishers */
    ros::Publisher RW_velocity_pub = n.advertise<std_msgs::Float64>("/cepheus/reaction_wheel_velocity_controller/command", 1);
    ros::Publisher LE_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_elbow_position_controller/command", 1);
    ros::Publisher LS_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_shoulder_position_controller/command", 1);

    /* messages to publish */
    std_msgs::Float64 msg_RW;
    std_msgs::Float64 msg_LE;
    std_msgs::Float64 msg_LS;
    
    /* init messages */ 
    msg_RW.data = 0.1;
    msg_LE.data = 0.1;
    msg_LS.data = 0.1;

    int currentMeasurement = 0;

    /* Create subscribers */
    ros::Subscriber RW_velocity_sub = n.subscribe<sensor_msgs::JointState>("/cepheus/joint_states", 1, velocityCheckCallback);
    ros::Subscriber position_sub = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, positionCheckCallback);

    
    ros::Rate loop_rate(frequency);


    while (ros::ok()) {

        RW_velocity_pub.publish(msg_RW);
        LE_position_pub.publish(msg_LE);
        LS_position_pub.publish(msg_LS);
        
        ros::spinOnce();

        // arm joins sinusoidal movement
        msg_LE.data = 3 * sin(ros::Time::now().toSec());
        msg_LS.data = -1 * sin(ros::Time::now().toSec());


        if (reachedVel && (currentMeasurement < NUM_OF_MEASUREMENTS)) {
            
            /* Keep RW desired velocity while taking measurements */

            msg_RW.data = DESIRED_VEL;
            
            ROS_INFO("-----------------------------------------------------------------");
            ROS_INFO("current measurement number: %d", currentMeasurement+1);
            ROS_INFO("q1: %.3f, q2: %.3f, q1dot: %.3f, q2dot: %.3f, omega0: %.3f", q1, q2, q1dot, q2dot, omega0);

            Y(currentMeasurement, 0) = (2*omega0 + q1dot) * cos(q1);
            Y(currentMeasurement, 1) = (2*omega0 + q1dot + q2dot) * cos(q1+q2);
            Y(currentMeasurement, 2) = (2*omega0 + q1dot) * sin(q1);
            Y(currentMeasurement, 3) = (2*omega0 + q1dot + q2dot) * sin(q1+q2);
            Y(currentMeasurement, 4) = omega0;
            Y(currentMeasurement, 5) = (2*omega0 + 2*q1dot + q2dot) * cos(q2);
            Y(currentMeasurement, 6) = omega0 + q1dot;
            Y(currentMeasurement, 7) = omega0 + q1dot + q2dot;

            currentMeasurement++;
        }
        else if (currentMeasurement >= NUM_OF_MEASUREMENTS) {

            /* Measurements completed */

            RW_velocity_sub.shutdown();
            position_sub.shutdown();
            // std::cout << Y << std::endl;
            
            Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qr_decomp(Y);
            // Eigen::FullPivLU<Matrix> lu_decomp(Y);
            // auto fs = Y.colPivHouseholderQr();
            auto rank = qr_decomp.rank();
            ROS_INFO("rank = %ld", rank);

            // break;
        }
        else {
            msg_RW.data += 0.1;
        }

        loop_rate.sleep();
    }


    return 0;
}
