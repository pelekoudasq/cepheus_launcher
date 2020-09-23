#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "define.h"

#include <typeinfo>

#define DESIRED_VEL 40  // RW_qdot_des [rad/s]
#define NUM_OF_MEASUREMENTS 1000

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;

template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
    Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
    0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
    tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
}

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

void positionCheckCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {

    omega0 = msg->twist[3].angular.z;
    // ROS_INFO("angular twist z: %.5f", omega0);
}


int main(int argc, char **argv) {

    // calculations
    double EulerConstant = std::exp(1.0);

    double M   = M0+M1+M2;
    double I0  = ((double)1/2)*(M0*pow(RH, (double)2));
    double I1  = 3.46*pow(EulerConstant, (double)-4);
    double I2  = 3.46*pow(EulerConstant, (double)-4);
    // double R0X = 0.1954*cos(27.9*PI/180);
    // double R0Y = 0.1954*sin(27.9*PI/180);

    double frequency = (float)1/DT;

    double hrw = Irw * DESIRED_VEL;

    /* define the robot 8 inertial parameters */
    double pi1 = M0*R0X*((M1+M2)*L1+M2*R1)/M;
    double pi2 = M0*R0X*M2*L2/M;
    double pi3 = M0*R0Y*((M1+M2)*L1+M2*R1)/M;
    double pi4 = M0*R0Y*M2*L2/M;
    double pi5 = I0+M0*(M1+M2)*(pow(R0X, (double)2) + pow(R0Y, (double)2))/M;
    double pi6 = M2*L2*(M0*L1+(M0+M1)*R1)/M;
    double pi7 = I1+(M0*(M1+M2)*pow(L1, (double)2)+2*M0*M2*L1*R1+M2*(M0+M1)*pow(R1, (double)2))/M;
    double pi8 = I2+(M2*(M0+M1)*pow(L2, (double)2))/M;

    Eigen::Matrix<double, 8, 1> robotInertialParameters;
    robotInertialParameters(0, 0) = pi1;
    robotInertialParameters(1, 0) = pi2;
    robotInertialParameters(2, 0) = pi3;
    robotInertialParameters(3, 0) = pi4;
    robotInertialParameters(4, 0) = pi5;
    robotInertialParameters(5, 0) = pi6;
    robotInertialParameters(6, 0) = pi7;
    robotInertialParameters(7, 0) = pi8;


    q1 = q2 = q1dot = q2dot = omega0 = 0.0;
    
    /* Eigen Matrix */
    Matrix Y;
    Y.resize(NUM_OF_MEASUREMENTS, 8);

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
    ros::Subscriber position_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, positionCheckCallback);

    
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
            if (rank == 8) {
                // std::cout << Y << std::endl;
                auto pinv = pseudoInverse(Y);
                // auto pinv = Y.completeOrthogonalDecomposition().pseudoInverse();
                // std::cout << Y.rows() << " " << Y.cols() << " " << pinv.rows() << " " << pinv.cols() <<'\n';

                auto pi_est = pinv*Hcm;
                std::ofstream file;
                file.open("/home/pelekoudas/cepheus_simulator/ls_pi_est.txt");
                if (file.is_open()) {
                    double e = 0.0;
                    for (int i = 0; i < 8; ++i) {
                        e = (robotInertialParameters(i, 0) - pi_est(i, 0))/robotInertialParameters(i, 0)*100;
                        file << "Robot Parameter " << i << ": " << robotInertialParameters(i, 0) << ", LS estimation: " << pi_est(i, 0) << ", error(%): "<< e << '\n';     
                    }
                    file.close();
                } else {
                    std::cout << pi_est << std::endl;
                } 
                break;
            }

            // break;
        }
        else {
            msg_RW.data += 0.1;
        }

        loop_rate.sleep();
    }


    return 0;
}
