#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <eigen3/Eigen/Dense>
#include <sstream>


int main(int argc, char **argv) {

    ros::init(argc, argv, "cepheus_controller_node");
    ros::NodeHandle n;

    ros::Publisher RW_velocity_pub = n.advertise<std_msgs::Float64>("/cepheus/reaction_wheel_velocity_controller/command", 1000);
    ros::Publisher LE_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_elbow_position_controller/command", 1000);
    ros::Publisher LS_position_pub = n.advertise<std_msgs::Float64>("/cepheus/left_shoulder_position_controller/command", 1000);

    ros::Rate loop_rate(1000);

    int count = 0;

    std_msgs::Float64 msg_RW;
    std_msgs::Float64 msg_LE;
    std_msgs::Float64 msg_LS;
    msg_RW.data = 0.1;
    // msg_LE.data = 0.1;
    // msg_LS.data = 0.1;

    while (ros::ok()) {



        // ROS_INFO("%f", msg_RW.data);
        RW_velocity_pub.publish(msg_RW);
        // LE_position_pub.publish(msg_LE);
        // LS_position_pub.publish(msg_LS);


        ros::spinOnce();

        msg_RW.data += 0.001;
        msg_LE.data += 0.001;
        msg_LS.data += 0.001;
        loop_rate.sleep();
        ++count;
    }


    return 0;
}