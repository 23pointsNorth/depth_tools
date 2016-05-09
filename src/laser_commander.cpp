#include <ros/ros.h>
#include <pr2_msgs/LaserTrajCmd.h>
#include <pr2_msgs/SetLaserTrajCmd.h>

const int num_scans = 3;
const double time_to_execute = 20.0; // sec
const int min_idx = -45.6; // deg
const int max_idx =  90.6; // deg

pr2_msgs::LaserTrajCmd CreateMessage() {
    pr2_msgs::LaserTrajCmd laser_traj_cmd_msg;

    laser_traj_cmd_msg.header.frame_id = "/odom_combined";
    laser_traj_cmd_msg.header.stamp = ros::Time::now();

    laser_traj_cmd_msg.profile = "blended_linear";
    laser_traj_cmd_msg.position.resize(3);
    ROS_INFO("Resized");
    laser_traj_cmd_msg.position[0] = 1.05;
    laser_traj_cmd_msg.position[1] = -0.7;
    laser_traj_cmd_msg.position[2] = 1.05;
    // laser_traj_cmd_msg.position.resize(num_scans);
    // for (int i = 0; i < num_scans; ++i) {
    //     laser_traj_cmd_msg.position[i] = ((double(i) / num_scans) *
    //                                       (abs(min_idx) + abs(max_idx)) - min_idx) * 3.14 / 180;
    // }

    ros::Duration dur;
    // laser_traj_cmd_msg.time_from_start.resize(num_scans);
    // // ROS_INFO("Resized time_from_start");
    // for (int i = 0; i < num_scans; ++i) {
    //     laser_traj_cmd_msg.time_from_start[i] =
    //         dur.fromSec((double(i) / num_scans) * time_to_execute);
    // }

    laser_traj_cmd_msg.time_from_start.resize(3);
    ROS_INFO("Resized time_from_start");
    laser_traj_cmd_msg.time_from_start[0] = dur.fromSec(0.0);
    laser_traj_cmd_msg.time_from_start[1] = dur.fromSec(1.8);
    laser_traj_cmd_msg.time_from_start[2] = dur.fromSec(2.3125);

    laser_traj_cmd_msg.max_velocity = 10;
    laser_traj_cmd_msg.max_acceleration = 30;
    return laser_traj_cmd_msg;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_communicator");
    ros::NodeHandle nh("~");

    ros::Publisher tilt_laser_traj_cmd_publisher =
        nh.advertise<pr2_msgs::LaserTrajCmd>("/laser_tilt_controller/set_traj_cmd", 1);

    pr2_msgs::LaserTrajCmd msg = CreateMessage();
    tilt_laser_traj_cmd_publisher.publish(msg);
    ROS_INFO("Published!");

    // while (ros::ok()) {
    //     ros::spinOnce();
    // }
    ros::spin();
    ros::param::del("~");
    ros::shutdown();
    return 0;
}
