#include <string>
#include <sstream>

#include <signal.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <ros/package.h>

#include <ros/xmlrpc_manager.h>

using namespace cv;
using namespace ros;

bool to_publish = false;

void publish_image_callback(const ros::TimerEvent&) {
    to_publish = true;
}

sensor_msgs::CameraInfo CreateCameraInfo(cv::Size frame_size,
                                         std::string frame_id) {
    sensor_msgs::CameraInfo ci;
    double fx = double(frame_size.width) / 2;
    double fy = double(frame_size.height) / 2;

    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, double(frame_size.width) / 2,
                 0, fy, double(frame_size.height) / 2,
                 0, 0, 1);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ci.K[i * 3 + j] = K.at<double>(i, j);

    for (int i = 0; i < 5; ++i)
        ci.D.push_back(0);

    cv::Mat P = (cv::Mat_<double>(3, 4) << fx, 0, double(frame_size.width) / 2, 0,
                 0, fy, double(frame_size.height) / 2, 0,
                 0, 0, 1, 0);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            ci.P[i * 4 + j] = P.at<double>(i, j);

    ci.width = frame_size.width;
    ci.height = frame_size.height;
    ci.distortion_model = "plumb_bob";
    ci.header.frame_id = frame_id;
    ci.header.stamp = ros::Time::now();

    return ci;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_fake_rgb");

    if (!ros::master::check()) {
        ROS_FATAL("[FAKE_LASER_RGB] Cannot detect ROS master!");
        return 1;
    }

    ros::NodeHandle node("~");

    int frame_rate;
    if (!node.getParam("/frame_rate", frame_rate)) {
        ROS_WARN(
            "[FAKE_LASER_RGB] Camera frequency ('frame_rate') "
            "cannot be read. Using 30 Hz.");
        frame_rate = 30;
    }

    std::string camera_frame;
    if (!node.getParam("camera_frame", camera_frame)) {
        ROS_WARN("[FAKE_LASER_RGB] Cannot read camera_frame parameter.");
    }

    int camera_width;
    if (!node.getParam("camera_width", camera_width)) {
        ROS_WARN("[FAKE_LASER_RGB] Cannot read camera_width parameter.");
    }

    int camera_height;
    if (!node.getParam("camera_height", camera_height)) {
        ROS_WARN("[FAKE_LASER_RGB] Cannot read camera_height parameter.");
    }

    ROS_INFO("[FAKE_LASER_RGB] Advertising.");
    image_transport::ImageTransport it(node);
    image_transport::CameraPublisher cam_pub =
        it.advertiseCamera("/image_raw", 1);

    ros::Rate loop_rate(30);
    ros::Timer timer =
        node.createTimer(ros::Duration(1.0 / frame_rate), publish_image_callback);

    while (node.ok()) {
        ros::spinOnce();

        cv::Mat frame = cv::Mat::zeros(camera_width, camera_height, CV_8UC3);

        if (to_publish) {
            sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                             frame).toImageMsg();
            image->header.frame_id = camera_frame;
            image->header.stamp = ros::Time::now();

            // Get CameraInfo data
            sensor_msgs::CameraInfo ci = CreateCameraInfo(
                                             frame.size(), image->header.frame_id);
            ci.header = image->header;
            cam_pub.publish(*image, ci);
            to_publish = false;
        }

        loop_rate.sleep();
    }

    ros::param::del("~");
    ros::shutdown();
    return 0;
}