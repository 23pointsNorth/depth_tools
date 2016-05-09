#include <cmath>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>

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

class CloudConverter {
  public:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    image_geometry::PinholeCameraModel cameraModel;
    sensor_msgs::CameraInfo info_msg;

    cv::Size cameraSize;
    std::string world_frame;
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::CameraPublisher pub_;
    ros::Subscriber cloud_sub;
    double max_depth;
    std::string camera_frame;
    int camera_width;
    int camera_height;
    int num_threads;
    bool postprocess_x;
    int x_max_gap;
    bool postprocess_y;
    int y_max_gap;

    CloudConverter(): nh("~"), it(nh), tfListener(tfBuffer) {
        pub_ = it.advertiseCamera("/depth/image_raw", 1);

        if (!nh.getParam("world_frame", world_frame)) {
            ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read world_frame parameter.");
        }

        if (!nh.getParam("max_depth", max_depth)) {
            ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read max_depth parameter.");
        }

        if (!nh.getParam("camera_frame", camera_frame)) {
            ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read camera_frame parameter.");
        }

        if (!nh.getParam("camera_width", camera_width)) {
            ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read camera_width parameter.");
        }

        if (!nh.getParam("camera_height", camera_height)) {
            ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read camera_height parameter.");
        }

        if (!nh.getParam("postprocess_x", postprocess_x)) {
            ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read postprocess_x parameter.");
        }
        if (postprocess_x) {
            if (!nh.getParam("x_max_gap", x_max_gap)) {
                ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read x_max_gap parameter. Setting to 1.");
                x_max_gap = 1;
            }
            ROS_INFO_STREAM("x_max_gap: " << x_max_gap);
        }

        if (!nh.getParam("postprocess_y", postprocess_y)) {
            ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read postprocess_y parameter.");
        }
        if (postprocess_y) {
            if (!nh.getParam("y_max_gap", y_max_gap)) {
                ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read y_max_gap parameter. Setting to 1.");
                y_max_gap = 1;
            }
        }
        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
                        "/cloud_in", 1, &CloudConverter::NewCloud2Callback, this);

        cameraSize = cv::Size(camera_width, camera_height);

        info_msg = CreateCameraInfo(cameraSize, camera_frame);
        cameraModel.fromCameraInfo(info_msg);

        num_threads = std::max(std::thread::hardware_concurrency(), 1u);
        ROS_INFO_STREAM("Using " << num_threads << " threads to optimize processing.");
    }
    ~CloudConverter();

    void NewCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& cmsg);

  private:
    void CastPoints(pcl::PointCloud<pcl::PointXYZ>& cloud,
                    int start_idx, int end_idx, cv::Mat& image);
    void InterpolateLine(cv::Mat& image, int start_idx, int end_idx,
                         int step_size, int max_gap);
    void InterpolateRowsRange(cv::Mat& image, int row_min, int row_max,
                              int max_gap);
    void InterpolateColsRange(cv::Mat& image, int col_min, int col_max,
                              int max_gap);
};

CloudConverter::~CloudConverter() {
    ros::param::del("~");
    ros::shutdown();
}

void CloudConverter::NewCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr&
                                       cmsg) {
    ros::Time begin = ros::Time::now();

    ROS_DEBUG_STREAM("Starting the callback!");
    cv::Mat image = cv::Mat::zeros(cameraSize, CV_16UC1);

    geometry_msgs::TransformStamped transform;
    sensor_msgs::PointCloud2 cloud_out;

    ROS_DEBUG_STREAM("RECEIVED new message!");
    try {
        ros::Time acquisition_time = cmsg->header.stamp;
        ROS_DEBUG_STREAM("Asking can tf?");
        // std::string error_msg;
        // if (!tfBuffer.canTransform(
        //      cameraModel.tfFrame(), cameraModel.stamp(),
        //      world_frame, acquisition_time + ros::Duration().fromSec(0.2),
        //      world_frame, ros::Duration(1.0/25)),
        //      &error_msg){
        //  ROS_WARN_STREAM("[POINTCLOUD2_TO_DEPTH] Cannot find tf transforms for this message. "
        //      "Skipping... The tfs are: " << cameraModel.tfFrame() << " and " << world_frame <<
        //      " acquisition_time: " << acquisition_time << " cameraModel.stamp(): " << cameraModel.stamp() <<
        //      " with error: " << error_msg);
        //  return;
        // }

        // or ? tfBuffer.lookupTransform(cameraModel.tfFrame(), world_frame, ros::Time::now())

        // if (!tfBuffer.canTransform(
        //      cameraModel.tfFrame(),
        //      world_frame,
        //      acquisition_time + ros::Duration().fromSec(0.2),
        //      ros::Duration(1.0/25))){
        //  ROS_WARN_STREAM("[POINTCLOUD2_TO_DEPTH] Cannot find tf transforms for this message. "
        //      "Skipping... The tfs are: " << cameraModel.tfFrame() << " and " << world_frame <<
        //      " acquisition_time: " << acquisition_time);
        //  return;
        // }
        ROS_DEBUG_STREAM("CAN do the transform!!!");
        transform = tfBuffer.lookupTransform(cameraModel.tfFrame(),
                                             world_frame, ros::Time(0), //acquisition_time - ros::Duration().fromSec(45),
                                             ros::Duration(1.0 / 25));
        ROS_DEBUG_STREAM("Will perform the transform");

        tf2::doTransform(*cmsg, cloud_out, transform);
        ROS_DEBUG_STREAM("Converted the pointcloud tf.");
    } catch (tf2::TransformException& ex) {
        ROS_WARN("[POINTCLOUD2_TO_DEPTH] TF exception:\n%s", ex.what());
        ROS_WARN_STREAM("At: " << cmsg->header.stamp);
        return;
    }

    ROS_DEBUG_STREAM("Done with transformations!");

    // Convert to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_out, pcl_cloud);
    int cloudsize = pcl_cloud.width * pcl_cloud.height;

    std::thread t[num_threads];

    int batch = std::ceil(float(cloudsize) / num_threads);

    // Start threads
    for (int i = 0; i < num_threads; ++i) {
        t[i] = std::thread(&CloudConverter::CastPoints, this,
                           std::ref(pcl_cloud), batch * i, std::min(batch * (i + 1), cloudsize),
                           std::ref(image));
    }

    // Join threads
    for (int i = 0; i < num_threads; ++i) {
        t[i].join();
    }

    if (postprocess_x) {
        batch =  std::ceil(float(image.rows) / num_threads);

        for (int i = 0; i < num_threads; ++i) {
            t[i] = std::thread(&CloudConverter::InterpolateRowsRange, this,
                               std::ref(image), batch * i, std::min(batch * (i + 1), image.rows), x_max_gap);
        }

        for (int i = 0; i < num_threads; ++i) {
            t[i].join();
        }
    }

    if (postprocess_y) {
        batch =  std::ceil(float(image.cols) / num_threads);

        for (int i = 0; i < num_threads; ++i) {
            t[i] = std::thread(&CloudConverter::InterpolateColsRange, this,
                               std::ref(image), batch * i, std::min(batch * (i + 1), image.cols), y_max_gap);
        }

        for (int i = 0; i < num_threads; ++i) {
            t[i].join();
        }
    }

    ROS_DEBUG_STREAM("Publishing!");
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                         "mono16"
                                                         , image).toImageMsg();

    pub_.publish(*image_msg, info_msg);

    ROS_DEBUG_STREAM("Endign the callback");
    ROS_INFO_STREAM("Callback timer: " << ros::Time::now() - begin << " sec.");
}

void CloudConverter::CastPoints(pcl::PointCloud<pcl::PointXYZ>& cloud,
                                int start_idx, int end_idx, cv::Mat& image) {
    ushort* image_data = (ushort*)(image.data);
    for (int i = start_idx; i < end_idx; ++i) {
        // ROS_DEBUG_STREAM("Laser (x,y,z) = " << pcl_cloud.points[i]);
        cv::Point3d pt_cv(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        cv::Point2d uv = cameraModel.project3dToPixel(pt_cv);
        // ROS_WARN_STREAM("Projected to: " << uv);
        if (!cv::Rect(0, 0, image.cols, image.rows).contains(uv))
            continue;
        ushort depth_value = 0;
        if (pt_cv.z <= max_depth)
            depth_value = ushort(pt_cv.z * 1000); // Convert metres to mm.
        image_data[uint(uv.y) * image.cols + uint(uv.x)] =  depth_value;
    }
}

void CloudConverter::InterpolateRowsRange(cv::Mat& image, int row_min,
                                          int row_max, int max_gap) {
    for (int i = row_min; i < row_max; ++i) {
        InterpolateLine(image, i * image.cols, (i + 1) * image.cols, 1, max_gap);
    }
}

void CloudConverter::InterpolateColsRange(cv::Mat& image, int col_min,
                                          int col_max, int max_gap) {
    for (int i = col_min; i < col_max; ++i) {
        InterpolateLine(image, i, i + image.cols * (image.rows - 1),
                        image.cols, max_gap);
        // ROS_INFO_STREAM("COLS: " << i << " to " << i + image.cols * (image.rows - 1));
    }
}

void CloudConverter::InterpolateLine(cv::Mat& image, int start_idx, int end_idx,
                                     int step_size, int max_gap) {
    // skip the first zero pixels
    int last_non_zero = -1;
    int dist = 0;
    ushort* image_data = (ushort*)(image.data);
    for (int i = start_idx; i < end_idx; i += step_size, ++dist) {
        // ROS_INFO_STREAM("Inner idx: " << i);
        if (image_data[i] == 0)
            continue;
        if (last_non_zero != -1) {
            // current i has some pixel value
            // Look how far away we were
            // int dist = (i - last_non_zero) / step_size;
            if (dist < max_gap && dist > 1) {
                // ROS_INFO_STREAM("Using at dist " << dist << " max_gap " << max_gap << " id: " <<
                //                 last_non_zero << " " << i);
                double gradient = std::abs(
                                      double(image_data[last_non_zero] - image_data[i]) / dist);
                // ROS_INFO_STREAM("gradient: " << gradient);
                // In range - go back and fill in the gap
                int k = 0;
                for (int j = last_non_zero; j < i; j += step_size, ++k) {
                    image_data[j] = ushort(image_data[last_non_zero] + gradient * k);
                }
            }
        }
        last_non_zero = i;
        dist = 0;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud2_to_depth");

    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //    ros::console::notifyLoggerLevelsChanged();
    // }
    CloudConverter converter;

    ros::spin();

    ros::param::del("~");
    ros::shutdown();
    return 0;
}
