#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <string>

class LaserScanConverter {
  public:
    LaserScanConverter();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Publisher point_cloud_publisher_;
    ros::Subscriber scan_sub_;

    int max_scans;
    std::string tf_frame;
    bool rolling_publish;
    double range_cutoff;
};

LaserScanConverter::LaserScanConverter(): node_("~"), tfListener(tfBuffer) {
    if (!node_.getParam("max_scans", max_scans)) {
        ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read max_scans parameter. Setting to 1.");
        max_scans = 1;
    }

    if (!node_.getParam("fixed_frame", tf_frame)) {
        ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read fixed_frame parameter.");
    }

    if (!node_.getParam("rolling_publish", rolling_publish)) {
        ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read rolling_publish parameter.");
    }

    if (!node_.getParam("range_cutoff", range_cutoff)) {
        ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot read range_cutoff parameter.");
    }

    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> (
                    "/scan", 20, &LaserScanConverter::scanCallback, this);
    point_cloud_publisher_ =
        node_.advertise<sensor_msgs::PointCloud2> ("/scan_cloud", 20, false);
}

