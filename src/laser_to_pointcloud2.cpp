#include <list>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include "laser_to_pointcloud2/laser_to_pointcloud2.hpp"

std::list<sensor_msgs::PointCloud2> all_cloud;

sensor_msgs::PointCloud2 concatenate_clouds(std::list<sensor_msgs::PointCloud2>
                                            clouds) {
	sensor_msgs::PointCloud2 res = clouds.front();

	ROS_DEBUG_STREAM("Merging cloud list with side: " << clouds.size());
	std::list<sensor_msgs::PointCloud2>::iterator it;
	for (it = clouds.begin(), ++it; it != clouds.end(); ++it)
		if (!pcl::concatenatePointCloud(res, *it, res))
			ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot concatenate PointCloud2s!");
	res.header.stamp = ros::Time::now();
	return res;
}

void LaserScanConverter::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan_in) {
	if (!tfBuffer.canTransform(
	            scan_in->header.frame_id,
	            tf_frame,
	            scan_in->header.stamp + ros::Duration().fromSec(
	                scan_in->ranges.size()*scan_in->time_increment),
	            ros::Duration(1.0))) {
		ROS_WARN("[LASER_TO_POINTCLOUD2] Cannot find tf for this message. Skipping...");
		return;
	}

	sensor_msgs::PointCloud2 cloud;
	projector_.transformLaserScanToPointCloud(tf_frame, *scan_in,
	                                          cloud, tfBuffer, range_cutoff);

	ROS_DEBUG_STREAM("Transformed cloud: " << cloud);

	all_cloud.push_back(sensor_msgs::PointCloud2(cloud));
	while (all_cloud.size() > max_scans) {
		all_cloud.pop_front();
		if (!rolling_publish) {
			point_cloud_publisher_.publish(concatenate_clouds(all_cloud));
			all_cloud.clear();
		}
	}

	if (rolling_publish) {
		point_cloud_publisher_.publish(concatenate_clouds(all_cloud));
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "laser_to_pointcloud2");

	LaserScanConverter converter;
	ROS_INFO("[LASER_TO_POINTCLOUD2] Converting LaserScan data to PointCloud2 ...");
	ros::spin();
	ros::param::del("~");
	ros::shutdown();
	return 0;
}
