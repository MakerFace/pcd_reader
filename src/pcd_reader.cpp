#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointfilter");
  if (argc <= 1) {
    ROS_ERROR("usage: roslaunch pcd_reader pcd_reader path:=/path/to/file");
    exit(1);
  }
  ros::NodeHandle nh;
  ros::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>("point_raw", 1);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCDReader reader;
  if (not reader.read(argv[1], *cloud)) ROS_INFO("reading %s", argv[1]);

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = "map";

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ROS_INFO("publish PointCloud2 message, frame_id is map");
    output.header.stamp = ros::Time::now();
    pcl_pub.publish(output);
    ros::spinOnce();
    ROS_INFO("CURRENT SEQ = %d", output.header.seq);
    loop_rate.sleep();
    output.header.seq++;
  }

  // system("pause");
  return (0);
}