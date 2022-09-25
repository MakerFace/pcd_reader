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
#include <string.h>
#include <iostream>

void fromROSMsg_DIY(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    // Get the field structure of this point cloud
    int pointBytes = cloud_msg->point_step;
    int offset_x;
    int offset_y;
    int offset_z;
    int offset_int;
    for (int f = 0; f < cloud_msg->fields.size(); ++f)
    {

        if (cloud_msg->fields[f].name == "x")
            offset_x = cloud_msg->fields[f].offset;
        if (cloud_msg->fields[f].name == "y")
            offset_y = cloud_msg->fields[f].offset;
        if (cloud_msg->fields[f].name == "z")
            offset_z = cloud_msg->fields[f].offset;
        if (cloud_msg->fields[f].name == "intensity")
            offset_int = cloud_msg->fields[f].offset;
    }

    // populate point cloud object
    for (int p = 0; p < cloud_msg->width * cloud_msg->height; ++p)
    {

        pcl::PointXYZI newPoint;
        newPoint.x = *(float *)(&cloud_msg->data[0] + (pointBytes * p) + offset_x);
        newPoint.y = *(float *)(&cloud_msg->data[0] + (pointBytes * p) + offset_y);
        newPoint.z = *(float *)(&cloud_msg->data[0] + (pointBytes * p) + offset_z);
        newPoint.intensity = *(unsigned char *)(&cloud_msg->data[0] + (pointBytes * p) + offset_int);

        cloud->points.push_back(newPoint);
    }
    cloud->height = cloud_msg->height;
    cloud->width = cloud_msg->width;
}

pcl::PCDWriter writer;
void callback(sensor_msgs::PointCloud2::Ptr msg)
{
    // pcl::PCLPointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl_conversions::toPCL(*msg, cloud);
    for (auto &f : msg->fields)
    {
        // std::cout << f.name << std::endl;
        ROS_INFO("field %s", f.name.c_str());
    }

    fromROSMsg_DIY(msg, xyzi_cloud);
    // exit(1);

    writer.writeASCII("/home/liubo/personal/ROSbag/new/" + std::to_string(msg->header.stamp.toSec()) + ".pcd", *xyzi_cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointfilter");
    ros::NodeHandle nh;
    auto ros_bag_callback =
        nh.subscribe("/front_hs/hslidar_points", 100, callback);

    ros::spin();
    // system("pause");
    return (0);
}