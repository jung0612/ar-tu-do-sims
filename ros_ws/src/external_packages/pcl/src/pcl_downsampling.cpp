//
// Created by hms on 4/19/20.
//

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_downsampled", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_crop;
        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
        sensor_msgs::PointCloud2 output;


        pcl::fromROSMsg(input, cloud);

        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            if (cloud.points[i].y>-0.07 && cloud.points[i].y<0.07 && !(cloud.points[i].z==1) && cloud.points[i].x>-0.2 && cloud.points[i].x<0.2)
            {
                cloud_crop.push_back(cloud.points[i]);
            }
        }
//        output.header = input.header;

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud_crop.makeShared());
        voxelSampler.setLeafSize(0.004f, 0.004f, 0.004f);
        voxelSampler.filter(cloud_downsampled);

        pcl::toROSMsg(cloud_downsampled, output);
        output.header = input.header;
//        output.header.frame_id = "/body";
        pcl_pub.publish(output);

    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_downsampling");

    cloudHandler handler;

    ros::spin();

    return 0;
}