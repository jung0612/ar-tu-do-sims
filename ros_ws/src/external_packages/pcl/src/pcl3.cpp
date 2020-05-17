
#include <ros/ros.h>
#include <pcl/point_cloud.h>
//#include <pcl/registration/icp.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/pcl_downsampled", 1, &cloudHandler::cloudCB, this);
//        pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched", 1);
        pcl_pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcl_bfalign", 1);

        ROS_INFO("dd");
        pcl::io::loadPCDFile<pcl::PointXYZ> ("3dmarker5.pcd", cloud);
//        pcl::io::loadPCDFile<pcl::PointXYZ> ("3dmarker5.pcd", cloud2);
//        for (size_t i = 0; i < cloud.points.size(); ++i)
//            {
////                cloud.points[i].x  = 0.001*cloud.points[i].x + 0.3;
////                cloud.points[i].y  = 0.001*cloud.points[i].y - 0;
////                cloud.points[i].z  = 0.001*cloud.points[i].z + 0.7;
//                cloud.points[i].x  = 0.001*cloud2.points[i].y ;
//                cloud.points[i].y  = 0.001*cloud2.points[i].z ;
//                cloud.points[i].z  = 0.001*cloud2.points[i].x ;
//            }

        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
                cloud.points[i].x  = 0.001*cloud.points[i].x+0;
                cloud.points[i].y  = 0.001*cloud.points[i].y- 0;
                cloud.points[i].z  = 0.001*cloud.points[i].z+ 0.6;
//            cloud.points[i].x  = 0.001*cloud2.points[i].y ;
//            cloud.points[i].y  = 0.001*cloud2.points[i].z ;
//            cloud.points[i].z  = 0.001*cloud2.points[i].x ;
        }
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_in;
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        pcl::PointCloud<pcl::PointXYZ> cloud_aligned;
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 output2;

        pcl::fromROSMsg(input, cloud_in);

        cloud_out = cloud_in;
//        ROS_INFO("%i", cloud_in.points.size());

        pcl::toROSMsg(cloud, output2);



//        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//        icp.setInputSource(cloud.makeShared());
//        icp.setInputTarget(cloud_out.makeShared());
////        icp.setInputSource(cloud_out.makeShared());
////        icp.setInputTarget(cloud.makeShared());
//        icp.setMaxCorrespondenceDistance(0.1);
//        icp.setMaximumIterations(100);
//        icp.setTransformationEpsilon (1e-12);
//        icp.setEuclideanFitnessEpsilon(0.1);
//
//        icp.align(cloud_aligned);
//        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
//        pcl::toROSMsg(cloud_aligned, output);
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;



        output.header = input.header;
        output2.header = input.header;
//        output.header.frame_id = "/body";
//        output2.header.frame_id = "/body";
//        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
        pcl_pub.publish(output);
        pcl_pub2.publish(output2);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub2;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl");

    cloudHandler handler;

    ros::spin();

    return 0;
}