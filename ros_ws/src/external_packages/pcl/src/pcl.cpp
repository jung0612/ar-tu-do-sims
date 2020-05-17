
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <std_msgs/Float32MultiArray.h>

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

PointCloudT::Ptr object (new PointCloudT);
PointCloudT::Ptr object_aligned (new PointCloudT);
PointCloudT::Ptr scene (new PointCloudT);
FeatureCloudT::Ptr object_features (new FeatureCloudT);
FeatureCloudT::Ptr scene_features (new FeatureCloudT);
float car_est[3]={0};
int param1, param2, param3;
float param4, param5, param6;
class cloudHandler
{
public:

    cloudHandler()
    {
        pcl_sub = nh.subscribe("/pcl_downsampled", 1, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched", 1);
        pcl_pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcl_bfalign", 1);
        state_pub = nh.advertise<std_msgs::Float32MultiArray>("car_est", 1);
        nh.getParam("/pcl/param1", param1);
        nh.getParam("/pcl/param2", param2);
        nh.getParam("/pcl/param3", param3);
        nh.getParam("/pcl/param4", param4);
        nh.getParam("/pcl/param5", param5);
        nh.getParam("/pcl/param6", param6);

        std::string pkg;
        pkg = ros::package::getPath("pcl");
        fpath = pkg.append("/src/3dmarker2.pcd");
        ROS_INFO("dd");
        pcl::console::print_highlight ("Downsampling...\n");
        pcl::io::loadPCDFile<PointNT> (fpath, *object);

        for (size_t i = 0; i < object->points.size(); ++i)
        {

            object->points[i].x  = 0.001*object->points[i].x + 0;
            object->points[i].y  = 0.001*object->points[i].y - 0;
            object->points[i].z  = 0.001*object->points[i].z + 0;
        }
        pcl::toROSMsg(*object, output2);
        FeatureEstimationT fest;
        fest.setRadiusSearch (0.03);
        fest.setInputCloud (object);
        fest.setInputNormals (object);
        fest.compute (*object_features);
    }


    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::fromROSMsg(input, *scene);

//        pcl::console::print_highlight ("Estimating scene normals...\n");
        pcl::NormalEstimationOMP<PointNT,PointNT> nest;
        nest.setRadiusSearch (0.0005); //0.001
        nest.setInputCloud (scene);
        nest.compute (*scene);

//        pcl::console::print_highlight ("Estimating features...\n");
        FeatureEstimationT fest;
        fest.setRadiusSearch (0.03);
//        fest.setInputCloud (object);
//        fest.setInputNormals (object);
//        fest.compute (*object_features);
        fest.setInputCloud (scene);
        fest.setInputNormals (scene);
        fest.compute (*scene_features);

//        pcl::console::print_highlight ("Starting alignment...\n");
        pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
        align.setInputSource (object);
        align.setSourceFeatures (object_features);
        align.setInputTarget (scene);
        align.setTargetFeatures (scene_features);
        align.setMaximumIterations (param1); // Number of RANSAC iterations
        align.setNumberOfSamples (param2); // Number of points to sample for generating/prerejecting a pose 3
        align.setCorrespondenceRandomness (param3); // Number of nearest features to use 20
        align.setSimilarityThreshold (param4); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (2.5f * param5); // Inlier threshold 0.005
        align.setInlierFraction (param6); // Required inlier fraction for accepting a pose hypothesis
        {
            pcl::ScopeTime t("Alignment");
            align.align (*object_aligned);
        }

        if (align.hasConverged ())
        {
            // Print results
            printf ("\n");
            Eigen::Matrix4f transformation = align.getFinalTransformation ();
            pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
            pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
            pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
            pcl::console::print_info ("\n");
            pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
            pcl::console::print_info ("\n");
            pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
            car_est[0] = transformation(0,3);
            car_est[1] = transformation(1,3);
            car_est[2] = transformation(2,3);
        }
        else
        {
            pcl::console::print_error ("Alignment failed!\n");
            
        }
        pcl::toROSMsg(*object_aligned, output);
        output.header = input.header;
        output2.header = input.header;

        std_msgs::Float32MultiArray msg;
        msg.data.push_back(car_est[0]);
        msg.data.push_back(car_est[1]);
        msg.data.push_back(car_est[2]);


        pcl_pub.publish(output);
        pcl_pub2.publish(output2);
        state_pub.publish(msg);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub2;
    ros::Publisher state_pub;

    std::string fpath;
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output2;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl");

    cloudHandler handler;

    ros::spin();

    return 0;
}