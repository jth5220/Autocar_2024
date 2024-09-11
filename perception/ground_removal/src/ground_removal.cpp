#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub_coef;
ros::Publisher pub_pc2;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{   
    // Get start time
    ros::Time start_time = ros::Time::now();

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    // ROI 자르기 (PassThrough 활용)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-6.0, 6.0);
    pass.filter (cloud);

    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.27, 3.); // 0.2
    pass.filter(cloud);
    
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-4., 4.);
    pass.filter(cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Setting parameters
    seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
    seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
    seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
    seg.setMaxIterations (00);               //최대 실행 수
    seg.setDistanceThreshold (0.03); 
    seg.setInputCloud (cloud.makeShared ());
    seg.segment (inliers, coefficients);
    
    // Publish the model coefficients
    // pcl_msgs::ModelCoefficients ros_coefficients;
    // pcl_conversions::fromPCL(coefficients, ros_coefficients);
    // pub_coef.publish (ros_coefficients);

    // Extract the inliers (ground points)
    pcl::PointCloud<pcl::PointXYZ> cloud_new;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
    extract.setNegative(true);  // true means remove the inliers
    extract.filter(cloud_new);

    // Convert the pcl/PointCloud data to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_new, output);
    // Publish the data
    pub_pc2.publish(output);

    // Get end time
    ros::Time end_time = ros::Time::now();

    // Calculate duration
    ros::Duration duration = end_time - start_time;
    ROS_INFO("Calculation took %f seconds.", duration.toSec());
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ground_removal");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub_pc2 = nh.advertise<sensor_msgs::PointCloud2> ("velodyne_points_ground_removed", 10);
    // pub_coef = nh.advertise<pcl_msgs::ModelCoefficients> ("ground_removal_coefficinces", 10);

    // Spin
    ros::spin ();
}