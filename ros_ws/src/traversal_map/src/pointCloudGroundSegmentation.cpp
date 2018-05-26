#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Dense>
#include <cmath>
#include <ctime>


#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>
#include "opencv2/highgui/highgui.hpp"


pcl::PointCloud<pcl::PointXYZ> point_cloud_map;

ros::Publisher pub_ground, pub_obstacle;


void generate_road_image(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointIndices::Ptr ground_inliers,  cv::Mat& road_image,  cv::Mat& road_mask)
{



    for (int idx = 0; idx < ground_inliers->indices.size(); idx++)
    {
      road_image.at<cv::Vec4b>(ground_inliers->indices[idx])= cv::Vec4b(0, 255, 0, 1);
      // road_mask.at<double>(ground_inliers->indices[idx])= 1;

    }


  road_image = road_image.reshape(4, cloud->height);
  road_mask = road_mask.reshape(1, cloud->height);


}





void extract_ground (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg_ptr, cv::Mat& road_image,  cv::Mat& road_mask)
{



  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr ground_inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.2);

  seg.setInputCloud (cloud);
  seg.segment (*ground_inliers, *coefficients);

  if (ground_inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model ground_inliers: " << ground_inliers->indices.size () << std::endl;



  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZ>);

  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (ground_inliers);
  extract.setNegative (false);
  extract.filter (*cloud_ground);

  // pcl::ExtractIndices<pcl::PointXYZ> extract_obstacle;

  // extract_obstacle.setInputCloud (cloud);
  // extract_obstacle.setIndices (ground_inliers);
  // extract_obstacle.setNegative (true);
  // extract_obstacle.filter (*cloud_obstacle);

  // sensor_msgs::PointCloud2 cloud_msg_ground;
  // sensor_msgs::PointCloud2 cloud_msg_obstacle;

  // pcl::toROSMsg(*cloud_ground, cloud_msg_ground);
  // pcl::toROSMsg(*cloud_obstacle, cloud_msg_obstacle);

  // cloud_msg_ground.header = point_cloud_msg_ptr->header;
  // cloud_msg_obstacle.header = point_cloud_msg_ptr->header;

  // std::cout << "cloud_ground: " << cloud_msg.header << std::endl;

  // Publish the data.
  // pub_ground.publish (cloud_msg_ground);
  // pub_obstacle.publish(cloud_msg_obstacle);

  generate_road_image(cloud, ground_inliers, road_image, road_mask);

}

void blend_road_to_image(cv::Mat& road_image, cv::Mat& image,  cv::Mat& road_mask)
{
    // road_image.convertTo(road_image, CV_32FC3);
    // image.convertTo(image, CV_32FC3);
    // road_mask.convertTo(road_mask, CV_32FC3, 1.0/255);



    cv::Mat image_alpha(image.size(), CV_8UC4);
    cv::cvtColor(image, image_alpha, CV_RGB2RGBA, 4);

    cv::Mat ouImage = cv::Mat::zeros(image.size(), image.type());

    std::cout << "road_mask " << road_mask.size() << " road_image " << road_image.size() << " image " << image.size() << std::endl;
    // cv::multiply(road_mask, road_image, road_image); 
    cv::add(road_image, image_alpha, ouImage); 
    cv::imshow("blend", ouImage);


}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg_ptr, const sensor_msgs::ImageConstPtr& image_msg_ptr)
{

  // Container for pcl::PCLPointCloud2 data
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*point_cloud_msg_ptr, *cloud);

  cv::Mat road_image(1, cloud->height*cloud->width, CV_8UC4, cv::Vec4b(0, 0, 0, 0));
  cv::Mat road_mask(1, cloud->height*cloud->width, CV_32FC1, 0.);

  extract_ground(cloud, point_cloud_msg_ptr, road_image, road_mask);

  cv::Mat image = cv_bridge::toCvShare(image_msg_ptr, "bgr8")->image;

  // cv::imshow("road_image", road_image);
  // cv::imshow("image", image);

  blend_road_to_image(road_image, image, road_mask);

  cv::waitKey(10);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "point_cloud_subscriber");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/zed/point_cloud/cloud_registered", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/zed/left/image_rect_color", 1);

  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(points_sub, image_sub, 100);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS publisher for the output point cloud
  pub_ground = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_ground", 1);
  pub_obstacle = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_obstacle", 1);

  // Spin
  ros::spin ();
}