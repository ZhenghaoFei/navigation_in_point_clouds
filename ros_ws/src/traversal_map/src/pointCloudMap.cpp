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


#include <Eigen/Dense>
#include <cmath>
#include <ctime>


#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>
#include "opencv2/highgui/highgui.hpp"


pcl::PointCloud<pcl::PointXYZ> point_cloud_map;

ros::Publisher pub;

void generate_traversal_map(const pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::MatrixXi& collision_map)
{


    float grid_size_xy = 0.2;
    float car_height_min = - 0.8;
    float car_height_max = car_height_min + 1;
    float min_x = 0.;
    float min_y = -20.;

    uint pnumber =  cloud.width * cloud.height;

    int x_idx ,y_idx;

    // Main Loop
    for (int point_id = 0; point_id < pnumber; ++point_id)
    {
        // make sure is a valid point
        if (!std::isnan(cloud.points[point_id].x) && !std::isnan(cloud.points[point_id].y) && !std::isnan(cloud.points[point_id].z))
        {
            // check if is point is in collision range
            if ((cloud.points[point_id].z >= car_height_min) && (cloud.points[point_id].z <= car_height_max))
            {
                x_idx = ceil((cloud.points[point_id].x - min_x)/grid_size_xy);
                y_idx = ceil((cloud.points[point_id].y - min_y)/grid_size_xy);
                
                // make sure this point is in mapping range
                if((x_idx>=0) && ((x_idx<100)) && (y_idx>=0) && ((y_idx<200)))
                {
                  collision_map(x_idx, y_idx) += 1;
                }
            }
        }

    }


}





void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg_ptr, const nav_msgs::OdometryConstPtr& Odometry_msg_ptr)
{

  // Container for pcl::PCLPointCloud2 data
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*point_cloud_msg_ptr, cloud);

  // std::cout << "PointCloud before filtering: " << cloud.width * cloud.height 
  //      << " data points (" << pcl::getFieldsList (cloud) << ")." << std::endl;

  std::cout << "odom_msg: " << Odometry_msg_ptr->header << std::endl;

  std::cout << "point_cloud_msg: " << point_cloud_msg_ptr->header << std::endl;


  point_cloud_map += cloud;

  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(point_cloud_map, cloud_msg);
  cloud_msg.header = point_cloud_msg_ptr->header;
  cloud_msg.header.frame_id = "map";

  std::cout << "point_cloud_map: " << cloud_msg.header << std::endl;

  // Publish the data.
  pub.publish (cloud_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "point_cloud_subscriber");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/zed/point_cloud/cloud_registered", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/zed/odom", 1);

  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(points_sub, odom_sub, 10);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe ("/zed/point_cloud/cloud_registered", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_cloud_map", 1);

  // Spin
  ros::spin ();
}