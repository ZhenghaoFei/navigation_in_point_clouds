#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>
#include <cmath>
#include <ctime>


#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>
#include "opencv2/highgui/highgui.hpp"

ros::Publisher pub;

void generate_traversa_map(const pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::MatrixXf& collision_map)
{


    float grid_size_xy = 0.2;
    float car_height_min = - 0.9;
    float car_height_max = car_height_min + 1.5;
    float min_x = 0.;
    float min_y = -20.;

    uint pnumber = 720*1080;

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


void threshold_collision_map(Eigen::MatrixXf& collision_map, int threshold)
{
  int size = collision_map.size();
  for (int i = 0; i < size; ++i)
  {
    if(collision_map(i) > threshold)
    {
      collision_map(i) = 1;
    }
    else
    {
      collision_map(i) = 0;
    }
  }
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg_ptr)
{

  // Container for pcl::PCLPointCloud2 data
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*point_cloud_msg_ptr, cloud);

  std::cout << "PointCloud before filtering: " << cloud.width * cloud.height 
       << " data points (" << pcl::getFieldsList (cloud) << ")." << std::endl;


  float grid_size_xy = 0.2;
  int map_x, map_y;

  map_x = 100; // x from 0 m to 20 m
  map_y = 200; // y from -20 m to 20 m

  Eigen::MatrixXf collision_map = Eigen::MatrixXf::Zero(map_x, map_y);


  generate_traversa_map(cloud, collision_map);
  // std::cout << "collision_map "<< collision_map<< std::endl; 

  int threshold = 200;
  threshold_collision_map(collision_map, threshold);
  // collision_map /= collision_map.maxCoeff();
  // std::cout << "collision_map "<< collision_map<< std::endl; 

  cv::Mat traversal_map;
  cv::eigen2cv(collision_map, traversal_map);


  cv::imshow( "traversal_map", traversal_map );
  cv::waitKey(100);


  // convert the traversal map to ros image msg
  sensor_msgs::ImagePtr traversal_map_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", traversal_map).toImageMsg();




  // Do data processing here...
  // output = *input;

  // Publish the data.
  pub.publish (traversal_map_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "point_cloud_subscriber");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/zed/point_cloud/cloud_registered", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::Image> ("traversal_map", 1);

  // Spin
  ros::spin ();
}