#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
 #include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
  std::string incloudfile = "../data/example.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

  pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);

    std::cerr << "Loaded PointClouds: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  time_t tstart, tend; 
  tstart = time(0);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr ground_inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (cloud);
  seg.segment (*ground_inliers, *coefficients);

  if (ground_inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model ground_inliers: " << ground_inliers->indices.size () << std::endl;


  tend = time(0); 
  std::cout << "It took "<< tend - tstart <<" second(s)."<< std::endl; 

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (ground_inliers);
  extract.setNegative (false);
  extract.filter (*cloud_p);
  std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud_p);
  while (!viewer.wasStopped ())
  {

  }
  return (0);
  
}
