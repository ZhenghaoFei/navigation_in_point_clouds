#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <cmath>
#include <ctime>

#include <Eigen/Dense>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"

int main (int argc, char *argv[])
{
    // std::string incloudfile = argv[1];
    std::string incloudfile = "../data/example.pcd";
    time_t tstart, tend; 
    tstart = time(0);


    // Load cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;

    pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);
  
    // reader.read (incloudfile, *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

    uint pnumber = cloud->width * cloud->height;
    std::cout << "pnumber " << pnumber << std::endl;

    tend = time(0); 
    std::cout << "It took "<< tend - tstart <<" second(s)."<< std::endl; 


// --------------------- 
// Rotate point clouds
// ---------------------

    Eigen::Matrix4f transformation_matrix;
    transformation_matrix <<    0.8947,    0.3783,   -0.2377,         0,
                               -0.3894,    0.9211,         0,         0,
                                0.2189,    0.0926,    0.9713,         0,
                                     0,         0,         0,    1.0000;

    printf ("transformation_matrix\n");
    std::cout << transformation_matrix << std::endl;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::transformPointCloud (*cloud, *transformed_cloud, transformation_matrix);

    // // Visualization
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // viewer.showCloud (transformed_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

// --------------------- 
// Create 2.5D Grid Map
// ---------------------
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::fromPCLPointCloud2(cloud,*temp_cloud);

    pcl::PointXYZ minPt, maxPt;
    float grid_size_xy = 0.2;
    float grid_size_z = 0.1; 
    int map_x, map_y, map_z;

    tstart = time(0);
    pcl::getMinMax3D (*cloud, minPt, maxPt);
    map_x = ceil((maxPt.x - minPt.x)/grid_size_xy);
    map_y = ceil((maxPt.y - minPt.y)/grid_size_xy);
    map_z = ceil((maxPt.z - minPt.z)/grid_size_z);

    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    std::cout << "map_x " << map_x << std::endl; 
    std::cout << "map_y " << map_y << std::endl; 
    std::cout << "map_z " << map_z << std::endl; 


    tend = time(0); 
    std::cout << "It took "<< tend - tstart <<" second(s)."<< std::endl; 


    Eigen::MatrixXf voxel_grid = Eigen::MatrixXf::Zero(map_x+1, map_y+1);


    float car_height_min = - 0.9;
    float car_height_max = car_height_min + 0.5;
    int x_idx ,y_idx;

    // Main Loop
    long x = 0;
    for (int point_id = 0; point_id < pnumber; ++point_id)
    {
        if (!std::isnan(cloud->points[point_id].x) && !std::isnan(cloud->points[point_id].y) && !std::isnan(cloud->points[point_id].z))
        {
            if ((cloud->points[point_id].z >= car_height_min) && (cloud->points[point_id].z <= car_height_max))
            {
                x_idx = ceil((cloud->points[point_id].x - minPt.x)/grid_size_xy);
                y_idx = ceil((cloud->points[point_id].y - minPt.y)/grid_size_xy);
                voxel_grid(x_idx, y_idx) += 1;
            }
        }

    }

    voxel_grid /= voxel_grid.maxCoeff();
    // std::cout << "voxel_grid "<< voxel_grid<< std::endl; 

    cv::Mat traversal_map;
    cv::eigen2cv(voxel_grid, traversal_map);


    cv::imshow( "traversal_map", traversal_map );
    cv::waitKey(10000);

    // Save filtered output
    return (0);
}