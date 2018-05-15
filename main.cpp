#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>


#include <ctime>


int main (int argc, char *argv[])
{
    // std::string incloudfile = argv[1];
    std::string incloudfile = "../data/example.pcd";
    time_t tstart, tend; 
    tstart = time(0);


    // Load cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
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
// Creat 2.5D Grid Map
// ---------------------
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::fromPCLPointCloud2(cloud,*temp_cloud);


    tstart = time(0);
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D (*cloud, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;



    tend = time(0); 
    std::cout << "It took "<< tend - tstart <<" second(s)."<< std::endl; 

    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // viewer.showCloud (cloud);
    // while (!viewer.wasStopped ())
    // {
        
    // }
    // // // Set up KDTree
    // pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
    // tree->setInputCloud (cloud);

    // // Neighbors containers
    // std::vector<int> k_indices;
    // std::vector<float> k_distances;

    // Main Loop

    // long x = 0;
    // for (int point_id = 0; point_id < pnumber; ++point_id)
    // {
    //     x += cloud->points[point_id].x;
    // }


    // Save filtered output
    return (0);
}