#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ctime>




 int main (int argc, char *argv[])
 {
    // std::string incloudfile = argv[1];
    std::string incloudfile = "../data/example.pcd";
    time_t tstart, tend; 
    tstart = time(0);


    // Load cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);
    int pnumber = (int)cloud->size ();


    std::cout << "pnumber " << pnumber << std::endl;   
    tend = time(0); 
    std::cout << "It took "<< tend - tstart <<" second(s)."<< std::endl; 


    // // Set up KDTree
    // pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
    // tree->setInputCloud (cloud);

    // // Neighbors containers
    // std::vector<int> k_indices;
    // std::vector<float> k_distances;

    // Main Loop
    tstart = time(0);

    int x;
    for (int point_id = 0; point_id < pnumber; ++point_id)
    {
        x = cloud->points[point_id].x;
    }

    tend = time(0); 
    std::cout << "It took "<< tend - tstart <<" second(s)."<< std::endl; 

    // Save filtered output
    return (0);
 }