#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
using namespace std;

typedef pcl::PointXYZ Point_t;


int main( int argc, char** argv)
{


    
    pcl::PointCloud<Point_t>::Ptr input_cloud(new pcl::PointCloud<Point_t>); 

    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/practice/pcd/tabletop.pcd",*input_cloud);

    /*회전 변환행렬*/
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    float theta = M_PI/4;
    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 50., 0.0, 0.0;


    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Executing the transformation
    pcl::PointCloud<Point_t>::Ptr transformed_cloud (new pcl::PointCloud<Point_t> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*input_cloud, *transformed_cloud, transform_2);
    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    pcl::io::savePCDFile<Point_t>("/home/gb/Documents/practice/pcd/test_pcd.pcd", *transformed_cloud);


   

    cout << "Point number : "<< input_cloud->width * input_cloud->height <<endl; 
    return 0;

    
}