#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;

typedef pcl::PointXYZ Point_t;


int main( int argc, char** argv)
{
    pcl::PointCloud<Point_t>::Ptr input_cloud(new pcl::PointCloud<Point_t>); 

    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/practice/pcd/tabletop.pcd",*input_cloud);


    cout << "Point number : "<< input_cloud->width * input_cloud->height <<endl; 
    return 0;

    
}