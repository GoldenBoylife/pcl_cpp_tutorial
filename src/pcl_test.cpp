#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointXYZ Point_t;

int main(int argc, char** argv) 
{
    pcl::PointCloud<Point_t>::Ptr input_cloud(new pcl::PointCloud<Point_t>);
    pcl::PointCloud<Point_t>::Ptr cloud_filtered(new pcl::PointCloud<Point_t>);

    pcl::io::loadPCDFile<Point_t>  ("/home/gb/Documents/pcd/merge_route123+orimap23_commonline_Opti2.pcd", *input_cloud);

    std::cout <<"input : "<< input_cloud->points.size() <<endl;
    
    pcl::VoxelGrid<Point_t> vox_cloud;
    vox_cloud.setInputCloud (input_cloud);
    vox_cloud.setLeafSize(0.2f, 0.2f, 0.2f); 
    vox_cloud.filter(*cloud_filtered);

    std::cout << "Output : "<<cloud_filtered->points.size() <<endl;

    pcl::io::savePCDFile<Point_t> ("/home/gb/Documents/pcd/merge_route123+orimap23_commonline_Opti2_DownSize0.2_result.pcd", *cloud_filtered);
    return (0);

    
}