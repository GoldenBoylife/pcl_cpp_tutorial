#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{

  //READ #1 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gb/Documents/practice/tabletop.pcd", *cloud); //내부적으로 reader.read() 호출 

  //READ #2
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::io::loadPCDFile<pcl::PointXYZ>("tabletop.pcd", cloud) //내부적으로 reader.read() 호출 

  //READ #3
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PCDReader reader;
  //reader.read<pcl::PointXYZ>("tabletop.pcd", cloud);


  std::cout << "Loaded " << cloud->width * cloud->height  << std::endl; //cloud_filtered->points.size()

  return (0);
}