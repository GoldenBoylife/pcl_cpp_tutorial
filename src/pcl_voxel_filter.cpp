#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ Point_t;

int main (int argc, char** argv)
{
  pcl::PointCloud<Point_t>::Ptr cloud (new pcl::PointCloud<Point_t>);
  pcl::PointCloud<Point_t>::Ptr cloud_filtered (new pcl::PointCloud<Point_t>);

  // *.PCD 파일 읽기 (https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gb/Documents/practice/pcd/table_scene_lms400.pcd", *cloud);

 // 포인트수 출력
  std::cout << "Input : " << cloud->points.size () << " (" << pcl::getFieldsList (*cloud) <<")"<< std::endl;

  // 오브젝트 생성 
  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);              //입력
  sor.setLeafSize (0.01f, 0.01f, 0.01f); //leaf size  1cm 
  sor.filter (*cloud_filtered);          //출력 

  // 생성된 포인트클라우드 수 출력 
  std::cout << "Output : " << cloud_filtered->points.size () << " (" << pcl::getFieldsList (*cloud_filtered) <<")"<< std::endl;

  // 생성된 포인트클라우드 저장 
  pcl::io::savePCDFile<pcl::PointXYZ>("/home/gb/Documents/practice/pcd/table_scene_lms400_downsampled.pcd", *cloud_filtered);
  return (0);
}