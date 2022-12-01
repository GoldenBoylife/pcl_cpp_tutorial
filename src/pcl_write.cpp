#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //SAVE #1
  //#pcl::PointCloud<pcl::PointXYZ> cloud; //SAVE #2


  // Fill in the cloud data
  cloud->width = 100;
  cloud->height = 20;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

/*랜덤으로 100*20개 사이즈로 pcl 만들기 */
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }


  //SAVE #1
  pcl::io::savePCDFile<pcl::PointXYZ>("/home/gb/Documents/practice/test_pcd.pcd", *cloud); //Default binary mode save


  //SAVE #2
  //내부적으로 writer.write 호출
  //pcl::io::savePCDFile<pcl::PointXYZ>("test_pcd.pcd", cloud); //Default binary mode save
  //pcl::io::savePCDFileASCII<pcl::PointXYZ>("test_pcd_ASCII.pcd", cloud); //ASCII mode
  //pcl::io::savePCDFileBinary<pcl::PointXYZ>("test_pcd_Binary.pcd", cloud); //binary mode 




  //SAVE #3
  //pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ>("test_pcd.pcd", cloud); //Default binary mode save
  //writer.writeASCII<pcl::PointXYZ>("test_pcd_ASCII.pcd", cloud); //ASCII mode
  //writer.writeBinary<pcl::PointXYZ>("test_pcd_Binary.pcd", cloud); //binary mode
  //options. writer.write("test.pcd", *cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);


  std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;


  return (0);
}