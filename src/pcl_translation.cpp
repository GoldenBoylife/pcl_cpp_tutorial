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
    float theta = M_PI/4;       //M_PI/4로해야 루트 같음. 
    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 50., 0.0, 0.0;


    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Executing the transformation
    pcl::PointCloud<Point_t>::Ptr transformed_cloud (new pcl::PointCloud<Point_t> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*input_cloud, *transformed_cloud, transform_2);


    /*회전 변환행렬 3번째: 행렬 형태로 넣기. */
    Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();
    transform_3 (0,0) = 0.994088;transform_3 (0,1) = 0.108174;transform_3 (0,2) = -0.00950684;transform_3 (0,3) = -0.413594;
    transform_3 (1,0) = -0.108173;transform_3 (1,1) = 0.994133;transform_3 (1,2) = 0.000589963;transform_3 (1,3) = 0.0463004;
    transform_3 (2,0) = 0.00951481;transform_3 (2,1) = -0.000442023;transform_3 (2,2) =0.999955;transform_3 (2,3) = 0.0593671;
    transform_3 (3,0) = 0.0;transform_3 (3,1) = 0.0;transform_3 (3,2) = 0.0;transform_3 (3,3) = 1.0;


    //인자: 입력, 출력, 메트릭스  -> 이걸로 이동시킴. 
    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;
    pcl::io::savePCDFile<Point_t>("/home/gb/Documents/practice/pcd/test_pcd.pcd", *transformed_cloud);



   

    cout << "Point number : "<< input_cloud->width * input_cloud->height <<endl; 
    return 0;

    
}