#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace std;

typedef pcl::PointXYZ Point_t;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main( int argc, char** argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);
    
    /*topic 안에 pcl 넣기*/
    // PointCloud::Ptr msg (new PointCloud);
        
    pcl::PointCloud<Point_t>::Ptr input_cloud_msg(new pcl::PointCloud<Point_t>); 
    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/practice/pcd/tabletop.pcd",*input_cloud_msg);

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
    pcl::transformPointCloud (*input_cloud_msg, *transformed_cloud, transform_2);
    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;


    input_cloud_msg->header.frame_id = "pcl_frame";
    //frame을 정해줘야 rviz에서 fix해서 볼 수 있음. 

    //없어도됨. input_cloud_msg->height = input_cloud_msg->width = 50;
    //없어도됨.  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
    

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), transformed_cloud->header.stamp); 
        //??
        pub.publish (*transformed_cloud);
        ros::spinOnce ();
        loop_rate.sleep ();
    }

    
}