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




    input_cloud_msg->header.frame_id = "pcl_frame";
    //frame을 정해줘야 rviz에서 fix해서 볼 수 있음. 

    //없어도됨. input_cloud_msg->height = input_cloud_msg->width = 50;
    //없어도됨.  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
    

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), input_cloud_msg->header.stamp); 
        //??
        pub.publish (*input_cloud_msg);
        ros::spinOnce ();
        loop_rate.sleep ();
    }

    
}