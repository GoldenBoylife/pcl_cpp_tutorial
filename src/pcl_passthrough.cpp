#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//Filtering a PointCloud using a PassThrough filter


int main(int arg, char** argv)
{
    typedef pcl::PointXYZRGB Point_t;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::pointXYZRGB);

    pcl::PointCloud<Point_t>::Ptr cloud (new pcl::PointCloud<Point_t>);
    pcl::PointCloud<Point_t>::Ptr cloud_filtered(new pcl::PointCloud<Point_t>);
    //Ptr대신 * 쓰면 building 안됨. 


    /*파일 읽기*/
    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/practice/pcd/tabletop.pcd", *cloud);


    /*포인트수 출력*/
    std::cout << "Loaded : "<< cloud->width * cloud->height << std::endl;


    /*오브젝트 생성*/
    pcl::PassThrough<Point_t> pass;
    pass.setInputCloud(cloud);          //입력 cloud
    pass.setFilterFieldName("z");       //적용할 좌표 축 정하기
    pass.setFilterLimits(0.70,1.5);      //적용할 값( 최소값, 최대값)
    //pass.setFilterLimitsNegative(true);       //적용할 값 외
    pass.filter(*cloud_filtered);       //필터 적용

    std::cout <<"Filtered : " << cloud_filtered->width * cloud_filtered->height << std::endl;
    //포인트 수 출력

    pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/gb/Documents/practice/pcd/tabletop_passthrough.pcd", *cloud_filtered);

    return (0);

}