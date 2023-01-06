#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
using namespace std;

typedef pcl::PointXYZ Point_t;
typedef pcl::PointCloud<Point_t> PointCloud_t;


/*  입력: patch_graph_pcd 5개 
    출력: 5개 합쳐진 pcd 1개

    단계: 
    patch_graph_pcd 하나씩 가져오기 point_cloud;
    map+= point_cloud
    map 출력: 

*/

/*
 입력:point cloud 2개, 각자 크기
 출력:합쳐진 point cloud 1개 , 합쳐진 크기
  
*/
PointCloud_t::Ptr pcl_registration(PointCloud_t::Ptr main_cloud, PointCloud_t::Ptr patch_cloud)
{
    cout <<"pcl_registration 시작" <<endl;
printf("%d\n", __LINE__);
    PointCloud_t::Ptr result(new PointCloud_t);
printf("%d\n", __LINE__);
    pcl::IterativeClosestPoint<Point_t, Point_t> registration;
printf("%d\n", __LINE__);
    registration.setInputSource(main_cloud);
printf("%d\n", __LINE__);
    registration.setInputTarget(patch_cloud);
printf("%d\n", __LINE__);
    registration.align(*result);
    //?
printf("%d\n", __LINE__);
    if(registration.hasConverged())
    {
        std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
printf("%d\n", __LINE__);
		std::cout << "Transformation matrix:" << std::endl;
		std::cout << registration.getFinalTransformation() << std::endl;
    }
    else std::cout << "ICP did not converge." << std::endl;
    //pcl::io::savePCDFile<Point_t>("/home/gb/Documents/pcd/pcl_registration_result.pcd",*finalCloud);

    return result;
    
}
int main(int argc, char** argv)
{

    pcl::PointCloud<Point_t>::Ptr map(new pcl::PointCloud<Point_t>);
    //::Ptr로 선언해야 뒤에서 스마트 포인터 애러 안난다.
    pcl::PointCloud<Point_t>::Ptr maps(new pcl::PointCloud<Point_t>); 

    for(int i =0 ; i < 1 ;i++)
    {
        pcl::PointCloud<Point_t>::Ptr point_cloud(new pcl::PointCloud<Point_t>);
        std::string s1 ="/home/gb/Documents/map/code_test2/ori_commonline_221214_merge/pcds/";
        std::string s2 ="00000";
        std::string s3 = std::to_string(i);
        std::string s4 = ".pcd";
        std::string s = s1+s2+s3+s4;
        cout << "s = "<< s<<endl;
        pcl::io::loadPCDFile<Point_t> (s,*point_cloud);
        map = pcl_registration(map, point_cloud);
        cout <<" i번째" << i << endl;
    }

    
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(map);

    while(!viewer.wasStopped())
    {
    }
    return 0;
    
}

