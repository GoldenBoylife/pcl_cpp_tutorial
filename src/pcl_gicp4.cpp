/*
 코드 설명: 
  기존 문제점:voxel하지않고  registration하면 5분걸린다. 

  main맵은 voxel filter -> patch는 정상  ->gicp로 [M] 얻기  
얻은 [M]값이 voxel전과 비슷한지 확인한다. 
 
  
*/

#include <thread>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/common/transforms.h>

#include <pcl/registration/gicp.h>

#include <pcl/filters/voxel_grid.h>     //voxel

#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
using namespace std;

typedef pcl::PointXYZ Point_t;

void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src,pcl::PointCloud<pcl::PointXYZ>& pc_result, double var_voxel_size)
{
    static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pc_src);
    voxel_filter.setLeafSize(var_voxel_size, var_voxel_size, var_voxel_size);
    voxel_filter.filter(pc_result);
}


void viewer_pcds3(pcl::PointCloud<Point_t>::Ptr main_cloud,pcl::PointCloud<Point_t>::Ptr patch_cloud, pcl::PointCloud<Point_t>::Ptr result_cloud) 
{
	//Initializing point cloud visualizer
	pcl::visualization::PCLVisualizer::Ptr
	viewer_final (new pcl::visualization::PCLVisualizer ("3D pcd viwer"));
	viewer_final->setBackgroundColor (0, 0, 0);
	
	// Coloring and visualizing target cloud (red).
	pcl::visualization::PointCloudColorHandlerCustom<Point_t>
	main_color (main_cloud, 255,0, 0); //빨간색
	viewer_final->addPointCloud<pcl::PointXYZ> (main_cloud, main_color, "main cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
													1, "main cloud");


	// Coloring and visualizing transformed input cloud (green).
	pcl::visualization::PointCloudColorHandlerCustom<Point_t>
	patch_color (patch_cloud ,0, 255, 0); //노란색
	viewer_final->addPointCloud<Point_t> (patch_cloud, patch_color, "patch cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "patch cloud");
	// Coloring and visualizing target cloud (red).
	pcl::visualization::PointCloudColorHandlerCustom<Point_t>
	result_color (result_cloud, 0, 0, 255);  //파란색
	viewer_final->addPointCloud<pcl::PointXYZ> (result_cloud, result_color, "result cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
													1, "result cloud");

	// else std::cout << "ICP did not converge." << std::endl;
    // Starting visualizer
    viewer_final->addCoordinateSystem (2.0, "global");  // 좌표계 나온다.  
    viewer_final->initCameraParameters ();

	// Wait until visualizer window is closed.
	while (!viewer_final->wasStopped ())
	{
		viewer_final->spinOnce (100);
		std::this_thread::sleep_for(100ms);
	}


}


int main(int argc, char**argv)
{
    /*----------- param  start------------- */
    float voxelsize= 0.1;
    Eigen::Matrix4f trans_matrix;
    trans_matrix <<     1,   0,  0, 0.305,
                        0,   1,  0, 0.000,
                        0,   0,  1, 0.0,
                        0,   0,  0,     1;

    /*----------- param  end------------- */


    pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // *src = *load_bin("/home/gb/Documents/pcd/kitti00_000000.bin");
    // *src = *load_bin("/home/gb/Documents/pcd/room_scan1.pcd");
    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/room_scan1.pcd", *main_cloud);
    std::cout << "Loaded " << main_cloud->size () << " data points from room_scan1.pcd" << std::endl;

    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/room_scan2.pcd", *patch_cloud);
    std::cout << "Loaded " <<patch_cloud->size() <<"data points from room_scan2.pcd" << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_main_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    
    voxelize(main_cloud, *filtered_main_cloud, voxelsize);
    
    main_cloud = filtered_main_cloud;
    std::cout << "filtered cloud  &&Loaded "<<main_cloud->size() <<"data points " << std::endl;


    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(1.0);
    gicp.setTransformationEpsilon(0.001);
    gicp.setMaximumIterations(1000);

    pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 걸리는 시간 측정
    chrono::system_clock::time_point t_start = chrono::system_clock::now();

    // Registration 시행
    gicp.setInputSource(main_cloud); //src를  main맵으로 해야 하고, voxel해야 한다.
    gicp.setInputTarget(patch_cloud);  
    gicp.align(*align_cloud);

    chrono::system_clock::time_point t_end = chrono::system_clock::now();
    /*******************************************/
    chrono::duration<double> t_reg = t_end - t_start;
    cout<<"Takes "<<t_reg.count()<<" sec..."<<endl;

    // Set outputs
    Eigen::Matrix4f src2tgt   = gicp.getFinalTransformation();
    double score     = gicp.getFitnessScore();
    bool is_converged = gicp.hasConverged();

    cout<<src2tgt<<endl;
    cout<<score<<endl;

	viewer_pcds3(main_cloud, patch_cloud, align_cloud) ;

  }