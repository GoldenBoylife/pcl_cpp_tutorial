#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals; //cmake 시 set(CMAKE_CXX_FLAGS "-std=c++14")

typedef pcl::PointXYZ Point_t;


void viewer_pcds3(pcl::PointCloud<Point_t>::Ptr patchCloud,pcl::PointCloud<Point_t>::Ptr mainCloud, pcl::PointCloud<Point_t>::Ptr finalCloud) 
{
	//Initializing point cloud visualizer
	pcl::visualization::PCLVisualizer::Ptr
	viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer_final->setBackgroundColor (0, 0, 0);
	

	// Coloring and visualizing target cloud (red).
	pcl::visualization::PointCloudColorHandlerCustom<Point_t>
	patch_color (patchCloud, 255,0, 0);
	//빨간색
	viewer_final->addPointCloud<pcl::PointXYZ> (patchCloud, patch_color, "patch cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
													1, "patch cloud");


	// Coloring and visualizing transformed input cloud (green).
	pcl::visualization::PointCloudColorHandlerCustom<Point_t>
	main_color (mainCloud ,0, 255, 0);
	//노란색
	viewer_final->addPointCloud<Point_t> (mainCloud, main_color, "main cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "main cloud");
	// Coloring and visualizing target cloud (red).
	pcl::visualization::PointCloudColorHandlerCustom<Point_t>
	final_color (finalCloud, 0, 0, 255); 
	//파란색
	viewer_final->addPointCloud<pcl::PointXYZ> (finalCloud, final_color, "final cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
													1, "final cloud");

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


int main (int argc, char** argv)
{
  //첫번째 point cloud loading
  pcl::PointCloud<Point_t>::Ptr patch_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // if (pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/piece/ori_commonline_init_1214_000050.pcd", *target_cloud) == -1)
  // {
  //   PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
  //   return (-1);
  // }
  // pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/room_scan1.pcd", *target_cloud);
  // pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/piece/ori_commonline_221214_merge_000045.pcd", *patch_cloud);
  pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/room_scan1.pcd", *patch_cloud);
  std::cout << "Loaded " << patch_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // /*target_cloud를 이동시키기*/
  // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // float theta =M_PI/4;
  // transform_2.translation() << 0,0,0;
  // //63, 20.0 , 0.0;
  // //47.8658 -6.65082 -2.23915 0.0700049 0.0283966 0.0748911

  // //The same rotation matrix as before; theta radians around Z axis
  // transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  // pcl::PointCloud<Point_t>::Ptr transformed_cloud (new pcl::PointCloud<Point_t> ());
  // pcl::transformPointCloud (*target_cloud, *target_cloud, transform_2);

  // Loading second scan of room from new perspective.
  pcl::PointCloud<Point_t>::Ptr main_cloud (new pcl::PointCloud<Point_t>);
  // if (pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/map/ori_commonline_221214_merge/000050/cloud.pcd", *input_cloud) == -1)
  // {
  //   PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
  //   return (-1);
  // }
  // pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/map/ori_commonline_221214_merge/000050/cloud.pcd", *main_cloud);
  pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/room_scan2.pcd", *main_cloud);
  std::cout << "Loaded " << main_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  /* 속도빠르게 하기 위해서 필터 사용함. */
  pcl::PointCloud<Point_t>::Ptr filtered_main_cloud (new pcl::PointCloud<Point_t>);
  pcl::ApproximateVoxelGrid<Point_t> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (main_cloud);

  approximate_voxel_filter.filter (*filtered_main_cloud);
  std::cout << "Filtered cloud contains " << filtered_main_cloud->size ()
            << " data points from room_scan2.pcd" << std::endl;
  //target cloud는 필터 쓰지 않는다. /voxel grid구조는 각 point별로 쓰지 않고 statical data를 쓰기 때문이다. 

  /*NDT 초기화,내부 데이터 구조는 아직 초기화 안됨.*/
  pcl::NormalDistributionsTransform<Point_t, Point_t> ndt;
  //

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (1);  //원래 0.01
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);  // 원래 0.1
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);  //원래 1.0

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_main_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (patch_cloud);

  // 로봇의 odometry로 찾은 움직인 pose값을 넣어줌 .
  // Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  // Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<Point_t>::Ptr output_cloud (new pcl::PointCloud<Point_t>);
  // ndt.align (*output_cloud, init_guess);
  ndt.align (*output_cloud);  //이거 없으면 segmentation error

        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << ndt.getFinalTransformation() << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*main_cloud, *output_cloud, ndt.getFinalTransformation ());
  // 3번째 인자는 pose에 대한 4x4 메트릭스임
  // Saving transformed input cloud.
  // pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

	viewer_pcds3(patch_cloud, main_cloud, output_cloud) ;



  // // Initializing point cloud visualizer
  // pcl::visualization::PCLVisualizer::Ptr
  // viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer_final->setBackgroundColor (0, 0, 0);

  // // Coloring and visualizing target cloud (red).
  // pcl::visualization::PointCloudColorHandlerCustom<Point_t>
  // target_color (target_cloud, 255, 0, 0);
  // viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  // viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
  //                                                 1, "target cloud");

  // // Coloring and visualizing transformed input cloud (green).
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  // output_color (output_cloud, 0, 255, 0);
  // viewer_final->addPointCloud<Point_t> (output_cloud, output_color, "output cloud");
  // viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
  //                                                 1, "output cloud");

  // // Starting visualizer
  // viewer_final->addCoordinateSystem (1.0, "global");
  // viewer_final->initCameraParameters ();

  // // Wait until visualizer window is closed.
  // while (!viewer_final->wasStopped ())
  // {
  //   viewer_final->spinOnce (100);
  //   std::this_thread::sleep_for(100ms);
  // }

  return (0);
}