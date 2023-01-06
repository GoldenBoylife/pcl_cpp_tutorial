#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <thread>


#include <iostream>
using namespace std;
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
int main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<Point_t>::Ptr patchCloud(new pcl::PointCloud<Point_t>);
	pcl::PointCloud<Point_t>::Ptr mainCloud(new pcl::PointCloud<Point_t>);
	pcl::PointCloud<Point_t>::Ptr finalCloud(new pcl::PointCloud<Point_t>);


	// Read two PCD files from disk.
	// if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *sourceCloud) != 0)
	// {
	// 	return -1;
	// }
	// if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *targetCloud) != 0)
	// {
	// 	return -1;
	// }

    // pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/ori_commonline_init_1214.pcd", *targetCloud);
    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/pcd/piece/ori_commonline_221214_merge_000045.pcd",*patchCloud);
    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/map/ori_commonline_221214_merge/000050/cloud.pcd",*mainCloud);


	// ICP object.
	pcl::IterativeClosestPoint<Point_t, Point_t> registration;
	registration.setInputSource(patchCloud);
	registration.setInputTarget(mainCloud);

	registration.align(*finalCloud);
	if (registration.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
		std::cout << registration.getFinalTransformation() << std::endl;
	}

	viewer_pcds3(patchCloud, mainCloud, finalCloud) ;

	

    // pcl::io::savePCDFile<Point_t>("/home/gb/Documents/pcd/pcl_registration_result.pcd",*finalCloud);

    return (0);
}