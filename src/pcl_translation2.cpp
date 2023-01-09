
#include <iostream>
#include <pcl/io/pcd_io.h> //loadPCDFile
// #include <thread>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ Point_t;

void viewer_pcds2(pcl::PointCloud<Point_t>::Ptr main_cloud, pcl::PointCloud<Point_t>::Ptr result_cloud)
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
		// std::this_thread::sleep_for(100ms);
	}
}

pcl::PointCloud<Point_t>::Ptr tranform_cloud (pcl::PointCloud<Point_t>::Ptr input_cloud,Eigen::Matrix4f trans_matrix )
{
    pcl::PointCloud<Point_t>::Ptr result_cloud(new pcl::PointCloud<Point_t>);
    pcl::transformPointCloud(*input_cloud,*result_cloud,trans_matrix);
    return result_cloud;
}

int main(int argc, char**argv) 
{

    /*----------- param  start------------- */
    Eigen::Matrix4f trans_matrix;
    trans_matrix <<     1,   0,  0, 0.305,
                        0,   1,  0, 0.000,
                        0,   0,  1, 0.0,
                        0,   0,  0,     1;

    /*----------- param  end------------- */
    pcl::PointCloud<Point_t>::Ptr input_cloud(new pcl::PointCloud<Point_t>); 
    pcl::io::loadPCDFile<Point_t> ("/home/gb/Documents/practice/pcd/tabletop.pcd",*input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    transformed_cloud = tranform_cloud(input_cloud, trans_matrix);

    viewer_pcds2(input_cloud, transformed_cloud);
}