#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

Eigen::Matrix4f ndtRegistration ( pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud, Eigen::Matrix4f init_guess,float &score )
{
  



  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (20);

  // Setting point cloud to be aligned.
  ndt.setInputSource (input_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.



  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;
  score = ndt.getFitnessScore ();
  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
  // Saving transformed input cloud.
  Eigen::Matrix4f returnMatrix = ndt.getFinalTransformation ();
  std::cerr<<ndt.getFinalTransformation ()<<std::endl;
  




  return returnMatrix;
 
}


int main(int argc, char* args[])
{
  std::ofstream outfile;

	int i;
	std::string p;

        outfile.open (p.assign(args[1]).append("/../pose.txt").c_str());

	int num_id=0;
 	
  pcl::visualization::CloudViewer viewer ("mapping viewer");  
 


  pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(4,4);
  Eigen::Matrix4f poseOldRelative = Eigen::Matrix4f::Identity(4,4);

	std::ifstream infile(args[2]);
	std::string temp;
  float score;
  

	while (getline(infile,temp)){
		
		points->clear();
        
		pcl::io::loadPCDFile(p.assign(args[1]).append("/").append(temp.c_str()).c_str(),*points);
    // pcl::io::savePCDFileASCII ("/home/xiesc/test.pcd", points);
    int j  = points->size();
		cout << "Read point cloud with " << j << " belong to " <<temp <<endl;
        

    if (num_id == 0)

    { 
      *mapCloud = *points;
        char poseTime[20];
        strncpy(poseTime,temp.c_str(),20);
      outfile<<temp.substr(0,20)<<"    "<<pose.row(0)<<"   "<<pose.row(1)<<"   "<<pose.row(2)<<std::endl;
      num_id ++;
      continue;

    }

    Eigen::Matrix4f pose2 = ndtRegistration (points,mapCloud, pose , score);

    if (score < 5)//如果匹配成功，就用新的匹配值
    {
      poseOldRelative = pose.inverse()*pose2;
      pose = pose2;
      pcl::transformPointCloud (*points, *mapCloud2, pose);
      *mapCloud2 += *mapCloud; 
      
      pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
      approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
      approximate_voxel_filter.setInputCloud (mapCloud2);
      approximate_voxel_filter.filter (*mapCloud);
      mapCloud2->clear();
    }
    else// 否则用匀速假设，但是不把点pushback回去
    {
      std::cerr<<"error"<<std::endl;
      
    }

    viewer.showCloud(mapCloud);  

    num_id ++;

    char poseTime[20];
    strncpy(poseTime,temp.c_str(),20);
    outfile<<temp.substr(0,20)<<"    "<<pose.row(0)<<"   "<<pose.row(1)<<"   "<<pose.row(2)<<std::endl;
    pose = pose * poseOldRelative;

  }
        

    return 0;




}
