#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloud_cb (const PointCloud::ConstPtr& input){
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	PointCloud::Ptr cloud_filtered (new PointCloud);

	//downsampling the point cloud
	vg.setInputCloud (input);
  	vg.setLeafSize (0.01f, 0.01f, 0.01f);
  	vg.filter (*cloud_filtered);

  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud_filtered);

  	//Euclidean clustering
  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (1);
  	ec.setMinClusterSize (25);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud_filtered);
  	ec.extract (cluster_indices);

  	std::cout << "total points unfiltered: " << input->points.size() << " data points." << std::endl;
  	std::cout << "total points filtered: " << cloud_filtered->points.size() << " data points." << std::endl;

  	//display the clusters
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
  			it != cluster_indices.end (); ++it){

    	PointCloud::Ptr cloud_cluster (new PointCloud);
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      		cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    	}

    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;

    	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () 
    		<< " data points." << std::endl;
  }

  std::cout << " " << std::endl;
}


int main (int argc, char** argv){

  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe <PointCloud> ("/lidar/cloud", 1, cloud_cb);
  ros::spin ();
}