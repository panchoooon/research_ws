#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>


int
main (int argc, char** argv)
{
	ros::init(argc, argv, "downsampled");//ノード初期化
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("test_topic", 1);

	//Publishする点群を読込
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile(argv[1], *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  	// Generate pointcloud data
  	cloud->width = 1000;
  	cloud->height = 1;
  	cloud->points.resize (cloud->width * cloud->height);

  	for (size_t i = 0; i < cloud->points.size (); ++i)
  	{
    	cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    	cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    	cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  	}


	//Cloud2にconvert
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud, output);

	//Publish
	pub.publish(output);	

	ros::spin();

}
