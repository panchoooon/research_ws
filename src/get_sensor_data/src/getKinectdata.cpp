#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h> //convert用
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <pcl/visualization/cloud_viewer.h> //PCL(可視化)


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloud;

/*170207 なぜかihsrbができないので、とりあえずkinectから取得する*/
//string topic_name = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points"; 

string topic_name = "/camera/depth_registered/points";
ros::Publisher pub_in;
ros::Publisher pub_pc2;

//pcl::PointCloud<pcl::PointXYZRGB> cloud_output;

int saveflag = 0;
string num;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{

	cout <<"Now saving" << endl;

	
	
	
	//sensor_msgs/PointCloud2　型のメッセージをPublish
	/*sensor_msgs::PointCloud2 output;
	output = *input;
	pub_in.publish(output);*/

	//sensor_msgs/PointCloud2　から pcl::PCLPointCloud2へ型変換
	/*pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input, pcl_pc2);
	*/

	
	pcl::PointCloud<pcl::PointXYZRGB> cloud_output;// (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromROSMsg(*input, cloud_output);

	string file_name =  "cloud" + num + ".pcd";

	pcl::io::savePCDFileASCII (file_name, cloud_output);

	saveflag = 1;

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "getKinectdata");//ノード初期化
	ros::NodeHandle nh;
	
	pub_in = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);	


	cout << "pcdファイルの番号を入力"<< endl;
	cout << "例. 入力 「2」 →「 cloud2.pcd」を出力"<< endl;

	cin >> num;

	cout << num << endl;

	ros::Subscriber sub = nh.subscribe(topic_name, 100, callback);//サブスクライブ
	
	

	//pcl::io::savePCDFileASCII ("test_pcd2.pcd", cloud_output);

	ros::Rate spin_rate(10); //入力したHzでプログラムがループする 


	int i = 0;

	while (ros::ok()) {
    ros::spinOnce();
    	if (saveflag==1){
     	 exit(1);
     	 }
    	else{
    	  i+=1;
      	  spin_rate.sleep();
    	}
  	}

	
	//while(saveflag == 0);
	//	ros::spin();

	

	return 0;
}
