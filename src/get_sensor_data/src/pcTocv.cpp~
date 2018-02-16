#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>


#include <pcl/filters/voxel_grid.h>//ダウンサンプリング

#include <pcl_conversions/pcl_conversions.h>

//rosメッセージを扱うときのやつ
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

//opencv用
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;

using namespace std;


int
main(int argc,char **argv)
{
 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2()); 
	pcl::io::loadPCDFile (argv[1], *cloud);

	string argv1 = argv[1];

	

//ダウンサンプリング
	//ダウンサンプリングの為にVoxcelGridのオブジェクトをsorとして定義
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    //cloudを処理する点群として指定
    sor.setInputCloud (cloud);
    //ダウンサンプリングする間隔([m]単位)を指定
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    //フィルタリングを実行
    sor.filter(*cloud);

	 //保存
	/*
	string down_name = "downed_" + argv1;
	pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> (down_name, *cloud);
	
	pcl::visualization::CloudViewer viewer ("viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
	*/
	

//OpenCVに変換

	


	sensor_msgs::PointCloud2 msg_pc2;
	pcl::toROSMsg(*cloud,msg_pc2);
	sensor_msgs::Image msg_Img;
	pcl::toROSMsg(msg_pc2, msg_Img);

	cv_bridge::CvImagePtr cv_img_ptr;
	cv_img_ptr = cv_bridge::toCvCopy(msg_Img, sensor_msgs::image_encodings::BGR8);

	cv::Mat cvimg;
	cvimg = cv_img_ptr->image;
	vector<cv::Point2i> points_v;
	cv::Point2i obj_center(0, 0);

//オブジェクトの点を画像として詰めるところからやる!! 18/02/15


/*
	for(int y = 0, y < obj_cvimg.rows; y = y + 5){
		for(int x = 0; x < obj_cvimg.cols; x = x + 5){
        	if(obj_cvimg.at<cv::Vec3b>(y, x)[0] != 0 && //[0]:B [1]:G [2]:R
            	obj_cvimg.at<cv::Vec3b>(y, x)[1] != 0 &&
                obj_cvimg.at<cv::Vec3b>(y, x)[2] != 0){
                	points_v.push_back(cv::Point2i(x, y));
                    obj_center.x += x;
                    obj_center.y += y;
            }
        }
    } 
*/


///////////////////////////////////////////////

	//サイズ取り出し&表示
	uint32_t height = cloud->height;
	uint32_t width = cloud->width;
	
	cout << "width:" << cloud->width << endl;
	cout << "height:" << cloud->height << endl;

	//色の取り出し
	uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[0].rgb);

	int r = ((0x0000FF00 & rgb) >>  16);
	int g = ((0x0000FF00 & rgb) >>  8);
	int b = (rgb)       & 0x0000ff;

	//位置情報の取り出し
	


	
///////////////////////////////////////////////////



/*
	Mat im = imread("makibao.png",1); //カラーで読込

	int height = im.rows;
	int width = im.cols;

	cout <<"height"<< height << endl;
	cout << "width"<< width << endl;
	
	cout << "im.data[0]" << im.data[0] << endl;

	for(int i = 0; i<height; i++)
		for(int j = 0; j<width; j++)
			im.at<cv::Vec3b>(i,j) = Vec3b(255,0,0);

	imshow("Show image", im);		
	waitKey(0);	
*/




	
	return(0); 

} 
