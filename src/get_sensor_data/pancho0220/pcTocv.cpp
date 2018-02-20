#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>//.pcd 可視化
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
	//色情報取得に使う
 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2()); 

	pcl::io::loadPCDFile (argv[1], *cloud);
	
	string argv1 = argv[1];

	cout << "cloud->width. "  << cloud->width << endl;
	cout << "cloud->height. " << cloud->height << endl;
	cout <<"original cloud size. " <<cloud->width*cloud->height << endl;


//ダウンサンプリング
	
	//(4:3)の画像作成の為12で割り切れる点群数になるまで回す。
	int flag = 0;
	double n = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_down (new pcl::PointCloud<pcl::PointXYZRGB>);
	while(flag == 0)
	{
			
    	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		//pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    	//cloudを処理する点群として指定
    	sor.setInputCloud (cloud);
    	//ダウンサンプリングする間隔([m]単位)を指定
    	sor.setLeafSize (n + 0.0012f, n + 0.0014f, n + 0.0014f);
		//sor.setLeafSize (0.0022f, 0.0022f, 0.0022f);

    	//フィルタリングを実行
    	sor.filter(*cloud_down);
		//if(cloud_down->width  % 7 == 0){
		flag = 1;
		//cout <<"cloud->points[0].x. " << cloud->points[0].x << endl;
		cout << "flag == 1" << endl;
		cout << "n = " << n << endl;
		//}
		//cout << "n = " << n << endl;

		n = n + 0.00005;
	}
	/*
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.000001f, 0.000001f, 0.000001f);
	sor.filter(*cloud_down);
	*/
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_down (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::fromPCLPointCloud2(*cloud_tmp, *cloud_down);


	cout <<"down width. " << cloud_down->width << endl;
	cout <<"down height. " << cloud_down->height << endl;

	//可視化for downsampled/*
    /*pcl::visualization::CloudViewer viewer ("viewer_downsampled");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }*/
	
//点群を画像に変換
/*
	//手法1.一旦ROSMsgに変換するやつ
	sensor_msgs::PointCloud2 msg_pc2;
	pcl::toROSMsg(*cloud,msg_pc2);
	sensor_msgs::Image msg_Img;
	pcl::toROSMsg(msg_pc2, msg_Img);

	cv_bridge::CvImagePtr cv_img_ptr;
	cv_img_ptr = cv_bridge::toCvCopy(msg_Img, sensor_msgs::image_encodings::BGR8);

	cv::Mat cvimg;
	cvimg = cv_img_ptr->image;
	vector<cv::Point2i> points_v;
	cv::Point2i center(0, 0);

	cv::imshow("cvimg", cvimg);
	cv::waitKey(0);

	
	
//オブジェクトの点を画像として詰めるところからやる!! 18/02/15

	for(int y = 0; y < cvimg.rows; y = y + 5){
		for(int x = 0; x < cvimg.cols; x = x + 5){
        	if(cvimg.at<cv::Vec3b>(y, x)[0] != 0 && //[0]:B [1]:G [2]:R
            	cvimg.at<cv::Vec3b>(y, x)[1] != 0 &&
                cvimg.at<cv::Vec3b>(y, x)[2] != 0){
                	points_v.push_back(cv::Point2i(x, y));
                    center.x += x;
                    center.y += y;
            }
        }
    } 

	//ofstream outputfile("pcTocv_image.txt");
	//outputfile << cvimg;
	//outputfile.close();


	//imshow("Created Image1", cvimg);
	//waitKey(0);	

*/	
///////////////////////////////////////////////////


	//手法2. 地道に点群を黒画像に格納
	float ratio = cloud_down->width / (float)(cloud->width * cloud->height);
	cout <<"ratio. " << ratio << endl; 
	//(4:3)の黒画像を作成 
	int c_size = cloud_down->width;
	//c_size = iw×ih, w:h = 4:3 よりiw, ihを求める。
	//int iw = (int)sqrt(4/3*c_size); //iw: image width
	//int ih = c_size/iw; //ih: image height
	
	//int c_size = cloud->points.size();
	int iw = cloud->width;
	int ih = cloud->height;
	

	cout << "c_size. " << c_size << endl;
	cout << "iw. " << iw * ratio << endl;
	cout << "ih. " << ih * ratio << endl;

	cv::Mat img = cv::Mat::zeros(ih*ratio+1,iw*ratio+1,CV_8UC3);
	//imshow("Black Image", img);		
	//waitKey(0);	

	//リサイズ
	cloud_down->width = iw * ratio;
	cloud_down->height = ih * ratio;

		
	//点群のRGB値を黒画像の値を入れ替える。
	for(int i = 0; i < ih*ratio; i++){
		for(int j = 0; j < iw*ratio; j++){
			//色の取り出し
			//手法1
			uint32_t rgb = * reinterpret_cast<int*>(&cloud_down->points[iw*i+j+1].rgb);
			int r = ((0x0000FF00 & rgb) >>  16);
			int g = ((0x0000FF00 & rgb) >>  8);
			int b = (rgb)       & 0x0000ff;
			//手法2
			//int r = cloud_down->points[iw*i+j+1].r;
			//int g = cloud_down->points[iw*i+j+1].g;
			//int b = cloud_down->points[iw*i+j+1].b;

			img.at<cv::Vec3b>(i,j) = Vec3b(b,g,r);
		}
	}
	
	imshow("Created Image", img);		
	waitKey(0);	
	
/*	
	Mat im = imread("makibao.jpg",1); //カラーで読込

	int row = im.rows;
	int col = im.cols;

	cout <<"row"<< row << endl;
	cout <<"col"<< col << endl;
	
	cout << "im.data[0]" << im.data[0] << endl;

	//画像アクセス
	for(int i = 0; i<row; i++)
		for(int j = 0; j<col; j++)
			//im.at<cv::Vec3b>(i,j) = Vec3b(0,0,0);//(B,G,R)

	imshow("Show Image", im);		
	waitKey(0);	
*/


	
	return(0); 

} 
