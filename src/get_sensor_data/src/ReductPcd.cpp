#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>//.pcd 可視化
#include <pcl/filters/passthrough.h>
#include <pcl/filters/bilateral.h>
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

//平面検出用
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


//#include <random>
#include <stdlib.h>



using namespace cv;

using namespace std;


int
main(int argc,char **argv)
{

//1. 読込	
 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

	pcl::io::loadPCDFile (argv[1], *cloud);
	
	string argv1 = argv[1];

    //cw.cloud width, ch. cloud height
    int cw = cloud->width;
    int ch = cloud->height;
    int csize = cw*ch;

	cout << "original width. "  << cw << endl;
	cout << "original height. " << ch << endl;
	cout <<"original size. " << csize << endl;
    cout << endl;

//2.Passthrough filter
/*    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);

    int Pcw = cloud->width;
    int Pch = cloud->height;
    int Pcsize = cw*ch;

    cout << "Passed width. "  << Pcw << endl;
	cout << "Passed height. " << Pch << endl;
	cout << "Passed size. " << Pcsize << endl;
    cout << endl;
*/

//2.平面検出    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);

    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    float param = 0.0025;
    seg.setDistanceThreshold (param);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    //平面が検出されない場合
    if(inliers->indices.size() == 0){
        ROS_INFO("[obj_proc]: There is no plane.");
        return(-1);
    }
    //平面を青色にする
    for(size_t i = 0; i < inliers->indices.size(); ++i){
        cloud->points[inliers->indices[i]].r = 0;
        cloud->points[inliers->indices[i]].g = 0;
        cloud->points[inliers->indices[i]].b = 255;
    }


//3. 縮小 Reduction

    int ratio = 2;//縮小率
    int Rcw = cw / ratio;
    int Rch = ch / ratio;
    int Rcsize = Rcw * Rch;


    cout << "Reducted width. "  << Rcw << endl;
	cout << "Reducted height. " << Rch << endl;
	cout << "Reducted size. " << Rcsize << endl;

    cv::Mat img = cv::Mat::zeros((ch*0.9)/ratio,Rcw,CV_8UC3);  


    //周辺の画素から一様分布に従ってランダムな値を代表値として取る。
    cout <<RAND_MAX << endl;//RAND_MAX. <stdlib.h>の定数
    int max = ratio*2 -1;
    int min = 0;
    //点群のRGB値を黒画像の値を入れ替える。
	for(int i = 0; i < ch*0.9; i = i + ratio){
		for(int j = 0; j < cw; j = j +ratio){
			//乱数指定
            int flag = min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));


            //色の取り出し
			//int r = cloud->points[cw*i+j].r;
			//int g = cloud->points[cw*i+j].g;
			//int b = cloud->points[cw*i+j].b;
            //cout << "(w,h). (i,j). = (" << i << "," << j << ")." << endl; 
            int r,g,b;
            switch(flag){
                case 0://左上の画素を取る
                    r = cloud->points[cw*i+j].r;
			        g = cloud->points[cw*i+j].g;
			        b = cloud->points[cw*i+j].b;                    

                case 1://右上の画素
                    r = cloud->points[cw*i+j+1].r;
			        g = cloud->points[cw*i+j+1].g;
			        b = cloud->points[cw*i+j+1].b;

                case 2://左下の画素
                    r = cloud->points[cw*(i+1)+j].r;
			        g = cloud->points[cw*(i+1)+j].g;
			        b = cloud->points[cw*(i+1)+j].b;

                case 3://右下の画素
                    r = cloud->points[cw*(i+1)+j+1].r;
			        g = cloud->points[cw*(i+1)+j+1].g;
			        b = cloud->points[cw*(i+1)+j+1].b;

            }
            img.at<cv::Vec3b>(i/ratio,j/ratio) = Vec3b(b,g,r);
			
		}
	}
    cout << "After Reducted" << endl; 
    imshow("Reducted Image", img);
    //imwrite(name,img);
    waitKey(0);
    
    return(0);

}