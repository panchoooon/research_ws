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

#include <string.h>


using namespace cv;

using namespace std;

int main(int argc, char **argv)
{
    int data_size = 100;

    for(int n = 0; n < data_size; n++){

    //1.読込
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        string str1("2Tshirts/");
        stringstream ss;
        if (n == 1){
            ss << n + 1;
        }
        else{
            ss << n;
        }
        string str2 = ss.str();
        string file_name = str1 + str2 + "_ex1.pcd";
        pcl::io::loadPCDFile (file_name, *cloud);
        //オリジナル点群の基本情報
        int cw = cloud->width;
        int ch = cloud->height;
        int csize = cw*ch;

    //2.縮小
        int ratio = 2;//縮小率
        int Rcw = cw / ratio;
        int Rch = ch / ratio;
        int Rcsize = Rcw * Rch;

        cv::Mat img = cv::Mat::zeros((ch*0.9)/ratio,Rcw,CV_8UC3);  
        //周辺の画素から一様分布に従ってランダムな値を代表値として取る。
        //cout <<RAND_MAX << endl;//RAND_MAX. <stdlib.h>の定数
        int max = ratio*2 -1;
        int min = 0;

         //点群のRGB値を黒画像の値を入れ替える。
	    for(int i = 0; i < ch*0.9; i = i + ratio){
		    for(int j = 0; j < cw; j = j +ratio){
			    //乱数指定
                int flag = min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
                //色の取り出し
			
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
    

    string str3("2Tshirts_P/");
    stringstream ss2;
    if (n == 1){
        ss2 << n + 1;
    }
    else{
        ss2 << n;
    }
    string str4 = ss2.str();
    string save_name = str3 + str4 + "_ex1.png";//preprocessed
 
    
    //cout << save_name << endl;
    imwrite(save_name, img);
    
        if(n % 10 == 0 && n >= 1)
            cout << n << " cloud is finished" << endl;       



    }

    cout << "All finished" << endl;


    return(0);
}