/*
読み込んだXYZRGBデータをダウンサンプリング 

*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h> //ダウンサンプリング用

#include <pcl/visualization/cloud_viewer.h> //PCL(可視化)


int
main(int argc, char** argv)
{

	//0. オリジナルの点を格納するインスタンスを定義
    pcl::PCLPointCloud2::Ptr cloud_origin (new pcl::PCLPointCloud2()); 
	//1.pcdファイルから読込
    
    pcl::io::loadPCDFile (argv[1], *cloud_origin);   

	//2.ダウンサンプリングをする
    //C++の標準出力. ダウンサンプリング前の点群を表示
    //std::cerr << "PointCloud before downsampling: " << cloud_origin->width * cloud_origin->height 
      // << " data points (" << pcl::getFieldsList (*cloud_origin) << ").\n";

    //ダウンサンプリング後の点群を入れる
    pcl::PCLPointCloud2::Ptr cloud_downsampled (new pcl::PCLPointCloud2()); 
     

    //ダウンサンプリングの為にVoxcelGridのオブジェクトをsorとして定義
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    //cloudを処理する点群として指定
    sor.setInputCloud (cloud_origin);
    //ダウンサンプリングする間隔([m]単位)を指定
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    //フィルタリングを実行
    sor.filter(*cloud_downsampled);

	//変換
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_downsampled, *cloud_filtered);
   
    
    std::cerr << "PointCloud after downsampling: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";



    //保存
    string argv1 = argv[1];//argv[1]を文字列型として受け取る。
	string down_name = "downed_" + argv1; 

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> (down_name, *cloud_filtered);

    //可視化for downsampled
    pcl::visualization::CloudViewer viewer_down ("viewer_downsampled");
    viewer_down.showCloud (cloud_filtered);
    while (!viewer_down.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }


	return(0);

}

