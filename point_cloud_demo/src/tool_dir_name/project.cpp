
#include "project.h"

//#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>

#include <pcl/console/time.h>  //pcl计算时间
//pcl::console::TicToc time; time.tic();
//+程序段 +
//cout << time.toc() / 1000 << "s" << endl;

//using namespace pcl;



void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 0); //设置背景颜色为黑色
}

bool tool_class::openPcd(std::string pcdFile)
{
	pcl::console::TicToc time; time.tic();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//char strfilepath[256] = "rabbit.pcd";
	if (-1 == pcl::io::loadPCDFile(pcdFile.c_str(), *cloud))
	{
		std::cout << "error input!" << std::endl;
		return false;
	}

	std::cout << cloud->points.size() << std::endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer: Rabbit");

	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	cout << time.toc() / 1000 << "s" << endl;
	system("pause");
	return true;
}

bool tool_class::copyPcd(std::string fromPcd, std::string toPcd)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (-1 == pcl::io::loadPCDFile(fromPcd.c_str(), *cloud))
	{
		std::cout << "error input!" << std::endl;
		return false;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int > indexs = { 1, 2, 5 };
	pcl::copyPointCloud(*cloud, indexs, *cloudOut);


	pcl::visualization::CloudViewer viewer("Cloud Viewer: copy");
	viewer.showCloud(cloudOut);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");

	return true;
}


bool tool_class::link(std::string fpcd, std::string spcd)
{
    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(fpcd.c_str(), *cloud1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(spcd.c_str(), *cloud2);

		// 定义对象
    pcl::visualization::PCLVisualizer viewer;
    // 设置背景颜色，默认黑色
    viewer.setBackgroundColor(100, 100, 100);  // rgb

		// --- 显示点云数据 ----
    // "cloud1" 为显示id，默认cloud,显示多个点云时用默认会报警告。
    viewer.addPointCloud(cloud1, "cloud1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 0, 0);  // rgb
    // 将点云设置颜色，默认白色
    viewer.addPointCloud(cloud2, red, "cloud2");

    // 将两个点连线
    pcl::PointXYZ temp1 = cloud1->points[0];
    pcl::PointXYZ temp2 = cloud1->points[10];

    viewer.addLine(temp1, temp2, "line0"); 

	// --- 显示网格数据 ---
    //pcl::PolygonMesh mesh;
    //pcl::io::loadPLYFile("read.ply", mesh);
    //viewer.addPolygonMesh(mesh);

	viewer.spin();

	system("pause");
    return true;


}






































tool_class::tool_class()
{
	std::cout << "Start tool_class" << std::endl;

}
tool_class::~tool_class()
{
	std::cout << "End tool_class" << std::endl;

}