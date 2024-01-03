
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








































tool_class::tool_class()
{
	std::cout << "Start tool_class" << std::endl;

}
tool_class::~tool_class()
{
	std::cout << "End tool_class" << std::endl;

}