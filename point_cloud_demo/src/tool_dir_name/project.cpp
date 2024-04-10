
#include "project.h"

//#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/cloud_viewer.h>

#include <pcl/console/time.h>  //pcl����ʱ��
//pcl::console::TicToc time; time.tic();
//+����� +
//cout << time.toc() / 1000 << "s" << endl;

//using namespace pcl;



void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 0); //���ñ�����ɫΪ��ɫ
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
    // ��ȡ����
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(fpcd.c_str(), *cloud1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(spcd.c_str(), *cloud2);

		// �������
    pcl::visualization::PCLVisualizer viewer;
    // ���ñ�����ɫ��Ĭ�Ϻ�ɫ
    viewer.setBackgroundColor(100, 100, 100);  // rgb

		// --- ��ʾ�������� ----
    // "cloud1" Ϊ��ʾid��Ĭ��cloud,��ʾ�������ʱ��Ĭ�ϻᱨ���档
    viewer.addPointCloud(cloud1, "cloud1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 0, 0);  // rgb
    // ������������ɫ��Ĭ�ϰ�ɫ
    viewer.addPointCloud(cloud2, red, "cloud2");

    // ������������
    pcl::PointXYZ temp1 = cloud1->points[0];
    pcl::PointXYZ temp2 = cloud1->points[10];

    viewer.addLine(temp1, temp2, "line0"); 

	// --- ��ʾ�������� ---
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