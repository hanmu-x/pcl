
#include "tool_dir_name/project.h"
#include "tool_dir_name/config.hpp"
#include "socket/socket.h"


#include <filesystem>


#include<iostream>

int main() 
{

	std::filesystem::path data_1(DEFAULT_DATA_DIR);
    data_1 += "/tuzi.pcd";
    std::filesystem::path data_2(DEFAULT_DATA_DIR);
    data_2 += "/tuzi_copy.pcd";
	// //打开一个pcd
    //tool_class::openPcd(data_1.string());
	//tool_class::copyPcd(data_1.string(), data_2.string());

    std::string ip = "192.168.142.1"; // 服务器IP地址
    int port = 8080; // 服务器端口号

    //if (Socket::tcpClientSync(ip, port))
    //if (Socket::tcpClientAsyn(ip, port))
    //{
    //    std::cout << "TCP client executed successfully" << std::endl;
    //}
    //else {
    //    std::cerr << "TCP client failed" << std::endl;
    //}

    return 0;
}


//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//
//
//int main(int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(5, 1));
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Fill in the CloudIn data
//    for (auto& point : *cloud_in)
//    {
//        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//    }
//
//    std::cout << "Saved " << cloud_in->size() << " data points to input:" << std::endl;
//
//    for (auto& point : *cloud_in)
//        std::cout << point << std::endl;
//
//    *cloud_out = *cloud_in;
//
//    std::cout << "size:" << cloud_out->size() << std::endl;
//    for (auto& point : *cloud_out)
//        point.x += 0.7f;
//
//    std::cout << "Transformed " << cloud_in->size() << " data points:" << std::endl;
//
//    for (auto& point : *cloud_out)
//        std::cout << point << std::endl;
//
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(cloud_in);
//    icp.setInputTarget(cloud_out);
//
//    pcl::PointCloud<pcl::PointXYZ> Final;
//    icp.align(Final);
//
//    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//        icp.getFitnessScore() << std::endl;
//    std::cout << icp.getFinalTransformation() << std::endl;
//
//    return (0);
//}














//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);//老版的此处可能是vtkRenderingOpenGL
//VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingFreeType);
//
//int user_data;

////该回调函数，在主函数中只注册一次 ，该函数只实现对可视化对象背景颜色的设置
//void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
//{
//    viewer.setBackgroundColor(1.0, 0.5, 1.0);//设置背景颜色
//    std::cout << "我只执行一次" << std::endl;
//
//}
////该回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串
//void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
//{
//    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "play the" << count++;
//    viewer.removeShape("text", 0);//移除旧的文字
//    viewer.addText(ss.str(), 200, 300, "text", 0);//添加新文字
//}
//
//int main()
//{
//    std::filesystem::path data_1(DEFAULT_DATA_DIR);
//    data_1 += "/tuzi.pcd";
//    //创建点云对象
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//    pcl::io::loadPCDFile(data_1.string().c_str(), *cloud);//读取pcd文件到点云对象
//
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//初始化可视化对象
//
//    //showCloud函数是同步的，渲染显示之前程序会一直在此等候
//    viewer.showCloud(cloud);
//
//    //下面的内容是一个模板，支持用户进行更复杂的操作。
//    //该注册函数在可视化的时候只执行一次
//    viewer.runOnVisualizationThreadOnce(viewerOneOff);//
//
//    //该注册函数在渲染输出时每帧都调用
//    viewer.runOnVisualizationThread(viewerPsycho);
//    while (!viewer.wasStopped())
//    {
//        //此处可以添加其他处理
//        user_data++;
//    }
//    return 0;
//}




//int main()
//{
//
//    Config config;
//#ifndef NDEBUG
//    std::string configPath = "../../../../Config/my_config.json";
//#else
//    std::string configPath = "./my_config.json";
//#endif
//    if (config.read_config(configPath))
//    {
//        std::cout << "Read config file succession " << std::endl;
//    }
//    else
//    {
//        std::cout << "ERROR : Failed to read config file " << std::endl;
//        return 1;
//    }
//
//    std::filesystem::path data_1(DEFAULT_DATA_DIR);
//    data_1 += "/geo_db/example6.db";
//
//    tool_class tc;
//
//
//    return 0;
//}
