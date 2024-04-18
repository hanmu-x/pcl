
#include "pcl_tool/pcl_tool.h"
#include "pcl_tool/config.hpp"
#include "socket/socket.h"

#include <filesystem>
#include<iostream>

#include <pcl/io/pcd_io.h>



int main()
{
    std::filesystem::path data_1(DEFAULT_DATA_DIR);
    data_1 += "/tuzi.pcd";

    std::filesystem::path data_2(DEFAULT_DATA_DIR);
    data_2 += "/consensus.pcd";

    std::filesystem::path data_3(DEFAULT_DATA_DIR);
    data_3 += "/table_scene_lms400.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud = PclTool::openPointCloudFile(data_3.string());


    //pcl::PointCloud<pcl::PointXYZ>::Ptr Projection_xy = PclTool::cloudProjection(table_cloud, 0.0, 0.0, 1.0, 0.0);

    //PclTool::viewerPcl(Projection_xy);

    pcl::PointCloud<pcl::PointXYZ>::Ptr Projection_yz = PclTool::cloudProjection(table_cloud, 1.0, 0.0, 0.0, 0.0);

    PclTool::viewerPcl(Projection_yz);

    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr scrfilter_table = PclTool::statisticalOutlierRemovalFilter(table_cloud, 50, 1.0, true);

    PclTool::viewerPcl(scrfilter_table);

    return 0;

    pcl::PCLPointCloud2::Ptr cloud2_table_cloud2 = PclTool::openPointCloudFile2(data_3.string());

    pcl::PointCloud<pcl::PointXYZ>::Ptr concloud = PclTool::openPointCloudFile(data_2.string());


    pcl::PCLPointCloud2::Ptr cloud2_table_fl = PclTool::voxelGridFilter(cloud2_table_cloud2, 0.1, 0.1, 0.1);

    PclTool::viewerPcl(cloud2_table_fl);
    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr tuzi = PclTool::openPointCloudFile(data_1.string());
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthfilter = PclTool::passThroughFilter(tuzi, "x", -0.05, 0.02,  false);

    PclTool::viewerPcl(passthfilter);
    return 0;




    std::vector<int> index2 = PclTool::randomSampleConsensus(concloud, 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (PclTool::copyPcd(concloud, outcloud, index2))
    {
        // 打开pcd
        PclTool::viewerPcl(outcloud);
    }

    return 0;



    // 随机生成一个索引
    int randomIndex = rand() % tuzi->size();
    // 获取随机点的坐标
    pcl::PointXYZ searchPoint = tuzi->points[randomIndex];
    // 执行k紧邻搜索
    std::vector<int> index = PclTool::octreeRadiusSearch(tuzi, 10, searchPoint, 0.12);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>);

    //if (PclTool::copyPcd(cloud_tuzi, outcloud, index))
    //{
    //    // 打开pcd
    //    PclTool::viewerPcl(outcloud);
    //}




    //std::filesystem::path data_2(DEFAULT_DATA_DIR);
    //data_2 += "/tuzi_copy.pcd";

    //PclTool::link(data_1.string(), data_2.string());

	return 0;
}


















//int main() 
//{
//
//	std::filesystem::path data_1(DEFAULT_DATA_DIR);
//    data_1 += "/tuzi.pcd";
//    std::filesystem::path data_2(DEFAULT_DATA_DIR);
//    data_2 += "/tuzi_copy.pcd";
//	// //打开一个pcd
//    //PclTool::openPcd(data_1.string());
//	//PclTool::copyPcd(data_1.string(), data_2.string());
//
//    std::string ip = "192.168.0.163"; // 服务器IP地址
//    int port = 8080; // 服务器端口号
//    //Socket so;
//    ////if (Socket::tcpClientSync(ip, port))  // 同步
//    //if (so.tcpClientAsyn(ip, port))  // 异步
//    // {
//    //    std::cout << "TCP client executed successfully" << std::endl;
//    //}
//    //else {
//    //    std::cout << "TCP client failed" << std::endl;
//    //}
//
//    if (Socket::updClientSync(ip, port))  // 同步
//    {
//        std::cout << "TCP client executed successfully" << std::endl;
//    }
//    else {
//        std::cout << "TCP client failed" << std::endl;
//    }
//
//    return 0;
//
//
//}











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
//    PclTool tc;
//
//
//    return 0;
//}
