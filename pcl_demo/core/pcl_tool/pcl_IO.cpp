
#include "pcl_IO.h"

#include <filesystem>
#include <thread>
#include <pcl/io/pcd_io.h>                   // 读取和写入PCD
#include <pcl/io/ply_io.h>                   // 读取和写入PLY
#include <pcl/visualization/cloud_viewer.h>  // 可视化点云数据的CloudViewer类

#include <pcl/console/time.h>  //pcl计算时间
// pcl::console::TicToc time; time.tic();
//+程序段 +
// cout << time.toc() / 1000 << "s" << endl;

// using namespace pcl;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色
}

bool PclIO::viewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    if (cloud == nullptr)
    {
        std::cout << "The point cloud data is empty" << std::endl;
        return false;
    }

    std::cout << "point size:" << cloud->points.size() << std::endl;
    std::cout << "height:" << cloud->height << std::endl;
    std::cout << "width:" << cloud->width << std::endl;
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer: Rabbit");

    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud(cloud);
    if (normals != nullptr)
    {
        viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
    }
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    // viewer.showCloud(cloud);
    // viewer.runOnVisualizationThreadOnce(viewerOneOff);
    // system("pause");

    std::cout << "End show " << std::endl;
    return true;
}

bool PclIO::viewerPcl(pcl::PCLPointCloud2::Ptr cloud)
{
    if (cloud == nullptr)
    {
        std::cout << "The point cloud data is empty" << std::endl;
        return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);  // 将PCLPointCloud2转换为PointXYZ类型的点云

    std::cout << "point size:" << cloud_xyz->points.size() << std::endl;
    std::cout << "height:" << cloud_xyz->height << std::endl;
    std::cout << "width:" << cloud_xyz->width << std::endl;

    pcl::visualization::CloudViewer viewer("Cloud Viewer: Rabbit");

    viewer.showCloud(cloud_xyz);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    system("pause");
    std::cout << "End show " << std::endl;
    return true;
}

bool PclIO::viewerPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    if (cloud == nullptr)
    {
        std::cout << "The point cloud data is empty" << std::endl;
        return false;
    }

    std::cout << "point size:" << cloud->points.size() << std::endl;
    std::cout << "height:" << cloud->height << std::endl;
    std::cout << "width:" << cloud->width << std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer: Rabbit");

    viewer.showCloud(cloud);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    system("pause");
    std::cout << "End show " << std::endl;
    return true;
}

bool PclIO::viewerPcl(pcl::PointCloud<pcl::PointNormal> cloud_normals)
{
    if (cloud_normals.empty())
    {
        std::cout << "The point cloud data is empty" << std::endl;
        return false;
    }

    std::cout << "Points in cloud_normals: " << cloud_normals.points.size() << std::endl;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer: Normals");
    viewer.setBackgroundColor(0, 0, 0);  // 设置背景色为黑色，提高对比度
    viewer.initCameraParameters();
    viewer.setSize(800, 600);

    // viewer.addPointCloud(cloud_normals.makeShared(), "cloud_normals");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_normals");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_normals");
    viewer.addPointCloudNormals<pcl::PointNormal>(cloud_normals.makeShared(), 1, 0.05, "cloud_normals_normals");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    std::cout << "End show " << std::endl;
    return true;
}

// bool PclIO::viewerPcl(pcl::PointCloud<pcl::PointNormal> cloud_normals)
//{
//     if (cloud_normals.empty())
//     {
//         std::cout << "The point cloud data is empty" << std::endl;
//         return false;
//     }
//
//     std::cout << "point size:" << cloud_normals.points.size() << std::endl;
//     std::cout << "height:" << cloud_normals.height << std::endl;
//     std::cout << "width:" << cloud_normals.width << std::endl;
//
//     pcl::visualization::PCLVisualizer viewer("Cloud Viewer: Normals");
//
//     // 将带有法线的点云可视化
//     viewer.addPointCloudNormals<pcl::PointNormal>(cloud_normals.makeShared());
//
//     while (!viewer.wasStopped())
//     {
//         viewer.spinOnce();
//     }
//
//     std::cout << "End show " << std::endl;
//     return true;
// }

bool PclIO::viewerPcl(pcl::PolygonMesh& triangles)
{
    // 初始化PCL可视化对象
    pcl::visualization::PCLVisualizer viewer("3D Viewer");

    // 设置背景颜色
    viewer.setBackgroundColor(0, 0, 0);

    // 添加三角化后的网格模型
    viewer.addPolygonMesh(triangles, "triangles");

    // 设置相机位置和方向
    viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);

    // 启动可视化
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "End show " << std::endl;
    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclIO::openPointCloudFile(const std::string& filename)
{
    std::string fileExtension = std::filesystem::path(filename).extension().string();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (fileExtension == ".pcd" || fileExtension == ".PCD")
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
        {
            // 如果无法读取文件，则返回空指针
            std::cout << "Unable to open PCD file:" << filename << std::endl;
            return nullptr;
        }
    }
    else if (fileExtension == ".ply" || fileExtension == ".PLY")
    {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1)
        {
            // 如果无法读取文件，则返回空指针
            std::cout << "Unable to open PLY file: " << filename << std::endl;
            return nullptr;
        }
    }
    else
    {
        // 不支持的文件格式
        std::cout << "不支持的文件格式: " << fileExtension << std::endl;
        return nullptr;
    }
    return cloud;
}

pcl::PCLPointCloud2::Ptr PclIO::openPointCloudFile2(const std::string& filename)
{
    std::string fileExtension = std::filesystem::path(filename).extension().string();
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

    if (fileExtension == ".pcd" || fileExtension == ".PCD")
    {
        if (pcl::io::loadPCDFile(filename, *cloud) == -1)
        {
            // 如果无法读取文件，则返回空指针
            std::cout << "Unable to open PCD file:" << filename << std::endl;
            return nullptr;
        }
    }
    else if (fileExtension == ".ply" || fileExtension == ".PLY")
    {
        if (pcl::io::loadPLYFile(filename, *cloud) == -1)
        {
            // 如果无法读取文件，则返回空指针
            std::cout << "Unable to open PLY file: " << filename << std::endl;
            return nullptr;
        }
    }
    else
    {
        // 不支持的文件格式
        std::cout << "不支持的文件格式: " << fileExtension << std::endl;
        return nullptr;
    }
    return cloud;
}

bool PclIO::savePointCloudFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename)
{
    if (!cloud || cloud->empty())
    {
        std::cout << "ERROR : Invalid point cloud data " << std::endl;
        return false;
    }

    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cout << "成功将点云数据写入到文件: " << filename << std::endl;
    return true;
}

bool PclIO::openPcd(std::string pcdFile)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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
    std::cout << "End show " << std::endl;
    return true;
}

bool PclIO::copyPcd(std::string fromPcd, std::string toPcd)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (-1 == pcl::io::loadPCDFile(fromPcd.c_str(), *cloud))
    {
        std::cout << "error input!" << std::endl;
        return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indexs = {1, 2, 5};
    pcl::copyPointCloud(*cloud, indexs, *cloudOut);

    pcl::visualization::CloudViewer viewer("Cloud Viewer: copy");
    viewer.showCloud(cloudOut);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    system("pause");

    return true;
}

bool PclIO::copyPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& toColoud, std::vector<int> copyIndexs)
{
    if (fromCloud == nullptr)
    {
        std::cout << "The point cloud data is empty" << std::endl;
        return false;
    }

    pcl::copyPointCloud(*fromCloud, copyIndexs, *toColoud);
    return true;
}

// bool PclIO::link(std::string fpcd, std::string spcd)
//{
//     // 读取点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile(fpcd.c_str(), *cloud1);
//
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile(spcd.c_str(), *cloud2);
//
//     // 定义对象
//     pcl::visualization::PCLVisualizer viewer;
//     // 设置背景颜色，默认黑色
//     viewer.setBackgroundColor(100, 100, 100);  // rgb
//
//     // --- 显示点云数据 ----
//     // "cloud1" 为显示id，默认cloud,显示多个点云时用默认会报警告。
//     viewer.addPointCloud(cloud1, "cloud1");
//
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud2, 255, 0, 0);  // rgb
//     // 将点云设置颜色，默认白色
//     viewer.addPointCloud(cloud2, red, "cloud2");
//
//     // 将两个点连线
//     pcl::PointXYZ temp1 = cloud1->points[0];
//     pcl::PointXYZ temp2 = cloud1->points[10];
//
//     viewer.addLine(temp1, temp2, "line0");
//
//     // --- 显示网格数据 ---
//     // pcl::PolygonMesh mesh;
//     // pcl::io::loadPLYFile("read.ply", mesh);
//     // viewer.addPolygonMesh(mesh);
//
//     viewer.spin();
//
//     system("pause");
//     return true;
// }