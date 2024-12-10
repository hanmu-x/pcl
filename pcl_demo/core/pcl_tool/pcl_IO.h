
#ifndef PCL_IO_H
#define PCL_IO_H

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>  // 点云类型
#include <pcl/point_types.h>  //点数据类型
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/conditional_removal.h>  // ConditionalRemoval 移除离群点

class PclIO
{
  public:
    /// <summary>
    /// 通过pcd点云指针打开,可视化展示
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static bool viewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals = nullptr);

    /// <summary>
    /// pcl::PCLPointCloud2::Ptr点云,可视化展示
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static bool viewerPcl(pcl::PCLPointCloud2::Ptr cloud);
    static bool viewerPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    static bool viewerPcl(pcl::PointCloud<pcl::PointNormal> cloud_normals);
    static bool viewerPcl(pcl::PolygonMesh& triangles);

    /// <summary>
    /// 打开点云数据文件(pcd,ply)文件
    /// </summary>
    /// <param name="file"></param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr openPointCloudFile(const std::string& filename);

    /// <summary>
    /// 打开点云文件保存到pcl::PCLPointCloud2::Ptr
    /// </summary>
    /// <param name="filename"></param>
    /// <returns></returns>
    static pcl::PCLPointCloud2::Ptr openPointCloudFile2(const std::string& filename);

    /// <summary>
    /// 保存点云到指定的文件中
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="filename"></param>
    /// <returns></returns>
    static bool savePointCloudFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename);

    /// <summary>
    /// 打开点pcd云文件,可视化展示
    /// </summary>
    /// <param name="pcdFile"></param>
    /// <returns></returns>
    static bool openPcd(std::string pcdFile);

    /// <summary>
    /// 复制一个点云文件到另一个点云文件中
    /// </summary>
    /// <param name="fromPcd"></param>
    /// <param name="toPcd"></param>
    /// <returns></returns>
    static bool copyPcd(std::string fromPcd, std::string toPcd);

    /// <summary>
    /// 将 fromCloud 点云中的 copyIndexs 索引值的点赋值到 toColoud点云中去
    /// </summary>
    /// <param name="fromCloud"></param>
    /// <param name="toColoud"></param>
    /// <param name="copyIndexs"></param>
    /// <returns></returns>
    static bool copyPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& toColoud, std::vector<int> copyIndexs);

    ///// <summary>
    ///// 连接点云
    ///// </summary>
    ///// <param name="fpcd"></param>
    ///// <param name="spcd"></param>
    ///// <returns></returns>
    // static bool link(std::string fpcd, std::string spcd);

    /// <summary>
    /// 点云中绘制立方体
    /// </summary>
    /// <param name="fromCloud"></param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr drawCube(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif  // PCL_IO_H
