
#include "pcl_tool.h"
#include <filesystem>

#include <pcl/io/pcd_io.h>  // 读取和写入PCD
#include<pcl/io/ply_io.h> // 读取和写入PLY
#include <pcl/visualization/cloud_viewer.h>  // 可视化点云数据的CloudViewer类

#include <pcl/kdtree/kdtree_flann.h>  //kdtree类定义头文件
#include <pcl/octree/octree.h>        //八叉树头文件

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/filters/passthrough.h>  // 直通滤波
#include<pcl/common/common_headers.h>

#include <pcl/console/time.h>  //pcl计算时间
// pcl::console::TicToc time; time.tic();
//+程序段 +
// cout << time.toc() / 1000 << "s" << endl;

// using namespace pcl;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclTool::openPointCloudFile(const std::string& filename)
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

bool PclTool::savePointCloudFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename)
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



bool PclTool::openPcd(std::string pcdFile)
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
    return true;
}

bool PclTool::viewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud == nullptr)
    {
        std::cout << "The point cloud data is empty" << std::endl;
        return false;
    }
    pcl::console::TicToc time;
    time.tic();
    std::cout << cloud->points.size() << std::endl;
    pcl::visualization::CloudViewer viewer("Cloud Viewer: Rabbit");

    viewer.showCloud(cloud);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    cout << time.toc() / 1000 << "s" << endl;
    system("pause");
    return true;
}

bool PclTool::copyPcd(std::string fromPcd, std::string toPcd)
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

bool PclTool::copyPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& toColoud, std::vector<int> copyIndexs)
{
    if (fromCloud == nullptr)
    {
        std::cout << "The point cloud data is empty" << std::endl;
        return false;
    }

    pcl::copyPointCloud(*fromCloud, copyIndexs, *toColoud);
    return true;
}

bool PclTool::link(std::string fpcd, std::string spcd)
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
    // pcl::PolygonMesh mesh;
    // pcl::io::loadPLYFile("read.ply", mesh);
    // viewer.addPolygonMesh(mesh);

    viewer.spin();

    system("pause");
    return true;
}

std::vector<int> PclTool::kdtreeKSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const unsigned int k)
{
    // 创建KdTreeFLANN对象，并把创建的点云设置为输入,searchPoint变量作为查询点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // 设置搜索空间
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxNKNSearch(k);          // 存储查询点近邻索引
    std::vector<float> pointNKNSquaredDistance(k);  // 存储近邻点对应距离平方

    // 打印相关信息
    std::cout << "K nearest neighbor search at (" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ") with K=" << k << std::endl;

    if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)  // 执行K近邻搜索
    {
        return pointIdxNKNSearch;
        //// 打印所有近邻坐标
        // for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        //{
        //	std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x << " " << cloud->points[pointIdxNKNSearch[i]].y << " " << cloud->points[pointIdxNKNSearch[i]].z << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        // }
    }
    else
    {
        return std::vector<int>();
    }
}

std::vector<int> PclTool::kdtreeRadiusSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const float radius)
{
    // 创建KdTreeFLANN对象，并把创建的点云设置为输入,searchPoint变量作为查询点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> pointIdxRadiusSearch;          // 存储近邻索引
    std::vector<float> pointRadiusSquaredDistance;  // 存储近邻对应距离的平方

    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)  // 执行半径R内近邻搜索方法
    {
        return pointIdxRadiusSearch;
    }
    else
    {
        return std::vector<int>();
    }
}

std::vector<int> PclTool::octreeVoxelSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const float resolution)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);  // 初始化Octree
    octree.setInputCloud(cloud);        // 设置输入点云 
    octree.addPointsFromInputCloud();   // 构建octree
    std::vector<int> pointIdxVec;       // 存储体素近邻搜索结果向量
    if (octree.voxelSearch(searchPoint, pointIdxVec))  // 执行搜索
    {
        return pointIdxVec;
    }
    else
    {
        return std::vector<int>();
    }
}


std::vector<int> PclTool::octreeKSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float resolution, const pcl::PointXYZ searchPoint, const unsigned int k)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);  // 初始化Octree
    octree.setInputCloud(cloud);          // 设置输入点云
    octree.addPointsFromInputCloud();     // 构建octree


    std::vector<int> pointIdxNKNSearch;          // 结果点的索引的向量
    std::vector<float> pointNKNSquaredDistance;  // 搜索点与近邻之间的距离的平方

    if (octree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        return pointIdxNKNSearch;
    }
    else
    {
        return std::vector<int>();
    }

}


std::vector<int> PclTool::octreeRadiusSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float resolution, const pcl::PointXYZ searchPoint, const float radius)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);  // 初始化Octree
    octree.setInputCloud(cloud);         // 设置输入点云
    octree.addPointsFromInputCloud();    // 构建octree

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        return pointIdxRadiusSearch;
    }
    else
    {
        return std::vector<int>();
    }
}


std::vector<int> PclTool::octreeChangeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr beforCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr afterCloud, const float resolution)
{
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

    // 添加点云到八叉树中，构建八叉树
    octree.setInputCloud(beforCloud);  // 设置输入点云
    octree.addPointsFromInputCloud();  // 从输入点云构建八叉树
    // 添加点云到八叉树中
    octree.setInputCloud(afterCloud);
    octree.addPointsFromInputCloud();
    std::vector<int> newPointIdxVector;  // 存储新添加的索引的向量

    // 获取前一 beforCloud 对应八叉树在 afterCloud 对应在八叉树中没有的点集
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);
    return newPointIdxVector;

}


std::vector<int> PclTool::randomSampleConsensus(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const unsigned int type)
{
    std::vector<int> inliers;
    if (type == 1)
    {  
        // 平面
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    else if (type == 2)
    {  
        // 球体
        pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    else
    {
        std::cout << "type error: 1 or 2 " << std::endl;
    }

    return inliers;
}





pcl::PointCloud<pcl::PointXYZ>::Ptr PclTool::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string field_name, float Limit_low, float Limit_hig, bool is_save)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 设置滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);       // 设置输入点云
    pass.setFilterFieldName(field_name);  // 设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits(Limit_low, Limit_hig);  // 设置在过滤字段的范围
    //if (is_save)
    //{
    //    // is_save:true: 保留(Limit_low~Limit_hig)范围内的点
    //    // is_save:false: 删除(Limit_low~Limit_hig)范围内的点
    //    pass.getFilterLimitsNegative();  // 设置保留范围内还是过滤掉范围内
    //}
    pass.filter(*cloud_filtered);  // 执行滤波，保存过滤结果在cloud_filtered

    return cloud_filtered;
}



PclTool::PclTool()
{
    std::cout << "Start PclTool" << std::endl;
}
PclTool::~PclTool()
{
    std::cout << "End PclTool" << std::endl;
}