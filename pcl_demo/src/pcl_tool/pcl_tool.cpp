
#include "pcl_tool.h"
#include <filesystem>

#include <pcl/io/pcd_io.h>                   // 读取和写入PCD
#include <pcl/io/ply_io.h>                   // 读取和写入PLY
#include <pcl/visualization/cloud_viewer.h>  // 可视化点云数据的CloudViewer类
#include <pcl/kdtree/kdtree_flann.h>  //kdtree类定义头文件
#include <pcl/octree/octree.h>        //八叉树头文件
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/passthrough.h>  // 直通滤波
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>  // VoxelGrid滤波下采样
#include <pcl/filters/statistical_outlier_removal.h>  // statisticalOutlierRemoval滤波器移除离群点
//#include <pcl/ModelCoefficients.h>        //模型系数头文件
#include <pcl/filters/project_inliers.h>  //投影滤波类头文件
#include <pcl/filters/extract_indices.h>  // 从一个点云中提取索引
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>  // RadiusOutlinerRemoval 移除离群点
#include <pcl/filters/impl/bilateral.hpp>        // 双边滤波
// #include <pcl/filters/conditional_removal.h> // ConditionalRemoval 移除离群点
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>  //pfh特征估计类头文件
#include <pcl/visualization/pcl_plotter.h> // 直方图的可视化 方法2
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>                 //创建凹多边形类定义头文件
#include <pcl/surface/gp3.h>           //贪婪投影三角化算法
#include <pcl/segmentation/extract_clusters.h>



#include <pcl/console/time.h>  //pcl计算时间
// pcl::console::TicToc time; time.tic();
//+程序段 +
// cout << time.toc() / 1000 << "s" << endl;

// using namespace pcl;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色
}

bool PclTool::viewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
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

    //viewer.showCloud(cloud);
    //viewer.runOnVisualizationThreadOnce(viewerOneOff);
    //system("pause");

    std::cout << "End show " << std::endl;
    return true;
}


bool PclTool::viewerPcl(pcl::PCLPointCloud2::Ptr cloud)
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

bool PclTool::viewerPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
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


bool PclTool::viewerPcl(pcl::PointCloud<pcl::PointNormal> cloud_normals)
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

    //viewer.addPointCloud(cloud_normals.makeShared(), "cloud_normals");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_normals");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_normals");
    viewer.addPointCloudNormals<pcl::PointNormal>(cloud_normals.makeShared(), 1, 0.05, "cloud_normals_normals");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    std::cout << "End show " << std::endl;
    return true;
}

//bool PclTool::viewerPcl(pcl::PointCloud<pcl::PointNormal> cloud_normals)
//{
//    if (cloud_normals.empty())
//    {
//        std::cout << "The point cloud data is empty" << std::endl;
//        return false;
//    }
//
//    std::cout << "point size:" << cloud_normals.points.size() << std::endl;
//    std::cout << "height:" << cloud_normals.height << std::endl;
//    std::cout << "width:" << cloud_normals.width << std::endl;
//
//    pcl::visualization::PCLVisualizer viewer("Cloud Viewer: Normals");
//
//    // 将带有法线的点云可视化
//    viewer.addPointCloudNormals<pcl::PointNormal>(cloud_normals.makeShared());
//
//    while (!viewer.wasStopped())
//    {
//        viewer.spinOnce();
//    }
//
//    std::cout << "End show " << std::endl;
//    return true;
//}


bool PclTool::viewerPcl(pcl::PolygonMesh& triangles)
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

pcl::PCLPointCloud2::Ptr PclTool::openPointCloudFile2(const std::string& filename)
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
    std::cout << "End show " << std::endl;
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

// bool PclTool::link(std::string fpcd, std::string spcd)
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
    octree.setInputCloud(cloud);                                            // 设置输入点云
    octree.addPointsFromInputCloud();                                       // 构建octree
    std::vector<int> pointIdxVec;                                           // 存储体素近邻搜索结果向量
    if (octree.voxelSearch(searchPoint, pointIdxVec))                       // 执行搜索
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
    octree.setInputCloud(cloud);                                            // 设置输入点云
    octree.addPointsFromInputCloud();                                       // 构建octree

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
    octree.setInputCloud(cloud);                                            // 设置输入点云
    octree.addPointsFromInputCloud();                                       // 构建octree

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

std::vector<int> PclTool::randomSampleConsensusALG(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double threshold, const unsigned int type)
{
    std::vector<int> inliers;
    if (type == 1)
    {
        // 平面
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(threshold);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    else if (type == 2)
    {
        // 球体
        pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
        ransac.setDistanceThreshold(threshold);
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
    pass.setInputCloud(cloud);                   // 设置输入点云
    pass.setFilterFieldName(field_name);         // 设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits(Limit_low, Limit_hig);  // 设置在过滤字段的范围
    // if (is_save)
    //{
    //     // is_save:true: 保留(Limit_low~Limit_hig)范围内的点
    //     // is_save:false: 删除(Limit_low~Limit_hig)范围内的点
    //     pass.getFilterLimitsNegative();  // 设置保留范围内还是过滤掉范围内
    // }
    pass.filter(*cloud_filtered);  // 执行滤波，保存过滤结果在cloud_filtered

    return cloud_filtered;
}

pcl::PCLPointCloud2::Ptr PclTool::voxelGridFilter(pcl::PCLPointCloud2::Ptr cloud, float lx, float ly, float lz)
{
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  // 创建滤波对象
    sor.setInputCloud(cloud);                 // 设置需要过滤的点云给滤波对象
    sor.setLeafSize(lx, ly, lz);              // 设置滤波时创建的体素体积 单位：m
    sor.filter(*cloud_filtered);              // 执行滤波处理，存储输出

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclTool::statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meank, double threshold, bool inversion)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;  // 创建滤波器对象

    sor.setInputCloud(cloud);           // 设置待滤波的点云
    sor.setMeanK(meank);                // 设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(threshold);  // 设置判断是否为离群点的阀值,较大的阈值将导致更多的点被判定为离群点。
    sor.setNegative(inversion);         // 是否对结果取反,false:删除离群点,true:保留离群点
    sor.filter(*cloud_filtered);        // 存储

    return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PclTool::RORemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int minInRadius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  // 创建滤波器

    outrem.setInputCloud(cloud);                  // 设置输入点云
    outrem.setRadiusSearch(radius);               // 设置半径为~的范围内找临近点
    outrem.setMinNeighborsInRadius(minInRadius);  // 设置查询点的邻域点集数小于~的删除

    outrem.filter(*cloud_filtered);  // 执行条件滤波   在半径为radius 在此半径内必须要有minInRadius个邻居点，此点才会保存

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclTool::conditionRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::FieldComparison<pcl::PointXYZ>::ConstPtr> comparisons)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建条件限定的下的滤波器
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());  // 创建条件定义对象

    // 添加在Z字段上大于0的比较算子
    // GT greater than
    // EQ equal
    // LT less than
    // GE greater than or equal
    // LE less than  or equal 小于等于

    //// 为条件定义对象添加比较算子
    // pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0));     // 添加在Z字段上大于0的比较算子
    // pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8));     // 添加在Z字段上小于0.8的比较算子
    // range_cond->addComparison(comp1);
    // range_cond->addComparison(comp2);

    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));  // 添加在Z字段上大于0的比较算子

    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));  // 添加在Z字段上小于0.8的比较算子

    for (const auto& once : comparisons)
    {
        range_cond->addComparison(once);
    }

    // 创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);    // 输入点云
    condrem.setKeepOrganized(true);  // 设置保持点云的结构
    // 设置是否保留滤波后删除的点，以保持点云的有序性，通过setuserFilterValue设置的值填充点云；或从点云中删除滤波后的点，从而改变其组织结构
    // 如果设置为true且不设置setUserFilterValue的值，则用nan填充点云
    // https://blog.csdn.net/qq_37124765/article/details/82262863

    // 执行滤波
    condrem.filter(*cloud_filtered);

    return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr PclTool::bilateralFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double halfSize ,const double standard_dev)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    // 创建了一个双边滤波器对象 fbf, 指定了点云中每个点的类型pcl::PointXYZI
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(cloud);
    fbf.setSearchMethod(tree);
    fbf.setHalfSize(halfSize);    // 空间域参数
    fbf.setStdDev(standard_dev);  // 值域参数
    fbf.filter(*cloud_filtered);

    return cloud_filtered;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PclTool::cloudProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float x, float y, float z, float c)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    // 填充 ModelCoefficients 的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1 也就是X——Y平面
    // 定义模型系数对象，并填充对应的数据
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = x;
    coefficients->values[1] = y;
    coefficients->values[2] = z;
    coefficients->values[3] = c;

    // 创建 ProjectInliers 对象，使用ModelCoefficients作为投影对象的模型参数
    pcl::ProjectInliers<pcl::PointXYZ> proj;  // 创建投影滤波对象
    proj.setModelType(pcl::SACMODEL_PLANE);   // 设置对象对应的投影模型
    proj.setInputCloud(cloud);                // 设置输入点云
    proj.setModelCoefficients(coefficients);  // 设置模型对应的系数
    proj.filter(*cloud_projected);            // 投影结果存储cloud_projected

    return cloud_projected;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PclTool::cloudExtraction(pcl::PCLPointCloud2::Ptr cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vecCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

    // 先对点云做VoxelGrid滤波器对数据进行下采样，在这里进行下才样是为了加速处理过程
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob = voxelGridFilter(cloud, 0.1, 0.1, 0.1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 转换为模板点云
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // 创建分割对象 (pcl::SACSegmentation进行随机采样一致性（RANSAC）平面分割)
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);        // 设置对估计模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_PLANE);    // 设置分割模型类别
    seg.setMethodType(pcl::SAC_RANSAC);       // 设置用哪个随机参数估计方法
    seg.setMaxIterations(1000);               // 设置最大迭代次数
    seg.setDistanceThreshold(0.01);           // 判断是否为模型内点的距离阀值

    // 设置ExtractIndices的实际参数
    pcl::ExtractIndices<pcl::PointXYZ> extract;  // 创建点云提取对象
    int i = 0;
    int nr_points = (int)cloud_filtered->points.size();  // 点云总数
    for (int i = 0; cloud_filtered->points.size() > 0.3 * nr_points; i++)
    {
        // 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
        seg.setInputCloud(cloud_filtered);
        // 执行分割，找到一个最佳拟合平面模型，并将内点索引存入*inliers，模型参数存入*coefficients
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            // 如果inliers->indices.size() 为0，说明没有找到合适的平面模型，此时跳出循环。
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // 提取入口 利用 extract 根据inliers提取出当前平面的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*temp_cloud);
        vecCloud.push_back(temp_cloud);
        std::cout << "Extract the " << i << "point cloud : " << temp_cloud->width * temp_cloud->height << " data points." << std::endl;

        // 创建筛选对象
        // 设置setNegative(true) 来从cloud_filtered中移除刚刚提取出的平面点云的点，保留剩下的点云数据准备下一轮分割。
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);  // 将剩下的点云数据赋回给cloud_filtered
    }

    return vecCloud;
}


pcl::PointCloud<pcl::Normal>::Ptr PclTool::normalCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
{
    // 创建法线估计估计向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    // 创建一个空的KdTree对象，并把它传递给法线估计向量
    // 基于给出的输入数据集，KdTree将被建立
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // 存储输出数据
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // 使用半径在查询点周围3厘米范围内的所有临近元素
    ne.setRadiusSearch(radius);  // 单位:米
    // 计算特征值
    ne.compute(*cloud_normals);

    return cloud_normals;
}

pcl::PointCloud<pcl::Normal>::Ptr PclTool::normalCalculationFromIndicators(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, std::vector<int> indicators)
{
    // //创建NormalEstimation类，并将输入数据集传递给它
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::shared_ptr<std::vector<int>> indicesptr(new std::vector<int>(indicators));
    ne.setIndices(indicesptr);

    // 创建一个空的kdtree表示形式，并将其传递给normal estimation 对象
    // 它的内容将根据给定的输入数据集填充到对象内部（因为未提供其他搜索表面）。
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // 在半径3cm的范围内近邻
    ne.setRadiusSearch(radius);
    // 计算特征
    ne.compute(*cloud_normals);

    return cloud_normals;
}

pcl::PointCloud<pcl::Normal>::Ptr PclTool::integralNormalCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float depth_factor, float smooth_size)
{
    // 创建法线估计向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // 这个类利用积分图技术高效地计算点云中各点的法线。
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    /************************************************************************
    三种法线估计方法
     COVARIANCE_MATRIX 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
     AVERAGE_3D_GRADIENT   模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
     AVERAGE_DEPTH_CHANGE  模式只创建了一个单一的积分图，从而平局深度变化计算法线
    *****************************************************************************/
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);  // 设置法线估计的方式AVERAGE_3D_GRADIENT

    ne.setMaxDepthChangeFactor(depth_factor);   // 设置深度变化系数
    ne.setNormalSmoothingSize(smooth_size);     // 设置法线优化时考虑的邻域的大小
    ne.setInputCloud(cloud);                    // 输入的点云
    ne.compute(*normals);                       // 计算法线

    return normals;
}



pcl::PointCloud<pcl::PFHSignature125>::Ptr PclTool::histogramFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

    // 打开点云文件估计法线等
    // 创建PFH估计对象pfh，并将输入点云数据集cloud和法线normals传递给它
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);

     // 如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);
    // 创建一个空的kd树表示法，并把它传递给PFH估计对象。
    // 基于已给的输入数据集，建立kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // 输出数据集
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

    // 使用半径在5厘米范围内的所有邻元素。
    // 注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
    pfh.setRadiusSearch(radius);  // 计算pfh特征值

    // 计算pfh特征值
    pfh.compute(*pfhs);

    // 临时添加测试
    // ========直方图可视化=============================
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*pfhs, 300);  // 设置的很坐标长度，该值越大，则显示的越细致
    plotter.plot();

    return pfhs;
}


pcl::PointCloud<pcl::PointNormal> PclTool::smoothAndNormalCal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 创建一个 KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 输出具有PointNormal类型，以便存储MLS计算的法线
    // 用于存储经过平滑处理后的点云数据以及每个点的法线信息。pcl::PointNormal类型包含位置信息（XYZ坐标）和法线信息（NXNYNZ）。
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // 这是PCL中用于执行移动最小二乘（Moving Least Squares, MLS）平滑和法线估计的关键类
    // 该算法基于每个点的邻域信息，通过最小二乘拟合的方式生成一个新的、更加平滑的表面，并同时计算每个点的法线
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);    // 开启法线计算功能。

    // 设置参数
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);  // 设置多项式拟合的阶数为2，这是一个平滑度的控制参数，阶数越高表示平滑程度越高。
    mls.setSearchMethod(tree);  // KD-Tree作为近邻搜索
    mls.setSearchRadius(2);  // 设定搜索邻域的半径长度，这个值决定了参与平滑和法线计算的邻域大小。

    mls.process(mls_points);  // 执行平滑处理和法线计算

    return mls_points;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PclTool::ExtractConvexConcavePolygons(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    // 先做一下直通滤波
      // 建立过滤器消除杂散的NaN
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);     // 设置输入点云
    pass.setFilterFieldName("z");  // 设置分割字段为z坐标
    pass.setFilterLimits(0, 1.1);  // 设置分割阀值为(0, 1.1)
    pass.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;


      // 分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // inliers存储分割后的点云
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 设置优化系数，该参数为可选参数
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    std::cout << "PointCloud after segmentation has: " << inliers->indices.size() << " inliers." << std::endl;

    // Project the model inliers点云投影滤波模型
    pcl::ProjectInliers<pcl::PointXYZ> proj;  // 点云投影滤波模型
    proj.setModelType(pcl::SACMODEL_PLANE);   // 设置投影模型
    proj.setIndices(inliers);
    proj.setInputCloud(cloud_filtered);
    proj.setModelCoefficients(coefficients);  // 将估计得到的平面coefficients参数设置为投影平面模型系数
    proj.filter(*cloud_projected);            // 得到投影后的点云
    std::cout << "PointCloud after projection has: " << cloud_projected->points.size() << " data points." << std::endl;

    // 存储提取多边形上的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;  // 创建多边形提取对象
    chull.setInputCloud(cloud_projected);   // 设置输入点云为提取后点云
    chull.setAlpha(0.1);
    chull.reconstruct(*cloud_hull);  // 创建提取创建凹多边形

    std::cout << "Concave hull has: " << cloud_hull->points.size() << " data points." << std::endl;
    
    return cloud_hull;
}



pcl::PolygonMesh PclTool::projectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 正态估计
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;                                   // 法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);           // 存储估计的法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  // 定义kd树指针
    tree->setInputCloud(cloud);                                                            // 用cloud构建tree对象
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);  // 估计法线存储到其中

    // 法线不应包含点法线+曲面曲率

      // //连接XYZ字段和法线字段*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);  // 连接字段
    //* cloud_with_normals = cloud + normals

    // 定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);  // 点云构建搜索树

      // 初始化对象
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;  // 定义三角化对象
    pcl::PolygonMesh triangles;                                // 存储最终三角化的网络模型

    // 设置连接点之间的最大距离（最大边长）
    gp3.setSearchRadius(0.025);  // 设置连接点之间的最大距离，（即是三角形最大边长）

      // 设置各参数值
    gp3.setMu(2.5);                        // 设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
    gp3.setMaximumNearestNeighbors(100);   // 设置样本点可搜索的邻域个数
    gp3.setMaximumSurfaceAngle(M_PI / 4);  // 设置某点法线方向偏离样本点法线的最大角度45
    gp3.setMinimumAngle(M_PI / 18);        // 设置三角化后得到的三角形内角的最小的角度为10
    gp3.setMaximumAngle(2 * M_PI / 3);     // 设置三角化后得到的三角形内角的最大角度为120
    gp3.setNormalConsistency(false);       // 设置该参数保证法线朝向一致

    // Get result
    gp3.setInputCloud(cloud_with_normals);  // 设置输入点云为有向点云
    gp3.setSearchMethod(tree2);             // 设置搜索方式
    gp3.reconstruct(triangles);             // 重建提取三角化

    // 附加顶点信息
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    return triangles;
}


bool PclTool::planeSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers)
{
    std::cout << "Point cloud data: " << cloud->points.size() << " points" << std::endl;


    // 创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // pcl::ModelCoefficients用于存储平面模型的系数（A、B、C和D）。
    // Model coefficients: 0 0 1 -1：平面模型的系数表示为[A, B, C, D]，其中A、B、C表示平面的法向量，D表示平面到原点的距离。在这里，系数为[0, 0, 1, -1]，表示平面的法向量在Z轴上，距离原点的距离为1，即平面方程为Z=1。
    // 而pcl::PointIndices用于存储内点的索引


    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE);  // 设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);     // 设置随机采样一致性方法类型
    seg.setDistanceThreshold(0.01);         // 设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                     // 表示点到估计模型的距离最大值，

    seg.setInputCloud(cloud);
    // 引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return false;
    }
    return true;
}


bool PclTool::cylindricalSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cylinder,
                     double radius_min,
                     double radius_max,
                     double distance_threshold)
{
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  // 法线估计对象

    // 过滤后的点云进行法线估计，为后续进行基于法线的分割准备数据
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(distance_threshold);
    seg.setRadiusLimits(radius_min, radius_max);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    //seg.setSearchMethod(tree);

    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    if (inliers_cylinder->indices.empty())
    {
        printf("Cylinder segmentation failed! No inliers found.\n");
        return false;
    }
    else
    {
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);
        printf("PointCloud representing the cylindrical component: %lu data points.\n", cloud_cylinder->points.size());
        return true;
    }
}




std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PclTool::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clustered_clouds;

    // 下采样
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);

       // 创建平面模型分割的对象并设置参数
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);  // 分割模型
    seg.setMethodType(pcl::SAC_RANSAC);     // 随机参数估计方法
    seg.setMaxIterations(100);              // 最大的迭代的次数
    seg.setDistanceThreshold(0.02);         // 设置阀值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points)  // 滤波停止条件
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);  // 输入
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);  // [平面
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        //  // 移去平面局内点，提取剩余点云
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }


    // 创建KdTree对象用于欧式聚类的搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  // 欧式聚类对象
    ec.setClusterTolerance(0.02);                       // 设置聚类容差为2cm
    ec.setMinClusterSize(100);                          // 设置一个聚类的最小点数为100
    ec.setMaxClusterSize(25000);                        // 设置一个聚类的最大点数为25000
    ec.setSearchMethod(tree);                           // 设置搜索方法
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);  // 从点云中提取聚类

    // 迭代聚类索引并创建每个聚类的点云
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clustered_clouds.push_back(cloud_cluster);
    }

    return clustered_clouds;
}





PclTool::PclTool()
{
    std::cout << "Start PclTool" << std::endl;
}
PclTool::~PclTool()
{
    std::cout << "End PclTool" << std::endl;
}