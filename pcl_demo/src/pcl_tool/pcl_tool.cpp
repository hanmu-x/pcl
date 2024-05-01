
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
#include <pcl/ModelCoefficients.h>        //模型系数头文件
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

    std::cout << "point size:" << cloud_normals.points.size() << std::endl;
    std::cout << "height:" << cloud_normals.height << std::endl;
    std::cout << "width:" << cloud_normals.width << std::endl;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer: Normals");

    // 将带有法线的点云可视化
    viewer.addPointCloudNormals<pcl::PointNormal>(cloud_normals.makeShared());

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
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

    pcl::SACSegmentation<pcl::PointXYZ> seg;  // 创建分割对象
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
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            // 无法估计给定数据集的平面模型。
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // 提取入口
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*temp_cloud);
        vecCloud.push_back(temp_cloud);
        std::cout << "Extract the " << i << "point cloud : " << temp_cloud->width * temp_cloud->height << " data points." << std::endl;

        // 创建筛选对象
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
    }

    return vecCloud;
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

pcl::PointCloud<pcl::PointXYZI>::Ptr PclTool::bilateralFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double standard_dev, const double halfSize)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    // 创建了一个双边滤波器对象 fbf, 指定了点云中每个点的类型pcl::PointXYZI
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(cloud);
    fbf.setSearchMethod(tree);
    fbf.setStdDev(standard_dev);  // 设置标准偏差
    fbf.setHalfSize(halfSize);    // 高斯双边滤波器窗口的一半大小
    fbf.filter(*cloud_filtered);

    return cloud_filtered;
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
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    /************************************************************************
    三种法线估计方法
     COVARIANCE_MATRIX 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
     AVERAGE_3D_GRADIENT   模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
     AVERAGE_DEPTH——CHANGE  模式只创建了一个单一的积分图，从而平局深度变化计算法线
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
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 输出具有PointNormal类型，以便存储MLS计算的法线
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init对象（第二种点类型用于法线，即使未使用）
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // 设置参数
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.1);

    // 修复
    mls.process(mls_points); 

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

















PclTool::PclTool()
{
    std::cout << "Start PclTool" << std::endl;
}
PclTool::~PclTool()
{
    std::cout << "End PclTool" << std::endl;
}