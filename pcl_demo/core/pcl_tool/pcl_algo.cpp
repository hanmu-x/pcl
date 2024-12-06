
#include "pcl_algo.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>       // VoxelGrid滤波下采样
#include <pcl/filters/extract_indices.h>  // 用于 ExtractIndices 过滤器
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>  //贪婪投影三角化算法


pcl::PolygonMesh PclAlgo::projectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PclAlgo::euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
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
        extract.filter(*cloud_plane);  // 平面
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
