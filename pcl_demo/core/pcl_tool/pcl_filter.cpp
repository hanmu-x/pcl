
#include "pcl_filter.h"

#include <pcl/kdtree/kdtree_flann.h>  //kdtree类定义头文件
#include <pcl/octree/octree.h>        //八叉树头文件
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/passthrough.h>  // 直通滤波
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>                   // VoxelGrid滤波下采样
#include <pcl/filters/statistical_outlier_removal.h>  // statisticalOutlierRemoval滤波器移除离群点
#include <pcl/filters/radius_outlier_removal.h>       // RadiusOutlinerRemoval 移除离群点
#include <pcl/filters/impl/bilateral.hpp>             // 双边滤波

std::vector<int> PclFilter::kdtreeKSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const unsigned int k)
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

std::vector<int> PclFilter::kdtreeRadiusSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const float radius)
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

std::vector<int> PclFilter::octreeVoxelSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const float resolution)
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

std::vector<int> PclFilter::octreeKSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float resolution, const pcl::PointXYZ searchPoint, const unsigned int k)
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

std::vector<int> PclFilter::octreeRadiusSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float resolution, const pcl::PointXYZ searchPoint, const float radius)
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

std::vector<int> PclFilter::octreeChangeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr beforCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr afterCloud, const float resolution)
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

std::vector<int> PclFilter::randomSampleConsensusALG(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double threshold, const unsigned int type)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilter::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string field_name, float Limit_low, float Limit_hig, bool is_save)
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
pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilter::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float lx, float ly, float lz)
{
    // 创建 VoxelGrid 滤波器对象
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);           // 设置输入点云
    sor.setLeafSize(lx, ly, lz);  // 设置体素大小，这里是 X、Y、Z 方向上每个体素的边长（单位：米）
    // 创建一个新的 PointCloud 来保存下采样后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 执行下采样
    sor.filter(*cloud_filtered);
    return cloud_filtered;
}

pcl::PCLPointCloud2::Ptr PclFilter::voxelGridFilter(pcl::PCLPointCloud2::Ptr cloud, float lx, float ly, float lz)
{
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  // 创建滤波对象
    sor.setInputCloud(cloud);                 // 设置需要过滤的点云给滤波对象
    sor.setLeafSize(lx, ly, lz);              // 设置滤波时创建的体素体积 单位：m
    sor.filter(*cloud_filtered);              // 执行滤波处理，存储输出

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilter::statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meank, double threshold, bool inversion)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilter::RORemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int minInRadius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  // 创建滤波器

    outrem.setInputCloud(cloud);                  // 设置输入点云
    outrem.setRadiusSearch(radius);               // 设置半径为~的范围内找临近点
    outrem.setMinNeighborsInRadius(minInRadius);  // 设置查询点的邻域点集数小于~的删除

    outrem.filter(*cloud_filtered);  // 执行条件滤波   在半径为radius 在此半径内必须要有minInRadius个邻居点，此点才会保存

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclFilter::conditionRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::FieldComparison<pcl::PointXYZ>::ConstPtr> comparisons)
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

pcl::PointCloud<pcl::PointXYZI>::Ptr PclFilter::bilateralFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double halfSize, const double standard_dev)
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