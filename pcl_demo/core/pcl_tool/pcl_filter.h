
#ifndef PCL_FILTER_H
#define PCL_FILTER_H

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>                  // 点云类型
#include <pcl/filters/conditional_removal.h>  // ConditionalRemoval 移除离群点

class PclFilter
{
  public:
    /// <summary>
    /// kdtree的k近邻索引(搜索出searchPoint点最近的Kdian)
    /// </summary>
    /// <param name="cloud">需要所有的点云</param>
    /// <param name="searchPoint">需要索引的点</param>
    /// <param name="k">索引的个数</param>
    /// <returns>返回索引出点的编号数组</returns>
    static std::vector<int> kdtreeKSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const unsigned int k);

    /// <summary>
    /// kdtree的半径近邻索引
    /// </summary>
    /// <param name="cloud">需要所有的点云</param>
    /// <param name="searchPoint">需要索引的点</param>
    /// <param name="radius">索引半径</param>
    /// <returns>返回索引出点的编号数组</returns>
    static std::vector<int> kdtreeRadiusSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const float radius);

    ////////////////// octree /////////////////

    /// <summary>
    /// octree 体素近邻搜索
    /// </summary>
    /// <param name="cloud">索引点云</param>
    /// <param name="searchPoint">索引点</param>
    /// <param name="resolution">分辨率</param>
    /// <returns>返回所有点编号数组</returns>
    static std::vector<int> octreeVoxelSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ searchPoint, const float resolution);

    /// <summary>
    /// octree的k近邻索引
    /// </summary>
    /// <param name="cloud">需要所有的点云</param>
    /// <param name="resolution">分辨率</param>
    /// <param name="searchPoint">需要索引的点</param>
    /// <param name="k">索引的个数</param>
    /// <returns>返回索引出点的编号数组</returns>
    static std::vector<int> octreeKSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float resolution, const pcl::PointXYZ searchPoint, const unsigned int k);

    /// <summary>
    /// octree的半径近邻索引
    /// </summary>
    /// <param name="cloud">需要所有的点云</param>
    /// <param name="searchPoint">需要索引的点</param>
    /// <param name="radius">索引半径</param>
    /// <returns>返回索引出点的编号数组</returns>
    static std::vector<int> octreeRadiusSearch(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float resolution, const pcl::PointXYZ searchPoint, const float radius);

    /// <summary>
    /// 检测从beforCloud点云到afterCloud点云增加的点集
    /// </summary>
    /// <param name="beforCloud"></param>
    /// <param name="afterCloud"></param>
    /// <returns></returns>
    static std::vector<int> octreeChangeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr beforCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr afterCloud, const float resolution);

    /////////////// 随机采样一致性算法 ///////////////

    /// <summary>
    /// 随机采样一致性算法
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="threshold">阈值</param>
    /// <param name="type">1:平面算法,2:球体</param>
    /// <returns>返回点的索引</returns>
    static std::vector<int> randomSampleConsensusALG(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double threshold, const unsigned int type);

    /////////////// filters 滤波 ///////////////

    /// <summary>
    /// 直通滤波
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="field_name"></param>
    /// <param name="Limit_low"></param>
    /// <param name="Limit_hig"></param>
    /// <param name="is_save"></param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string field_name, float Limit_low, float Limit_hig, bool is_save = true);

    /// <summary>
    /// VoxelGrid滤波下采样
    /// </summary>
    /// <param name="cloud">需要滤波的点云</param>
    /// <param name="lx">X、Y、Z 方向上每个体素的边长（单位：米）</param>
    /// <param name="ly">三维体素栅格的y</param>
    /// <param name="lz">三维体素栅格的z</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float lx, float ly, float lz);
    static pcl::PCLPointCloud2::Ptr voxelGridFilter(pcl::PCLPointCloud2::Ptr cloud, float lx, float ly, float lz);


    /// <summary>
    /// statisticalOutlierRemoval(统计离群点移除)移除离群点
    /// 适用于点云较为密集且分布均匀的情况
    /// </summary>
    /// <param name="cloud">被过滤的点云</param>
    /// <param name="meank">指定每个点的邻居数，用于计算平均距离</param>
    /// <param name="threshold">设置标准差倍数作为离群点判定的阈值。例如，如果某点的平均距离超过全局平均值加上 std_mul 倍的标准差，则认为它是离群点</param>
    /// <param name="Inversion">是否对结果取反,false:删除离群点,true:保留离群点</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meank, double threshold, bool Inversion = false);

    /// <summary>
    /// RadiusOutlinerRemoval(半径离群点移除)移除离群点
    /// 该滤波器基于局部密度的原理移除离群点,对于稀疏点云或非均匀分布的点云效果较好。
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="radius">设置半径的范围内找临近点</param>
    /// <param name="minInRadius">设置查询点的邻域点集数小于minInRadius的删除</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr RORemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int minInRadius);

    /// <summary>
    /// ConditionalRemoval 移除离群点
    /// 可以一次删除满足对输入的点云设定的一个或多个条件指标的所有的数据
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="comparisons">筛选条件</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr conditionRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::FieldComparison<pcl::PointXYZ>::ConstPtr> comparisons);

    /// <summary>
    /// 双边滤波
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="halfSize">高斯双边滤波器窗口的一半大小</param>
    /// <param name="standard_dev">设置标准偏差</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZI>::Ptr bilateralFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double halfSize, const double standard_dev);
};

#endif  // PCL_FILTER_H