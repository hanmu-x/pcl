
#ifndef PCL_FEATURE_H
#define PCL_FEATURE_H

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>  // 点云类型
#include <pcl/point_types.h>  //点数据类型

class PclFeature
{
  public:
    /// <summary>
    /// 法线估算
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="radius">半径,单位米</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::Normal>::Ptr normalCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);

    /// <summary>
    /// 指定索引的法线估算
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="radius">半径</param>
    /// <param name="indicators">指定的索引值</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::Normal>::Ptr normalCalculationFromIndicators(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, std::vector<int> indicators);

    /// <summary>
    /// 积分图法线估计
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="depth_factor">深度变化系数</param>
    /// <param name="smooth_size">法线优化时考虑的邻域的大小</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::Normal>::Ptr integralNormalCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float depth_factor, float smooth_size);

    /// <summary>
    /// 点特征直方图（PFH）描述子
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="radius"></param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PFHSignature125>::Ptr histogramFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius);

    /// <summary>
    /// 多项式重构的平滑和法线估计
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns>输出一个包含平滑后的点云数据以及相应法线信息的数据结构</returns>
    static pcl::PointCloud<pcl::PointNormal> smoothAndNormalCal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif  // PCL_FEATURE_H