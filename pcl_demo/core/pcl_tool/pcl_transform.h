
#ifndef PCL_TRANSFORM_H
#define PCL_TRANSFORM_H

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>  // 点云类型
#include <pcl/point_types.h>  //点数据类型
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>            //模型系数头文件
#include <pcl/filters/conditional_removal.h>  // ConditionalRemoval 移除离群点

class PclTransform
{
  public:
    /// <summary>
    /// 参数化模型投影点云
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="x">投影平面x面的系数</param>
    /// <param name="y"></param>
    /// <param name="z"></param>
    /// <param name="c"></param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float x, float y, float z, float c);

    /// <summary>
    /// 点云的平面提取(提取多个平面)
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudExtraction(pcl::PCLPointCloud2::Ptr cloud);

    /// <summary>
    /// 平面分割
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="coefficients">存储平面模型的系数（A、B、C和D）,
    /// [0, 0, 1, -1]，表示平面的法向量在Z轴上，距离原点的距离为1</param>
    /// <param name="inliers">存储内点的索引</param>
    /// <returns></returns>
    static bool planeSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers);

    /// <summary>
    /// 圆柱体模型的分割
    /// </summary>
    /// <param name="cloud_filtered">输入点云</param>
    /// <param name="cloud_cylinder">：输出参数，用于存储分割出来的圆柱形结构点云</param>
    /// <param name="radius_min">圆柱模型的最小半径</param>
    /// <param name="radius_max">圆柱模型的最大半径</param>
    /// <param name="distance_threshold">点到圆柱模型的最大允许距离，作为局内点的判断标准</param>
    /// <returns></returns>
    static bool cylindricalSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cylinder, double radius_min, double radius_max, double distance_threshold);

    /// <summary>
    /// 平面模型上提取凸（凹）多边形
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractConvexConcavePolygons(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    ////////////// 旋转矩阵 ///////////////

    Eigen::Matrix4f CalcRotatedMatrix(Eigen::Vector3f before, Eigen::Vector3f after);
};

#endif  // PCL_TRANSFORM_H