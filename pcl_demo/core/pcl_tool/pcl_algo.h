
#ifndef PCL_ALGO_H
#define PCL_ALGO_H

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>                  // 点云类型
#include <pcl/point_types.h>                  //点数据类型
#include <pcl/ModelCoefficients.h>            //模型系数头文件
#include <pcl/filters/conditional_removal.h>  // ConditionalRemoval 移除离群点

class PclAlgo
{
  public:



    /// <summary>
    /// 贪婪三角化
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static pcl::PolygonMesh projectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// <summary>
    /// 欧式聚类提取
    /// </summary>
    static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

#endif  // PCL_ALGO_H
