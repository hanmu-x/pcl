
#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>   // 点云类型
#include <pcl/point_types.h>    //点数据类型

class PclTool
{
  public:
    /// <summary>
    /// 打开点云文件,可视化展示
    /// </summary>
    /// <param name="pcdFile"></param>
    /// <returns></returns>
    static bool openPcd(std::string pcdFile);

    /// <summary>
    /// 通过点云指针打开,可视化展示
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static bool openPcd(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// <summary>
    /// 赋值一个点云文件到另一个点云文件中
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

    /// <summary>
    /// 连接点云
    /// </summary>
    /// <param name="fpcd"></param>
    /// <param name="spcd"></param>
    /// <returns></returns>
    static bool link(std::string fpcd, std::string spcd);

    ////////////////// kdtree /////////////////

    /// <summary>
    /// kdtree的k近邻索引
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

    PclTool();
    ~PclTool();

  private:
};
