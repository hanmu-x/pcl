
#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>   // 点云类型
#include <pcl/point_types.h>    //点数据类型
#include <pcl/PCLPointCloud2.h>


class PclTool
{
  public:

    /// <summary>
    /// 打开点云数据文件(pcd,ply)文件
    /// </summary>
    /// <param name="file"></param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr openPointCloudFile(const std::string& filename);

    /// <summary>
    /// 打开点云文件保存到pcl::PCLPointCloud2::Ptr
    /// </summary>
    /// <param name="filename"></param>
    /// <returns></returns>
    static pcl::PCLPointCloud2::Ptr openPointCloudFile2(const std::string& filename);


    /// <summary>
    /// 保存点云到指定的文件中
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="filename"></param>
    /// <returns></returns>
    static bool savePointCloudFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename);

    /// <summary>
    /// 打开点pcd云文件,可视化展示
    /// </summary>
    /// <param name="pcdFile"></param>
    /// <returns></returns>
    static bool openPcd(std::string pcdFile);

    /// <summary>
    /// 通过pcd点云指针打开,可视化展示
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static bool viewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /// <summary>
    /// pcl::PCLPointCloud2::Ptr点云,可视化展示
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static bool PclTool::viewerPcl(pcl::PCLPointCloud2::Ptr cloud);


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

    /// <summary>
    /// 检测从beforCloud点云到afterCloud点云增加的点集
    /// </summary>
    /// <param name="beforCloud"></param>
    /// <param name="afterCloud"></param>
    /// <returns></returns>
    static std::vector<int> octreeChangeDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr beforCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr afterCloud, const float resolution);

    ////////////// 随机采样一致性算法 ///////////////
    
    static std::vector<int> randomSampleConsensus(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const unsigned int type);

    ////////////// tracking 跟踪 ///////////////

    ////////////// 深度图 ///////////////


    ////////////// filters 滤波 ///////////////

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
    /// <param name="lx">三维体素栅格的x</param>
    /// <param name="ly">三维体素栅格的y</param>
    /// <param name="lz">三维体素栅格的z</param>
    /// <returns></returns>
    static pcl::PCLPointCloud2::Ptr voxelGridFilter(pcl::PCLPointCloud2::Ptr cloud, float lx, float ly, float lz);

    /// <summary>
    /// 使用statisticalOutlierRemoval滤波器移除离群点
    /// </summary>
    /// <param name="cloud">被过滤的点云</param>
    /// <param name="meank"></param>
    /// <param name="threshold"></param>
    /// <param name="Inversion">是否对结果取反,false:删除离群点,true:保留离群点</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int meank, double threshold, bool Inversion = false);



    PclTool();
    ~PclTool();

  private:
};
