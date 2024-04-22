
#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>  // 点云类型
#include <pcl/point_types.h>  //点数据类型
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/conditional_removal.h>  // ConditionalRemoval 移除离群点

class PclTool
{
  public:
    /// <summary>
    /// 通过pcd点云指针打开,可视化展示
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static bool viewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals = nullptr);

    /// <summary>
    /// pcl::PCLPointCloud2::Ptr点云,可视化展示
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static bool viewerPcl(pcl::PCLPointCloud2::Ptr cloud);

    static bool viewerPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

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

    ///// <summary>
    ///// 连接点云
    ///// </summary>
    ///// <param name="fpcd"></param>
    ///// <param name="spcd"></param>
    ///// <returns></returns>
    // static bool link(std::string fpcd, std::string spcd);

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

    /// <summary>
    /// 随机采样一致性算法
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="threshold">阈值</param>
    /// <param name="type">1:平面算法,2:球体</param>
    /// <returns>返回点的索引</returns>
    static std::vector<int> randomSampleConsensusALG(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double threshold, const unsigned int type);

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
    /// 点云的提取
    /// </summary>
    /// <param name="cloud"></param>
    /// <returns></returns>
    static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudExtraction(pcl::PCLPointCloud2::Ptr cloud);

    /// <summary>
    /// RadiusOutlinerRemoval 移除离群点
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="radius">设置半径的范围内找临近点</param>
    /// <param name="minInRadius">设置查询点的邻域点集数小于minInRadius的删除</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr RORemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, int minInRadius);

    /// <summary>
    /// ConditionalRemoval 移除离群点
    /// </summary>
    /// <param name="cloud">点云</param>
    /// <param name="comparisons">筛选条件</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZ>::Ptr conditionRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::FieldComparison<pcl::PointXYZ>::ConstPtr> comparisons);

    /// <summary>
    /// 双边滤波
    /// </summary>
    /// <param name="cloud"></param>
    /// <param name="standard_dev">设置标准偏差</param>
    /// <param name="halfSize">高斯双边滤波器窗口的一半大小</param>
    /// <returns></returns>
    static pcl::PointCloud<pcl::PointXYZI>::Ptr bilateralFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double standard_dev, const double halfSize);

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


    ////////////// tracking 跟踪 ///////////////

    ////////////// 深度图 ///////////////

    PclTool();
    ~PclTool();

  private:
};
