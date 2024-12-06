
#include "pcl_feature.h"

#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>  //pfh特征估计类头文件
#include <pcl/visualization/pcl_plotter.h>  // 直方图的可视化 方法2
#include <pcl/surface/mls.h>

pcl::PointCloud<pcl::Normal>::Ptr PclFeature::normalCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
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

pcl::PointCloud<pcl::Normal>::Ptr PclFeature::normalCalculationFromIndicators(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius, std::vector<int> indicators)
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

pcl::PointCloud<pcl::Normal>::Ptr PclFeature::integralNormalCalculation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float depth_factor, float smooth_size)
{
    // 创建法线估计向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // 这个类利用积分图技术高效地计算点云中各点的法线。
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    /************************************************************************
    三种法线估计方法
     COVARIANCE_MATRIX 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
     AVERAGE_3D_GRADIENT   模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
     AVERAGE_DEPTH_CHANGE  模式只创建了一个单一的积分图，从而平局深度变化计算法线
    *****************************************************************************/
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);  // 设置法线估计的方式AVERAGE_3D_GRADIENT

    ne.setMaxDepthChangeFactor(depth_factor);  // 设置深度变化系数
    ne.setNormalSmoothingSize(smooth_size);    // 设置法线优化时考虑的邻域的大小
    ne.setInputCloud(cloud);                   // 输入的点云
    ne.compute(*normals);                      // 计算法线

    return normals;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr PclFeature::histogramFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius)
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

pcl::PointCloud<pcl::PointNormal> PclFeature::smoothAndNormalCal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 创建一个 KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // 输出具有PointNormal类型，以便存储MLS计算的法线
    // 用于存储经过平滑处理后的点云数据以及每个点的法线信息。pcl::PointNormal类型包含位置信息（XYZ坐标）和法线信息（NXNYNZ）。
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // 这是PCL中用于执行移动最小二乘（Moving Least Squares, MLS）平滑和法线估计的关键类
    // 该算法基于每个点的邻域信息，通过最小二乘拟合的方式生成一个新的、更加平滑的表面，并同时计算每个点的法线
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);  // 开启法线计算功能。

    // 设置参数
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);  // 设置多项式拟合的阶数为2，这是一个平滑度的控制参数，阶数越高表示平滑程度越高。
    mls.setSearchMethod(tree);  // KD-Tree作为近邻搜索
    mls.setSearchRadius(2);     // 设定搜索邻域的半径长度，这个值决定了参与平滑和法线计算的邻域大小。

    mls.process(mls_points);  // 执行平滑处理和法线计算

    return mls_points;
}