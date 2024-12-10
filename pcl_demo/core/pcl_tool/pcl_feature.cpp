
#include "pcl_feature.h"

#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>  //pfh特征估计类头文件
#include <pcl/visualization/pcl_plotter.h>  // 直方图的可视化 方法2
#include <pcl/surface/mls.h>
#include <pcl/visualization/range_image_visualizer.h>


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



void PclFeature::depthMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::RangeImage& rangeImage)
{
    // 设置深度图生成的参数

    // angularResolution
    // 含义: 这是角度分辨率, 以弧度为单位。它定义了生成范围图像时在水平和垂直方向上每个像素之间的角度间隔。例如, 1°的角度分辨率将使得每个像素代表一个1° x 1°的视场。
    // 作用 : 这个参数影响生成的范围图像的分辨率。较小的角度分辨率意味着更高的图像分辨率, 反之则意味着图像分辨率较低。
    float angularResolution = (float)(1.0f * (M_PI / 180.0f));                             // 1° in radians
    
    // 最大水平角度, 表示图像的水平视场。它通常设置为 360°, 表示完整的水平旋转视角。在弧度制下, 它等于 2 * M_PI。
    // 此参数确定了生成的范围图像在水平方向上可以覆盖的最大视场范围。通常, 它会设置为 360°(2 * M_PI), 代表完整的圆形视角。
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));                               // 360° in radians
    
    //
    // 含义: 最大垂直角度, 表示图像的垂直视场。它通常设置为 180°, 表示从上到下的完整垂直视场范围, 在弧度制下, 它等于 M_PI。
    // 作用 : 此参数确定了生成的范围图像在垂直方向上可以覆盖的最大视场范围。通常, 它会设置为 180°(M_PI), 表示从地平线到天顶的完整视角。
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));                              // 180° in radians
    
    // sensorPose
    // 含义: 传感器的位置和朝向(传感器在世界坐标系中的变换)。这是一个 4x4 的变换矩阵, 可以包括旋转和平移信息。
    // 作用: 这个参数决定了传感器(例如激光雷达或深度相机)的位置和朝向。对于不同的应用, 传感器的安装位置和方向会影响生成的范围图像的视角和局部坐标系。通常, 如果传感器位于原点并朝向某个方向, 该参数将反映这一点。
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(0.0f, 0.0f, 0.0f));  // Sensor position
    
    // coordinateFrame
    // 含义: 指定范围图像使用的坐标系。可以是以下几种之一: 
    // pcl::RangeImage::CAMERA_FRAME: 使用相机坐标系, 通常是指图像的左上角表示相机的视点, 图像中心是相机的前方。
    pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::CAMERA_FRAME;      // Camera coordinate frame
    // noiseLevel
    // 含义: 点云数据中的噪声水平, 通常是一个介于 0 和 1 之间的值。
    // 作用: 此参数用于模拟点云数据中的噪声。较高的值表示噪声较多, 较低的值则表示噪声较少。它有助于在处理时模拟更真实的环境条件, 或者在生成范围图像时进行噪声滤波。
    float noiseLevel = 0.00;    // No noise
    // 含义: 传感器的最小测量范围, 通常用来排除距离传感器非常近的点。
    // 作用 : 此参数设置了在生成范围图像时, 哪些点云数据被认为是在传感器的有效测量范围之外。如果点云中的某些点距离传感器的距离小于 minRange, 这些点将被忽略。
    float minRange = 0.0f;  // Minimum range (no clipping)
    // 含义: 范围图像边界的大小, 通常用来控制范围图像的边缘部分如何处理。
    // 作用: 这个参数设置了范围图像边缘的处理方式。如果有像素没有数据(例如, 边界处), 则该参数指定如何填充这些区域。通常为 1, 表示边界有1个像素宽度。
    int borderSize = 1;  // Border size for image

    // 调用函数生成深度图
    // 使用输入的点云和给定的参数生成深度图（RangeImage）

    // 初始化 range_image_ptr 为 pcl::RangeImage 类型
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    rangeImage = *range_image_ptr;

    // 创建 rangeImage
    rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinateFrame, noiseLevel, minRange, borderSize);

    std::cout << "深度图 " << rangeImage << "\n";

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