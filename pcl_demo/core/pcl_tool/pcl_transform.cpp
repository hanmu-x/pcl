
#include "pcl_transform.h"

#include "pcl_filter.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>  //投影滤波类头文件
#include <pcl/filters/extract_indices.h>  // 从一个点云中提取索引
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>   // 直通滤波
#include <pcl/surface/concave_hull.h>  //创建凹多边形类定义头文件

pcl::PointCloud<pcl::PointXYZ>::Ptr PclTransform::cloudProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float x, float y, float z, float c)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    // 填充 ModelCoefficients 的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1 也就是X——Y平面
    // 定义模型系数对象，并填充对应的数据
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = x;
    coefficients->values[1] = y;
    coefficients->values[2] = z;
    coefficients->values[3] = c;

    // 创建 ProjectInliers 对象，使用ModelCoefficients作为投影对象的模型参数
    pcl::ProjectInliers<pcl::PointXYZ> proj;  // 创建投影滤波对象
    proj.setModelType(pcl::SACMODEL_PLANE);   // 设置对象对应的投影模型
    proj.setInputCloud(cloud);                // 设置输入点云
    proj.setModelCoefficients(coefficients);  // 设置模型对应的系数
    proj.filter(*cloud_projected);            // 投影结果存储cloud_projected

    return cloud_projected;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PclTransform::cloudExtraction(pcl::PCLPointCloud2::Ptr cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vecCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

    // 先对点云做VoxelGrid滤波器对数据进行下采样，在这里进行下才样是为了加速处理过程
    pcl::PCLPointCloud2::Ptr cloud_filtered_blob = PclFilter::voxelGridFilter(cloud, 0.1, 0.1, 0.1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 转换为模板点云
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // 创建分割对象 (pcl::SACSegmentation进行随机采样一致性（RANSAC）平面分割)
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);      // 设置对估计模型参数进行优化处理
    seg.setModelType(pcl::SACMODEL_PLANE);  // 设置分割模型类别
    seg.setMethodType(pcl::SAC_RANSAC);     // 设置用哪个随机参数估计方法
    seg.setMaxIterations(1000);             // 设置最大迭代次数
    seg.setDistanceThreshold(0.01);         // 判断是否为模型内点的距离阀值

    // 设置ExtractIndices的实际参数
    pcl::ExtractIndices<pcl::PointXYZ> extract;  // 创建点云提取对象
    int i = 0;
    int nr_points = (int)cloud_filtered->points.size();  // 点云总数
    for (int i = 0; cloud_filtered->points.size() > 0.3 * nr_points; i++)
    {
        // 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
        seg.setInputCloud(cloud_filtered);
        // 执行分割，找到一个最佳拟合平面模型，并将内点索引存入*inliers，模型参数存入*coefficients
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            // 如果inliers->indices.size() 为0，说明没有找到合适的平面模型，此时跳出循环。
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // 提取入口 利用 extract 根据inliers提取出当前平面的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*temp_cloud);
        vecCloud.push_back(temp_cloud);
        std::cout << "Extract the " << i << "point cloud : " << temp_cloud->width * temp_cloud->height << " data points." << std::endl;

        // 创建筛选对象
        // 设置setNegative(true) 来从cloud_filtered中移除刚刚提取出的平面点云的点，保留剩下的点云数据准备下一轮分割。
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);  // 将剩下的点云数据赋回给cloud_filtered
    }

    return vecCloud;
}

bool PclTransform::planeSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr& coefficients, pcl::PointIndices::Ptr& inliers)
{
    std::cout << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

    //- pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //    - pcl::ModelCoefficients 用于存储平面模型的系数（A、B、C和D）
    //    - Model coefficients: 0 0 1 -1：平面模型的系数表示为[A, B, C, D]，其中A、B、C表示平面的法向量，D表示平面到原点的距离。在这里，系数为[0, 0, 1, -1]，表示平面的法向量在Z轴上，距离原点的距离为1，即平面方程为Z=1。
    //- pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //    - 用于存储内点的索引

    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE);  // 设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);     // 设置随机采样一致性方法类型
    seg.setDistanceThreshold(0.01);         // 设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                            // 表示点到估计模型的距离最大值，

    seg.setInputCloud(cloud);
    // 引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return false;
    }
    return true;
}

bool PclTransform::cylindricalSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cylinder, double radius_min, double radius_max, double distance_threshold)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;  // 法线估计对象
    // 过滤后的点云进行法线估计，后续的圆柱体分割需要利用点的法线信息
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);  // 设置模型类型为圆柱（SACMODEL_CYLINDER）
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);  // 选择RANSAC作为随机采样方法
    // 设置最大迭代次数，距离阈值，以及圆柱的半径范围
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(distance_threshold);
    seg.setRadiusLimits(radius_min, radius_max);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // seg.setSearchMethod(tree);

    // 执行圆柱分割。这将根据设定的参数从输入点云和法线中寻找最佳圆柱模型，并返回局内点索引和模型系数。
    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    if (inliers_cylinder->indices.empty())
    {
        printf("Cylinder segmentation failed! No inliers found.\n");
        return false;
    }
    else
    {
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);
        printf("PointCloud representing the cylindrical component: %lu data points.\n", cloud_cylinder->points.size());
        return true;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclTransform::ExtractConvexConcavePolygons(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

    // 直通滤波,仅考虑值指定范围内的数据
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);     // 设置输入点云
    pass.setFilterFieldName("z");  // 设置分割字段为z坐标
    pass.setFilterLimits(0, 1.1);  // 设置分割阀值为(0, 1.1)
    pass.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

    // 平面分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // inliers存储分割后的点云
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 设置优化系数，该参数为可选参数
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    std::cout << "PointCloud after segmentation has: " << inliers->indices.size() << " inliers." << std::endl;

    // 点云投影, 将分割出的平面点（inliers）投影到识别出的平面上，进一步过滤非平面点
    pcl::ProjectInliers<pcl::PointXYZ> proj;  // 点云投影滤波模型
    proj.setModelType(pcl::SACMODEL_PLANE);   // 设置投影模型
    proj.setIndices(inliers);
    proj.setInputCloud(cloud_filtered);
    proj.setModelCoefficients(coefficients);  // 将估计得到的平面coefficients参数设置为投影平面模型系数
    proj.filter(*cloud_projected);            // 得到投影后的点云
    std::cout << "PointCloud after projection has: " << cloud_projected->points.size() << " data points." << std::endl;

    // 提取多边形
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;  // 创建多边形提取对象
    chull.setInputCloud(cloud_projected);   // 设置输入点云为提取后点云
    chull.setAlpha(0.1);
    chull.reconstruct(*cloud_hull);  // 创建提取创建凹多边形

    std::cout << "Concave hull has: " << cloud_hull->points.size() << " data points." << std::endl;

    return cloud_hull;
}

// 计算旋转矩阵,根据旋转前后的向量得到旋转矩阵
Eigen::Matrix4f PclTransform::CalcRotatedMatrix(Eigen::Vector3f before, Eigen::Vector3f after)
{
    // 对 before 和 after 向量进行归一化处理，确保它们都是单位向量
    before.normalize();
    after.normalize();

    // 点积运算 before.dot(after) 来计算两向量之间的夹角的余弦值
    // acos 函数计算得到夹角 angle（以弧度为单位）
    float angle = acos(before.dot(after));
    // 叉乘运算 before.cross(after) 来获取旋转轴方向的向量 p_R，这个向量垂直于 before 和 after 所构成的平面
    Eigen::Vector3f p_R = before.cross(after);
    p_R.normalize();  // 再次归一化 p_R，确保它是单位向量

    Eigen::Matrix4f rotateMatrix = Eigen::Matrix4f::Identity();
    rotateMatrix(0, 0) = cos(angle) + p_R[0] * p_R[0] * (1 - cos(angle));
    rotateMatrix(0, 1) = p_R[0] * p_R[1] * (1 - cos(angle) - p_R[2] * sin(angle));
    rotateMatrix(0, 2) = p_R[1] * sin(angle) + p_R[0] * p_R[2] * (1 - cos(angle));

    rotateMatrix(1, 0) = p_R[2] * sin(angle) + p_R[0] * p_R[1] * (1 - cos(angle));
    rotateMatrix(1, 1) = cos(angle) + p_R[1] * p_R[1] * (1 - cos(angle));
    rotateMatrix(1, 2) = -p_R[0] * sin(angle) + p_R[1] * p_R[2] * (1 - cos(angle));

    rotateMatrix(2, 0) = -p_R[1] * sin(angle) + p_R[0] * p_R[2] * (1 - cos(angle));
    rotateMatrix(2, 1) = p_R[0] * sin(angle) + p_R[1] * p_R[2] * (1 - cos(angle));
    rotateMatrix(2, 2) = cos(angle) + p_R[2] * p_R[2] * (1 - cos(angle));

    return rotateMatrix;
}
