
#include <filesystem>
#include<iostream>
#include <pcl/io/pcd_io.h>

#include "pcl_tool/pcl_IO.h"
#include "pcl_tool/pcl_filter.h"
#include "pcl_tool/pcl_feature.h"
#include "pcl_tool/pcl_transform.h"
#include "pcl_tool/pcl_algo.h"

//#include <vld.h>
#include <pcl/visualization/range_image_visualizer.h>

int main()
{
    std::filesystem::path data_1(DEFAULT_DATA_DIR);
    data_1 += "/tuzi.pcd";

    std::filesystem::path data_2(DEFAULT_DATA_DIR);
    data_2 += "/consensus.pcd";

    std::filesystem::path data_3(DEFAULT_DATA_DIR);
    data_3 += "/table_scene_lms400.pcd";

    std::filesystem::path data_4(DEFAULT_DATA_DIR);
    data_4 += "/ism_test_cat.pcd";

    std::filesystem::path data_5(DEFAULT_DATA_DIR);
    data_5 += "/ism_train_cat.pcd";

    std::filesystem::path data_6(DEFAULT_DATA_DIR);
    data_6 += "/table_scene_mug_stereo_textured.pcd";

    std::filesystem::path mapPcd(DEFAULT_DATA_DIR);
    mapPcd += "/pointcloud_map.pcd";

    std::filesystem::path rect(DEFAULT_DATA_DIR);
    rect += "/hill.pcd";


    pcl::PointCloud<pcl::PointXYZ>::Ptr mapPcdPtr = PclIO::openPointCloudFile(mapPcd.string());
    // 绘制立方体
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapRect = PclIO::drawCube(mapPcdPtr);
    PclIO::savePointCloudFile(mapRect, rect.string());
    PclIO::viewerPcl(mapRect);

    return 0;

    // VoxelGrid滤波下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr las_cloud = PclIO::openPointCloudFile("D:/1_wangyingjie/readfile/3_Mountain/1_RawPointCloud/1.pcd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr las_cloud_fl = PclFilter::voxelGridFilter(las_cloud, 0.1, 0.1, 0.1);
    PclIO::savePointCloudFile(las_cloud_fl, rect.string());

    PclIO::viewerPcl(las_cloud_fl);

    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud = PclIO::openPointCloudFile(data_3.string());

    // 深度图
    pcl::RangeImage rangeImage;
    PclFeature::depthMap(table_cloud, rangeImage);
    // 现在你可以将range_image可视化
    pcl::visualization::RangeImageVisualizer range_image_viewer("Range Image Viewer");
    range_image_viewer.showRangeImage(rangeImage);

    while (!range_image_viewer.wasStopped())
    {
        range_image_viewer.spinOnce();
    }

    return 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_scene = PclIO::openPointCloudFile(data_6.string());
    pcl::PointCloud<pcl::PointXYZ>::Ptr talble_hull = PclTransform::ExtractConvexConcavePolygons(table_scene);
    PclIO::viewerPcl(talble_hull);
    return 0;

    /// 多项式重构的平滑和法线估计
    pcl::PointCloud<pcl::PointXYZ>::Ptr train_cat = PclIO::openPointCloudFile(data_5.string());
    pcl::PointCloud<pcl::PointNormal> normalll = PclFeature::smoothAndNormalCal(train_cat);
    PclIO::viewerPcl(normalll);

    return 0;




    // 欧式聚类
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_table = PclAlgo::euclideanClustering(table_cloud);

    for (const auto& once : vec_table)
    {
        PclIO::viewerPcl(once);
    }

    return 0;

    // 圆柱分隔
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    PclTransform::cylindricalSegmentation(table_scene, cloud_cylinder, 0, 0.1, 0.05);
    if (!cloud_cylinder->points.empty())
    {
        PclIO::viewerPcl(cloud_cylinder);
    }
    else
    {
        std::cerr << "Can't find the cylindrical component." << std::endl;
    }


    return 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tuzi = PclIO::openPointCloudFile(data_1.string());

    


    // 平面分割
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeSeg_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 填充点云
    planeSeg_cloud->width = 15;
    planeSeg_cloud->height = 1;
    planeSeg_cloud->points.resize(planeSeg_cloud->width * planeSeg_cloud->height);

    // 生成数据，采用随机数填充点云的x,y坐标，都处于z为1的平面上
    for (size_t i = 0; i < planeSeg_cloud->points.size(); ++i)
    {
        planeSeg_cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        planeSeg_cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        planeSeg_cloud->points[i].z = 1.0;
    }

    // 设置几个局外点，即重新设置几个点的z值，使其偏离z为1的平面
    planeSeg_cloud->points[0].z = 2.0;
    planeSeg_cloud->points[3].z = -2.0;
    planeSeg_cloud->points[6].z = 4.0;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    PclTransform::planeSegmentation(planeSeg_cloud, coefficients, inliers);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    // 打印出平面模型
    std::cout << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

    std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
    for (size_t i = 0; i < inliers->indices.size(); ++i)
    {
        std::cout << inliers->indices[i] << "\t" << planeSeg_cloud->points[inliers->indices[i]].x << " " << planeSeg_cloud->points[inliers->indices[i]].y << " " << planeSeg_cloud->points[inliers->indices[i]].z << std::endl;
    }
    
    return 0;

    // 三角化
    pcl::PolygonMesh tuzimesh = PclAlgo::projectionTriangulation(tuzi);
    PclIO::viewerPcl(tuzimesh);

    return 0;
    




    // 待查看----------------------------------
    //pcl::PointCloud<pcl::Normal>::Ptr normal_intergra = PclTool::integralNormalCalculation(table_cloud, 0.02f, 10.0f);

    //PclIO::viewerPcl(table_cloud, normal_intergra);
    //return 0;
    // 待查看----------------------------------


    pcl::PointCloud<pcl::PointXYZ>::Ptr cat_cloud = PclIO::openPointCloudFile(data_4.string());

    // 待查看----------------------------------
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh = PclFeature::histogramFeatures(cat_cloud, 0.05);
    return 0;
    // 待查看----------------------------------

    pcl::PointCloud<pcl::Normal>::Ptr normal = PclFeature::normalCalculation(table_cloud, 0.03);
    PclIO::viewerPcl(table_cloud, normal);

    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr concloud = PclIO::openPointCloudFile(data_2.string());

    std::vector<int> index2 = PclFilter::randomSampleConsensusALG(concloud, 0.6, 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (PclIO::copyPcd(concloud, outcloud, index2))
    {
        // 打开pcd
        PclIO::viewerPcl(outcloud);
    }

    return 0;


        // 为条件定义对象添加比较算子
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0));     // 添加在Z字段上大于0的比较算子
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.08));     // 添加在Z字段上小于0.8的比较算子
    std::vector<pcl::FieldComparison<pcl::PointXYZ>::ConstPtr> comparisons;
    comparisons.push_back(comp1);
    comparisons.push_back(comp2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr condCloud = PclFilter::conditionRemoval(table_cloud, comparisons);
    
    PclIO::viewerPcl(condCloud);

    return 0;
    
    /// RadiusOutlinerRemoval 移除离群点
    pcl::PointCloud<pcl::PointXYZ>::Ptr RORemoval_cloud = PclFilter::RORemoval(table_cloud, 0.01, 1);
    PclIO::viewerPcl(RORemoval_cloud);




    // 点云提取
    pcl::PCLPointCloud2::Ptr cloud2_table_cloud2 = PclIO::openPointCloudFile2(data_3.string());

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vecExtraction = PclTransform::cloudExtraction(cloud2_table_cloud2);
    for (const auto& ones : vecExtraction)
    {
        PclIO::viewerPcl(ones);
    }


    // 点云投影


    pcl::PointCloud<pcl::PointXYZ>::Ptr Projection_xy = PclTransform::cloudProjection(table_cloud, 0.0, 0.0, 1.0, 0.0);

    PclIO::viewerPcl(Projection_xy);


    pcl::PointCloud<pcl::PointXYZ>::Ptr Projection_yz = PclTransform::cloudProjection(table_cloud, 1.0, 0.0, 0.0, 0.0);

    PclIO::viewerPcl(Projection_yz);

    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr scrfilter_table = PclFilter::statisticalOutlierRemovalFilter(table_cloud, 50, 1.0, true);

    PclIO::viewerPcl(scrfilter_table);

    return 0;



    pcl::PCLPointCloud2::Ptr cloud2_table_fl = PclFilter::voxelGridFilter(cloud2_table_cloud2, 0.1, 0.1, 0.1);

    PclIO::viewerPcl(cloud2_table_fl);
    return 0;



    pcl::PointCloud<pcl::PointXYZ>::Ptr passthfilter = PclFilter::passThroughFilter(tuzi, "x", -0.05, 0.02, false);

    PclIO::viewerPcl(passthfilter);
    return 0;



    // 随机生成一个索引
    int randomIndex = rand() % tuzi->size();
    // 获取随机点的坐标
    pcl::PointXYZ searchPoint = tuzi->points[randomIndex];
    // 执行k紧邻搜索
    std::vector<int> index = PclFilter::octreeRadiusSearch(tuzi, 10, searchPoint, 0.12);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>);

    //if (PclIO::copyPcd(cloud_tuzi, outcloud, index))
    //{
    //    // 打开pcd
    //    PclIO::viewerPcl(outcloud);
    //}


    //std::filesystem::path data_2(DEFAULT_DATA_DIR);
    //data_2 += "/tuzi_copy.pcd";

    //PclTool::link(data_1.string(), data_2.string());

	return 0;
}












//#include "socket/socket.h"
//int main() 
//{
//
//	std::filesystem::path data_1(DEFAULT_DATA_DIR);
//    data_1 += "/tuzi.pcd";
//    std::filesystem::path data_2(DEFAULT_DATA_DIR);
//    data_2 += "/tuzi_copy.pcd";
//	// //打开一个pcd
//    //PclTool::openPcd(data_1.string());
//	//PclIO::copyPcd(data_1.string(), data_2.string());
//
//    std::string ip = "192.168.0.163"; // 服务器IP地址
//    int port = 8080; // 服务器端口号
//    //Socket so;
//    ////if (Socket::tcpClientSync(ip, port))  // 同步
//    //if (so.tcpClientAsyn(ip, port))  // 异步
//    // {
//    //    std::cout << "TCP client executed successfully" << std::endl;
//    //}
//    //else {
//    //    std::cout << "TCP client failed" << std::endl;
//    //}
//
//    if (Socket::updClientSync(ip, port))  // 同步
//    {
//        std::cout << "TCP client executed successfully" << std::endl;
//    }
//    else {
//        std::cout << "TCP client failed" << std::endl;
//    }
//
//    return 0;
//
//
//}




//#include "config.hpp"
//int main()
//{
//
//    Config config;
//#ifndef NDEBUG
//    std::string configPath = "../../../../Config/my_config.json";
//#else
//    std::string configPath = "./my_config.json";
//#endif
//    if (config.read_config(configPath))
//    {
//        std::cout << "Read config file succession " << std::endl;
//    }
//    else
//    {
//        std::cout << "ERROR : Failed to read config file " << std::endl;
//        return 1;
//    }
//
//    std::filesystem::path data_1(DEFAULT_DATA_DIR);
//    data_1 += "/geo_db/example6.db";
//
//    PclTool tc;
//
//
//    return 0;
//}
