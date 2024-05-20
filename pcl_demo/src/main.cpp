
#include "pcl_tool/pcl_tool.h"
#include "pcl_tool/config.hpp"
#include "socket/socket.h"

#include <filesystem>
#include<iostream>

#include <pcl/io/pcd_io.h>



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


    pcl::PointCloud<pcl::PointXYZ>::Ptr table_scene = PclTool::openPointCloudFile(data_6.string());
    pcl::PointCloud<pcl::PointXYZ>::Ptr talble_hull = PclTool::ExtractConvexConcavePolygons(table_scene);
    PclTool::viewerPcl(talble_hull);
    return 0;

    /// 多项式重构的平滑和法线估计
    pcl::PointCloud<pcl::PointXYZ>::Ptr train_cat = PclTool::openPointCloudFile(data_5.string());
    pcl::PointCloud<pcl::PointNormal> normalll = PclTool::smoothAndNormalCal(train_cat);
    PclTool::viewerPcl(normalll);

    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud = PclTool::openPointCloudFile(data_3.string());


    // 欧式聚类
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_table = PclTool::euclideanClustering(table_cloud);

    for (const auto& once : vec_table)
    {
        PclTool::viewerPcl(once);
    }

    return 0;

    // 圆柱分隔
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    PclTool::cylindricalSegmentation(table_scene, cloud_cylinder, 0, 0.1, 0.05);
    if (!cloud_cylinder->points.empty())
    {
        PclTool::viewerPcl(cloud_cylinder);
    }
    else
    {
        std::cerr << "Can't find the cylindrical component." << std::endl;
    }


    return 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tuzi = PclTool::openPointCloudFile(data_1.string());

    


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

    PclTool::planeSegmentation(planeSeg_cloud, coefficients, inliers);

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
    pcl::PolygonMesh tuzimesh = PclTool::projectionTriangulation(tuzi);
    PclTool::viewerPcl(tuzimesh);

    return 0;
    




    // 待查看----------------------------------
    //pcl::PointCloud<pcl::Normal>::Ptr normal_intergra = PclTool::integralNormalCalculation(table_cloud, 0.02f, 10.0f);

    //PclTool::viewerPcl(table_cloud, normal_intergra);
    //return 0;
    // 待查看----------------------------------


    pcl::PointCloud<pcl::PointXYZ>::Ptr cat_cloud = PclTool::openPointCloudFile(data_4.string());

    // 待查看----------------------------------
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh = PclTool::histogramFeatures(cat_cloud, 0.05);
    return 0;
    // 待查看----------------------------------

    pcl::PointCloud<pcl::Normal>::Ptr normal = PclTool::PclTool::normalCalculation(table_cloud, 0.03);
    PclTool::viewerPcl(table_cloud, normal);

    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr concloud = PclTool::openPointCloudFile(data_2.string());

    std::vector<int> index2 = PclTool::randomSampleConsensusALG(concloud, 0.6, 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (PclTool::copyPcd(concloud, outcloud, index2))
    {
        // 打开pcd
        PclTool::viewerPcl(outcloud);
    }

    return 0;


        // 为条件定义对象添加比较算子
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0));     // 添加在Z字段上大于0的比较算子
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.08));     // 添加在Z字段上小于0.8的比较算子
    std::vector<pcl::FieldComparison<pcl::PointXYZ>::ConstPtr> comparisons;
    comparisons.push_back(comp1);
    comparisons.push_back(comp2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr condCloud = PclTool::conditionRemoval(table_cloud, comparisons);
    
    PclTool::viewerPcl(condCloud);

    return 0;
    
    /// RadiusOutlinerRemoval 移除离群点
    pcl::PointCloud<pcl::PointXYZ>::Ptr RORemoval_cloud = PclTool::RORemoval(table_cloud, 0.01, 1);
    PclTool::viewerPcl(RORemoval_cloud);




    // 点云提取
    pcl::PCLPointCloud2::Ptr cloud2_table_cloud2 = PclTool::openPointCloudFile2(data_3.string());

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vecExtraction = PclTool::cloudExtraction(cloud2_table_cloud2);
    for (const auto& ones : vecExtraction)
    {
        PclTool::viewerPcl(ones);
    }


    // 点云投影


    pcl::PointCloud<pcl::PointXYZ>::Ptr Projection_xy = PclTool::cloudProjection(table_cloud, 0.0, 0.0, 1.0, 0.0);

    PclTool::viewerPcl(Projection_xy);


    pcl::PointCloud<pcl::PointXYZ>::Ptr Projection_yz = PclTool::cloudProjection(table_cloud, 1.0, 0.0, 0.0, 0.0);

    PclTool::viewerPcl(Projection_yz);

    return 0;


    pcl::PointCloud<pcl::PointXYZ>::Ptr scrfilter_table = PclTool::statisticalOutlierRemovalFilter(table_cloud, 50, 1.0, true);

    PclTool::viewerPcl(scrfilter_table);

    return 0;



    pcl::PCLPointCloud2::Ptr cloud2_table_fl = PclTool::voxelGridFilter(cloud2_table_cloud2, 0.1, 0.1, 0.1);

    PclTool::viewerPcl(cloud2_table_fl);
    return 0;



    pcl::PointCloud<pcl::PointXYZ>::Ptr passthfilter = PclTool::passThroughFilter(tuzi, "x", -0.05, 0.02,  false);

    PclTool::viewerPcl(passthfilter);
    return 0;







    // 随机生成一个索引
    int randomIndex = rand() % tuzi->size();
    // 获取随机点的坐标
    pcl::PointXYZ searchPoint = tuzi->points[randomIndex];
    // 执行k紧邻搜索
    std::vector<int> index = PclTool::octreeRadiusSearch(tuzi, 10, searchPoint, 0.12);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>);

    //if (PclTool::copyPcd(cloud_tuzi, outcloud, index))
    //{
    //    // 打开pcd
    //    PclTool::viewerPcl(outcloud);
    //}




    //std::filesystem::path data_2(DEFAULT_DATA_DIR);
    //data_2 += "/tuzi_copy.pcd";

    //PclTool::link(data_1.string(), data_2.string());

	return 0;
}


















//int main() 
//{
//
//	std::filesystem::path data_1(DEFAULT_DATA_DIR);
//    data_1 += "/tuzi.pcd";
//    std::filesystem::path data_2(DEFAULT_DATA_DIR);
//    data_2 += "/tuzi_copy.pcd";
//	// //打开一个pcd
//    //PclTool::openPcd(data_1.string());
//	//PclTool::copyPcd(data_1.string(), data_2.string());
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











//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//
//
//int main(int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(5, 1));
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Fill in the CloudIn data
//    for (auto& point : *cloud_in)
//    {
//        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//    }
//
//    std::cout << "Saved " << cloud_in->size() << " data points to input:" << std::endl;
//
//    for (auto& point : *cloud_in)
//        std::cout << point << std::endl;
//
//    *cloud_out = *cloud_in;
//
//    std::cout << "size:" << cloud_out->size() << std::endl;
//    for (auto& point : *cloud_out)
//        point.x += 0.7f;
//
//    std::cout << "Transformed " << cloud_in->size() << " data points:" << std::endl;
//
//    for (auto& point : *cloud_out)
//        std::cout << point << std::endl;
//
//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(cloud_in);
//    icp.setInputTarget(cloud_out);
//
//    pcl::PointCloud<pcl::PointXYZ> Final;
//    icp.align(Final);
//
//    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//        icp.getFitnessScore() << std::endl;
//    std::cout << icp.getFinalTransformation() << std::endl;
//
//    return (0);
//}














//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);//老版的此处可能是vtkRenderingOpenGL
//VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingFreeType);
//
//int user_data;

////该回调函数，在主函数中只注册一次 ，该函数只实现对可视化对象背景颜色的设置
//void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
//{
//    viewer.setBackgroundColor(1.0, 0.5, 1.0);//设置背景颜色
//    std::cout << "我只执行一次" << std::endl;
//
//}
////该回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串
//void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
//{
//    static unsigned count = 0;
//    std::stringstream ss;
//    ss << "play the" << count++;
//    viewer.removeShape("text", 0);//移除旧的文字
//    viewer.addText(ss.str(), 200, 300, "text", 0);//添加新文字
//}
//
//int main()
//{
//    std::filesystem::path data_1(DEFAULT_DATA_DIR);
//    data_1 += "/tuzi.pcd";
//    //创建点云对象
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//    pcl::io::loadPCDFile(data_1.string().c_str(), *cloud);//读取pcd文件到点云对象
//
//    pcl::visualization::CloudViewer viewer("Cloud Viewer");//初始化可视化对象
//
//    //showCloud函数是同步的，渲染显示之前程序会一直在此等候
//    viewer.showCloud(cloud);
//
//    //下面的内容是一个模板，支持用户进行更复杂的操作。
//    //该注册函数在可视化的时候只执行一次
//    viewer.runOnVisualizationThreadOnce(viewerOneOff);//
//
//    //该注册函数在渲染输出时每帧都调用
//    viewer.runOnVisualizationThread(viewerPsycho);
//    while (!viewer.wasStopped())
//    {
//        //此处可以添加其他处理
//        user_data++;
//    }
//    return 0;
//}




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
