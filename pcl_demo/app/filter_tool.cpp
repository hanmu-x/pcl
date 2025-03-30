#include <filesystem>
#include <iostream>
#include <json/json.h> // 引入 jsoncpp 头文件

#include <pcl/io/pcd_io.h>
#include "pcl_tool/pcl_IO.h"
#include "pcl_tool/pcl_filter.h"
#include "pcl_tool/pcl_feature.h"
#include "pcl_tool/pcl_transform.h"
#include "pcl_tool/pcl_algo.h"

// #include <vld.h>
#include <pcl/visualization/range_image_visualizer.h>

int main()
{
    // 打开 JSON 文件
    std::ifstream configFile("../../../../config/filter_config.json");
    if (!configFile.is_open())
    {
        std::cerr << "Failed to open config file!" << std::endl;
        return -1;
    }

    // 解析 JSON 文件
    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string errs;

    if (!parseFromStream(readerBuilder, configFile, &root, &errs))
    {
        std::cerr << "Failed to parse JSON: " << errs << std::endl;
        return -1;
    }

    // 提取配置参数
    std::string input_file = root["input_file"].asString();
    std::string output_file = root["output_file"].asString();

    // 直通滤波器配置
    bool passthrough_enabled = root["filters"]["passthrough_filter"]["enabled"].asBool();
    std::string passthrough_axis = root["filters"]["passthrough_filter"]["axis"].asString();
    double passthrough_min = root["filters"]["passthrough_filter"]["min_value"].asDouble();
    double passthrough_max = root["filters"]["passthrough_filter"]["max_value"].asDouble();

    // 统计离群点移除配置
    bool sor_enabled = root["filters"]["statistical_outlier_removal"]["enabled"].asBool();
    int sor_mean_k = root["filters"]["statistical_outlier_removal"]["mean_k"].asInt();
    double sor_std_dev_mul_thresh = root["filters"]["statistical_outlier_removal"]["std_dev_mul_thresh"].asDouble();

    // 半径离群点移除配置
    bool ror_enabled = root["filters"]["radius_outlier_removal"]["enabled"].asBool();
    double ror_radius_search = root["filters"]["radius_outlier_removal"]["radius_search"].asDouble();
    int ror_min_neighbors = root["filters"]["radius_outlier_removal"]["min_neighbors_in_radius"].asInt();

    // 输出配置信息
    std::cout << "Input File: " << input_file << std::endl;
    std::cout << "Output File: " << output_file << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapPcdPtrSrc = PclIO::openPointCloudFile(input_file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (passthrough_enabled)
    {
        std::cout << "Passthrough Filter Enabled on Axis: " << passthrough_axis << ", Min: " << passthrough_min << ", Max: " << passthrough_max << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_mapPcdPtr = PclFilter::passThroughFilter(mapPcdPtrSrc, passthrough_axis, passthrough_min, passthrough_max);
        outputCloud = pass_mapPcdPtr;
    }
    else
    {
        outputCloud = mapPcdPtrSrc;
    }


    if (sor_enabled)
    {
        std::cout << "Statistical Outlier Removal Enabled, Mean K: " << sor_mean_k << ", Std Dev Threshold: " << sor_std_dev_mul_thresh << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scrfilter_mapPcdPtr = PclFilter::statisticalOutlierRemovalFilter(outputCloud, 50, 1.0);
        outputCloud = scrfilter_mapPcdPtr;
    }

    if (ror_enabled)
    {
        std::cout << "Radius Outlier Removal Enabled, Radius: " << ror_radius_search << ", Min Neighbors: " << ror_min_neighbors << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ror_mapPcdPtr = PclFilter::RORemoval(outputCloud, ror_radius_search, ror_min_neighbors);
        outputCloud = ror_mapPcdPtr;
    }
    PclIO::savePointCloudFile(outputCloud, output_file);
    PclIO::viewerPcl(outputCloud);

    return 0;
}


