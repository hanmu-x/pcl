<!--
 * @Author: yingjie_wang 2778809626@qq.com
 * @Date: 2024-05-10 18:53:18
 * @LastEditors: yingjie_wang 2778809626@qq.com
 * @LastEditTime: 2025-02-11 14:04:38
 * @FilePath: \OperationsAndMaintenanced:\1_wangyingjie\learn\github_project\3_pcl\pcl\pcl_demo\readme.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->

# PLC库学习

pcl版本: 1.13.0

vcpkg 版本
vcpkg package management program version 2023-01-24-8a88d63f241d391772fbde69af9cab96c3c64c75

vcpkg intall jsoncpp  pcl  pcl[visualization]  vtk  liblas



### pcl_IO.h (点云输入输出)

- 点云文件可视化
- 点云文件保存
- 点云文件拷贝
- 在点云文件中绘制立方体


### pcl_filter.h (索引/滤波)

- kdtree的k近邻索引
- kdtree的半径近邻索引
- octree 体素近邻搜索
- octree 的k近邻索引
- octree的半径近邻索引
- 检测从beforCloud点云到afterCloud点云增加的点集
- 随机采样一致性算法

- 直通滤波
- VoxelGrid滤波下采样
- statisticalOutlierRemoval滤波器移除离群点
- RadiusOutlinerRemoval 移除离群点
- ConditionalRemoval 移除离群点
- 双边滤波


### pcl_feature.h (特征值)

- 法线估算
- 指定索引的法线估算
- 积分图法线估计
- 深度图
- 点特征直方图（PFH）描述子
- 多项式重构的平滑和法线估计


### pcl_transform.h (转换)

- 参数化模型投影点云
- 点云的平面提取(提取多个平面)
- 平面分割
- 圆柱体模型的分割
- 平面模型上提取凸（凹）多边形


### pcl_algo.h (算法)

- 贪婪三角化
- 欧式聚类提取
- 点云重构算法:
  - 



Raw Point Cloud Reconstructed
重建的原始点云

Raw Point Cloud Reconstructed(edges hidden)
重建的原始点云(隐藏边)

Smoothed Point Cloud
平滑的点云

Smoothed Point Cloud
平滑的点云

Smooth Point Cloud
平滑点云

Reconstructed
重建

Reconstructed(edges hidden)
已重建(隐藏边)

Simplified Point Cloud
简化的点云

Simplified Point Cloud
简化的点云

Simplified Point Cloud
简化的点云

Reconstructed
重建

Reconstructed(edges hidden)
已重建(隐藏边)



