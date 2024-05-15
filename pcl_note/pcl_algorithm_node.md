
# 1. pcl库函数


## 1. kdtree的k近邻索引

功能: 搜索出最近的K个点

```cpp
std::vector<int> pointIdxNKNSearch(k);          // 存储查询点近邻索引
std::vector<float> pointNKNSquaredDistance(k);  // 存储近邻点对应距离平方
// 核心函数
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
int num = kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);
```

最终得到两个结果:
 -  pointIdxNKNSearch: 存储查询点近邻索引
 -  pointNKNSquaredDistance: 存储近邻点对应距离平方

## 2. kdtree的半径近邻索引

```cpp
std::vector<int> pointIdxRadiusSearch;          // 存储近邻索引
std::vector<float> pointRadiusSquaredDistance;  // 存储近邻对应距离的平方
// 核心函数
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
int num = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
```

最终得到两个结果:
 -  pointIdxNKNSearch: 存储查询点近邻索引
 -  pointNKNSquaredDistance: 存储近邻点对应距离平方



## 3. octree

```cpp
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
```

pcl::octree::OctreePointCloudSearch对象时，需要提供一个参数resolution，它代表了Octree的分辨率，即每个八叉树节点的尺寸。

具体来说，resolution指定了八叉树中每个立方体体素（voxel）的边长。这个值越小，八叉树的层级就越多，数据的分辨率就越高，但同时也会导致存储和查询的开销增加。相反，如果resolution较大，则八叉树的层级较少，数据的分辨率较低，但存储和查询的开销相对较小。

## 4. octree 体素近邻搜索

体素近邻搜索(Neighbors within voxel search)：它把查询点所在的体素中其他点的索引作为查询结果返回,结果以点索引向量的形式保存,因此搜索点和搜索结果之间的距离取决于八叉树的分辨率参数，根据需要修改resolution。

叉树中查找与给定搜索点紧邻的体素，并将这些体素中包含的点的索引存储在pointIdxVec向量中。这些紧邻的体素通常是以搜索点为中心的立方体体素，它们与搜索点相邻或包含搜索点。

，邻近体素（Neighboring Voxel）指的是与给定搜索点紧邻的体素。体素是三维空间中的立方体单元，用于离散化表示空间，并在点云处理中用于空间搜索和采样。
当我们进行空间搜索时，通常会将空间划分为多个体素，然后在每个体素中进行搜索。邻近体素即是与给定搜索点紧邻的这些体素。
在八叉树（Octree）等空间数据结构中，体素通常是树的叶子节点，每个叶子节点代表一个体素。通过将空间划分为体素，我们可以有效地组织和管理点云数据，并且可以快速地进行空间查询，例如近邻搜索、体素内点的检索等。
因此，在PCL中，邻近体素指的是与给定搜索点紧邻的这些离散化空间体素。在空间搜索任务中，我们可以利用邻近体素来查找紧邻点的属性或邻近体素中的点。

```cpp
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);      // 初始化Octree
octree.setInputCloud(cloud);                                                // 设置输入点云
octree.addPointsFromInputCloud();                                           // 构建octree
pcl::PointXYZ searchPoint;
std::vector<int> pointIdxVec;                                               // 存储体素近邻搜索结果向量
octree.voxelSearch(searchPoint, pointIdxVec);                               // 执行搜索
```


## octree的k近邻索引

搜索出给定点最近的k个点

```cpp
pcl::PointXYZ searchPoint;
std::vector<int> pointIdxNKNSearch;          // 结果点的索引的向量
std::vector<float> pointNKNSquaredDistance;  // 搜索点与近邻之间的距离的平方
octree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);

```


## octree的半径近邻索引

检索出索引点在指定半径内的所有点

```cpp
float radius;  //半径
pcl::PointXYZ searchPoint;
std::vector<int> pointIdxRadiusSearch;  // 结果点的索引
std::vector<float> pointRadiusSquaredDistance; //  搜索点与近邻之间的距离的平方
octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)
```

## KDTree和Octree的区别

### KD 树（KDTree）：

 - KD 树是一种二叉树数据结构，用于对 k 维空间中的点进行分割和组织。 构建 KD
 - 树的过程是递归的，在每一层选择一个维度进行划分，以便将点云空间划分为两个子空间，然后在子空间中继续划分。 在搜索过程中，KD
-  树通过在树中逐级向下遍历，根据点的坐标值来确定搜索路径，从而在平均情况下实现较快的搜索速度。 
-  优点：**适用于低维空间和相对稠密的点云数据**。

### Octree（Octree）：

 - Octree 是一种八叉树数据结构，用于对 3D 空间进行递归分割和组织。    构建 Octree 的过程也是递归的，将 3D
 -  空间分割为八个立方体子区域，然后对每个子区域递归执行相同的操作，直到达到指定的深度或者每个子区域中的点数达到阈值。   
-   在搜索过程中，Octree 通过遍历树结构，根据点的位置在空间中确定搜索路径，以及检查每个节点和叶子节点来实现搜索。   
-   优点：适用于处理 3D 点云数据，尤其在**处理稀疏点云和空间数据分布不均匀的情况下表现良好**。

总的来说，**KD 树适用于低维度空间和相对密集的点云数据**，而 **Octree 适用于处理 3D 点云数据，特别是在处理稀疏数据和空间分布不均匀**的情况下效果更好。选择使用哪种数据结构取决于您的应用场景和数据特征。



## 随机采样一致性算法


## 直通滤波

仅保留指定点云字段在指定范围内的数据

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
// 设置滤波器对象
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(cloud);                   // 设置输入点云
pass.setFilterFieldName(field_name);         // 设置过滤时所需要点云类型的Z字段
pass.setFilterLimits(Limit_low, Limit_hig);  // 设置在过滤字段的范围
pass.filter(*cloud_filtered);  // 执行滤波，保存过滤结果在cloud_filtered
```

## VoxelGrid滤波下采样

- 对点云进行抽稀处理

```cpp
pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  // 创建滤波对象
sor.setInputCloud(cloud);                 // 设置需要过滤的点云给滤波对象
将点云空间划分为均匀的立方体单元（体素），并将每个体素内的点合并成一个单独的点
sor.setLeafSize(lx, ly, lz);              // 设置滤波时创建的体素体积 单位：m
sor.filter(*cloud_filtered);              // 执行滤波处理，存储输出
```

sor.setLeafSize(lx, ly, lz); 
它将点云空间划分为均匀的立方体单元（体素），并将每个体素内的点合并成一个单独的点，从而减少点云数据量。


## statisticalOutlierRemoval滤波器移除离群点

StatisticalOutlierRemoval滤波器，主要用于从点云数据中识别并移除离群点。这些离群点可能是由于传感器噪声、环境干扰或是数据采集错误等原因造成的，它们通常不符合点云数据的整体分布规律，对于后续的点云分析（如表面重建、特征提取等）可能产生不利影响。

工作原理

    统计分析：对于点云中的每一个点，滤波器首先计算它到其邻近点的平均距离。这个邻近点的选取可以基于设定的邻域半径或者直接指定的邻域点数。

    假设分布：基于计算出的所有点到其邻域点的距离，假定这些距离服从高斯分布（正态分布）。在实际应用中，这意味着点云中大部分点到其邻域点的平均距离应该集中在某个均值附近，并且围绕这个均值的偏差（标准差）是有限的。

    阈值判断：根据高斯分布的特性，滤波器会计算出一个距离阈值，这个阈值通常是基于均值和标准差的倍数（例如，标准差的2-3倍）来确定。任何点到其邻域点的平均距离如果超出了这个阈值，则会被视为离群点。



```cpp
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;  // 创建滤波器对象

sor.setInputCloud(cloud);           // 设置待滤波的点云
sor.setMeanK(meank);                // 设置在进行统计时考虑查询点临近点数
sor.setStddevMulThresh(threshold);  // 设置判断是否为离群点的阀值,较大的阈值将导致更多的点被判定为离群点。
sor.setNegative(inversion);         // 是否对结果取反,false:删除离群点,true:保留离群点
sor.filter(*cloud_filtered);        // 存储
```

1.meank 参数：
    meank 是一个整数参数，表示在进行统计时考虑的查询点的临近点数。
    统计滤波器对每个点周围的临近点进行统计计算，以判断该点是否为离群点。meank 决定了用于计算统计信息的邻近点的数量。较大的 meank 值会考虑更多的邻近点，从而更准确地计算统计信息。

2.threshold 参数：
    threshold 是一个浮点数参数，表示判断是否为离群点的阈值。较大的阈值将导致更多的点被判定为离群点，而较小的阈值则导致更少的点被判定为离群点。
    统计滤波器通过计算每个点的邻域内点的平均距离和标准差，并根据阈值判断该点是否为离群点。如果一个点的距离超出了阈值的多少个标准差，它将被认为是离群点。

3.inversion 参数：
    inversion 是一个布尔类型的参数，表示是否对滤波结果进行反转。如果设置为 false，则删除被判断为离群点的点；如果设置为 true，则保留被判断为离群点的点，而删除其余点。



## RadiusOutlinerRemoval 移除离群点

RadiusOutlinerRemoval滤波器，它可以删除在输入点云一定范围内没有达到足够多近邻的所有数据点。

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  // 创建滤波器

outrem.setInputCloud(cloud);                  // 设置输入点云
outrem.setRadiusSearch(radius);               // 设置半径为~的范围内找临近点
outrem.setMinNeighborsInRadius(minInRadius);  // 设置查询点的邻域点集数小于~的删除
outrem.filter(*cloud_filtered);  // 执行条件滤波   在半径为radius 在此半径内必须要有minInRadius个邻居点，此点才会保存

```

## ConditionalRemoval 移除离群点

ConditionalRemoval滤波器，可以一次删除满足对输入的点云设定的一个或多个条件指标的所有的数据


```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());  // 创建条件定义对象

// ------------------添加条件------------------
pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0));     // 添加在Z字段上大于0的比较算子
pcl::FieldComparison<pcl::PointXYZ>::ConstPtr comp2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8));     // 添加在Z字段上小于0.8的比较算子
range_cond->addComparison(comp1);
range_cond->addComparison(comp2);
// ------------------添加条件------------------


// 创建滤波器并用条件定义对象初始化
pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
condrem.setCondition(range_cond);
condrem.setInputCloud(cloud);    // 输入点云
condrem.setKeepOrganized(true);  // 设置保持点云的结构
// 执行滤波
condrem.filter(*cloud_filtered);
```

// 添加在Z字段上大于0的比较算子
// GT greater than
// EQ equal
// LT less than
// GE greater than or equal
// LE less than  or equal 小于等于


## 双边滤波

双边滤波算法，是通过取邻近采样点的加权平均来修正当前采样点的位置，从而达到滤波效果。同时也会有选择地剔除部分与当前采样点“差异”太大的相邻采样点，从而达到保持原特征的目的。

双边滤波的核心概念仍然围绕着两个关键参数：空间域参数和值域参数。
空间域参数保证了邻近点的考虑范围，值域参数则确保了只有属性值相近的点才施加较大的影响，这样就能在平滑和保持细节之间找到一个良好的平衡

1. 空间域参数

空间域参数通常是指滤波时考虑的邻域大小或范围，它直接影响滤波在空间上的平滑程度。在PCL中，这个参数可能直接体现在滤波器设置的半径或邻域尺寸上，例如，通过setHalfSize方法设置的半径大小。具体来说：

    半径（Half Size）：定义了每个点周围考虑邻域的范围。在双边滤波中，这意味着滤波器会在每个点周围的球形区域内查找邻近点，这个球的半径就是由这个参数决定的。较大的半径会导致更广泛的邻域参与计算，使得滤波后的结果更加平滑，但同时也可能导致更多的边缘信息损失；较小的半径则相反，保持了更多的细节，但可能不足以去除所有噪声。

2. 值域参数

值域参数关注的是点云中点的属性值差异，最常见的是点的强度值。这个参数控制着滤波器在考虑点的相似性时的宽容度，即在属性值上多大差异的点被认为是相似的，可以相互影响：

    灰度阈值（StddevMulThresh 或 SigmaSqr）：这个参数决定了在值域中多少差异被认为是重要的。通常，它是标准差的倍数，用来衡量点云中强度值的波动程度。较高的值意味着滤波器对强度差异更加不敏感，即使强度变化较大，点之间的影响权重依然较高，这有助于在保持颜色或强度一致性的区域中进行平滑；而较低的值则使得滤波器对强度变化更加敏感，有助于在强度变化明显的边界区域保持锐利。



```cpp
pcl::PointCloud<pcl::PointXYZI>::Ptr PclTool::bilateralFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const double standard_dev, const double halfSize)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    // 创建了一个双边滤波器对象 fbf, 指定了点云中每个点的类型pcl::PointXYZI
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(cloud);
    fbf.setSearchMethod(tree);
    fbf.setHalfSize(halfSize);    // 空间域参数
    fbf.setStdDev(standard_dev);  // 值域参数
    fbf.filter(*cloud_filtered);

    return cloud_filtered;
}
```

### 1. `fbf.setHalfSize(halfSize);`

`halfSize`
- 此参数通常指定了滤波处理时考虑的邻域空间范围的半径大小。在三维点云处理中，它定义了一个以每个点为中心的立方体或球体邻域的半边长或半径。
  - 参数值越大,意味着更大的邻域，更多的邻近点将参与到滤波计算中，这有助于去除噪声，但也可能抹去更多细节和边缘信息。
  - 参数值越小,会限制影响范围，保留更多局部细节，但可能不足以平滑掉所有的噪声点。这个参数直接影响滤波器的空间局部性，是控制平滑程度和边缘保持的关键。

综上所述，`setStdDev`和`setHalfSize`这两个方法共同决定了滤波器如何在空间维度和属性值维度上平衡平滑度与细节保留，是调节滤波效果的重要手段。

### 2. `fbf.setStdDev(standard_dev);`

`standard_dev`
- 这个参数代表了在值域（或强度域、范围域）中的标准差乘数阈值。在滤波器中，它用来控制点云中点的属性值（如强度或颜色）差异的容忍度。换句话说，它决定了在计算点的贡献权重时，其属性值与中心点的差异达到多少时开始显著减小权重。
    - 参数值越大意味着滤波器对更大范围的属性值差异接受度更高，允许更多差异较大的点影响中心点的滤波结果，这可能导致更平滑的输出但可能模糊某些细节。
    - 参数值越较小的值会让滤波器对属性值的变化更加敏感，有助于保持边缘清晰。




## 点云投影

```cpp
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    // 填充 ModelCoefficients 的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1 也就是X——Y平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = 1;
    coefficients->values[3] = 0;

    // 创建 ProjectInliers 对象，使用ModelCoefficients作为投影对象的模型参数
    pcl::ProjectInliers<pcl::PointXYZ> proj;  // 创建投影滤波对象
    proj.setModelType(pcl::SACMODEL_PLANE);   // 设置对象对应的投影模型
    proj.setInputCloud(cloud);                // 设置输入点云
    proj.setModelCoefficients(coefficients);  // 设置模型对应的系数
    proj.filter(*cloud_projected);            // 投影结果存储cloud_projected
```











