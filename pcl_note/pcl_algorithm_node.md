
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


## 点云平面提取


```cpp
// 线对原始点云做 VoxelGrid滤波下采样,方便后面的计算
pcl::PCLPointCloud2::Ptr cloud 下采样->pcl::PCLPointCloud2::Ptr cloud_filtered_blob
// 数据类型转化
pcl::PCLPointCloud2::Ptr cloud_filtered_blob -> pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
// 创建分割对象 (pcl::SACSegmentation进行随机采样一致性（RANSAC）平面分割)
pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setOptimizeCoefficients(true);        // 设置对估计模型参数进行优化处理
seg.setModelType(pcl::SACMODEL_PLANE);    // 设置分割模型类别
seg.setMethodType(pcl::SAC_RANSAC);       // 设置用哪个随机参数估计方法
seg.setMaxIterations(1000);               // 设置最大迭代次数
seg.setDistanceThreshold(0.01);           // 判断是否为模型内点的距离阀值

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

```

这个函数`cloudExtraction`是用于从输入的点云数据中提取多个平面的。它主要通过以下步骤实现这一目标：


1. **点云预处理**：
   - 首先，对输入点云进行下采样，降低其密度。这一步通常是为了减少计算量和加快后续处理速度。
   - 下采样后的点云转换为`pcl::PointCloud<pcl::PointXYZ>`格式，以便于后续处理。

2. **平面分割**：
   - 使用`pcl::SACSegmentation`类进行随机采样一致性（RANSAC）平面分割。该过程会迭代寻找最适合数据集的平面模型，并将符合该平面模型的点（即内点）的索引存储在`pcl::PointIndices`中，同时计算得到的平面模型参数存储在`pcl::ModelCoefficients`中。
   - 分割前设置了一系列参数，包括优化系数、模型类型（平面）、RANSAC方法、最大迭代次数以及距离阈值，以确保高效且准确地识别平面。
   - 通过一个循环持续分割，直到剩余点云的数量小于初始点云数量的70%（这里有个小错误，应该是`cloud_filtered->points.size() < 0.3 * nr_points`），每次循环都会从当前点云中分离出一个平面。

3. **点云提取与存储**：
   - 使用`pcl::ExtractIndices`类根据上一步得到的内点索引提取对应的平面点云，并将其添加到`vecCloud`向量中，用于存储所有提取到的平面点云。
   - 在每次提取出一个平面后，通过将`setNegative(true)`，从当前的点云中移除已提取的平面点云，继续对剩余点云进行下一次分割，直到达到停止条件。



## 点云法线估计

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setInputCloud(cloud);

// 基于给出的输入数据集，KdTree将被建立
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
ne.setSearchMethod(tree);
// 存储输出数据
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
// 使用半径在查询点周围3厘米范围内的所有临近元素
ne.setRadiusSearch(radius);  // 单位:米
// 计算特征值
ne.compute(*cloud_normals);
```

这个函数`normalCalculation`的作用是从给定的点云中计算每个点的法线向量，这对于后续的许多点云处理任务如曲面重建、特征描述、分割等都是非常重要的。以下是该函数的详细解释及分析：

### 函数功能介绍：

1. **法线估计器创建**：
   - 使用`pcl::NormalEstimation`类创建一个法线估计器对象`ne`，该对象负责计算点云中每个点的法线。

2. **输入点云设置**：
   - 通过`ne.setInputCloud(cloud)`设置需要计算法线的输入点云。

3. **构建KdTree**：
   - 创建一个`pcl::search::KdTree<pcl::PointXYZ>`类型的搜索树对象`tree`，并将其作为搜索方法设置给法线估计器`ne`。KdTree是一种高效的近邻搜索算法，能够快速找到给定点周围的邻近点

   - 初始化一个指向`pcl::PointCloud<pcl::Normal>`类型的智能指针`cloud_normals`，用于存储计算出的法线信息。
   - 通过`ne.setRadiusSearch(radius)`设置邻域搜索的半径，即在计算每个点的法线时考虑的邻域范围。

4. **计算法线**：
   - 调用`ne.compute(*cloud_normals)`开始计算法线，并将结果存储在`cloud_normals`中。

5. **返回结果**：
   - 函数最后返回计算得到的法线云`cloud_normals`。



## 积分图法线估计


```cpp
// 创建法线估计向量
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);  // 设置法线估计的方式AVERAGE_3D_GRADIENT
ne.setMaxDepthChangeFactor(depth_factor);   // 设置深度变化系数
ne.setNormalSmoothingSize(smooth_size);     // 设置法线优化时考虑的邻域的大小
ne.setInputCloud(cloud);                    // 输入的点云
ne.compute(*normals);                       // 计算法线
```

- 三种法线估计方法
  - `AVERAGE_3D_GRADIENT` 模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
  - `COVARIANCE_MATRIX` 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
  - `AVERAGE_DEPTH_CHANGE` 模式只创建了一个单一的积分图，从而平局深度变化计算法线




## 平滑和法线估计


```cpp

// 创建一个 KD-Tree
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

// 输出具有PointNormal类型，以便存储MLS计算的法线
// 用于存储经过平滑处理后的点云数据以及每个点的法线信息。pcl::PointNormal类型包含位置信息（XYZ坐标）和法线信息（NXNYNZ）。
pcl::PointCloud<pcl::PointNormal> mls_points;

// 这是PCL中用于执行移动最小二乘（Moving Least Squares, MLS）平滑和法线估计的关键类
// 该算法基于每个点的邻域信息，通过最小二乘拟合的方式生成一个新的、更加平滑的表面，并同时计算每个点的法线
pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

mls.setComputeNormals(true);    // 开启法线计算功能。

// 设置参数
mls.setInputCloud(cloud);
mls.setPolynomialOrder(2);  // 设置多项式拟合的阶数为2，这是一个平滑度的控制参数，阶数越高表示平滑程度越高。
mls.setSearchMethod(tree);  // KD-Tree作为近邻搜索
mls.setSearchRadius(2);  // 设定搜索邻域的半径长度，这个值决定了参与平滑和法线计算的邻域大小。

mls.process(mls_points);  // 执行平滑处理和法线计算

return mls_points;

```



## 平面分割


```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;     // 平面分割的点云

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);   // 输出参数，用于存储分割得到的平面模型系数
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                  // 输出参数，存储被认为是平面模型一部分的点的索引（即局内点）

std::cout << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

// 创建分割对象
pcl::SACSegmentation<pcl::PointXYZ> seg;
// 可选择配置，设置模型系数需要优化
seg.setOptimizeCoefficients(true);
// 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
seg.setModelType(pcl::SACMODEL_PLANE);  // 设置模型类型
seg.setMethodType(pcl::SAC_RANSAC);     // 设置随机采样一致性方法类型
seg.setDistanceThreshold(0.01);         // 设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                        // 表示点到估计模型的距离最大值

seg.setInputCloud(cloud);
// 引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
seg.segment(*inliers, *coefficients);
if (inliers->indices.size() == 0)
{
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    return false;
}
return true;

```

- pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    - pcl::ModelCoefficients 用于存储平面模型的系数（A、B、C和D）
    - Model coefficients: 0 0 1 -1：平面模型的系数表示为[A, B, C, D]，其中A、B、C表示平面的法向量，D表示平面到原点的距离。在这里，系数为[0, 0, 1, -1]，表示平面的法向量在Z轴上，距离原点的距离为1，即平面方程为Z=1。
- pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    - 用于存储内点的索引


## 圆柱体模型的分割


```cpp



```
































