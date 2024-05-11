
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



































