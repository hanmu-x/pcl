# 常用函数(1)

给出一些可能会用到的工具函数。

* **时间计算**

pcl中计算程序运行时间有很多函数，其中利用控制台的时间计算是：

```cpp
#include <pcl/console/time.h>

pcl::console::TicToc time; 
time.tic(); 

 //  + 程序段 + : 计算这里的时间

cout << time.toc()/1000 << "s" <<endl;   // TicToc 计算的是毫秒

```

就可以以秒输出“程序段”的运行时间。

* **pcl::PointCloud::Ptr 和 pcl::PointCloud 的两个类相互转换**

  PointCloud 类是点云库（PCL）中的一个重要类，用于表示和操作点云数据。点云数据是在三维空间中采样的离散点的集合，可以用于表示物体的形状、表面和环境等信息。

  PointCloud 类中包含了多个成员变量和成员函数，用于存储和处理点云数据。它可以存储不同类型的点云数据，如点的坐标、法线、颜色等。PointCloud 类提供了一些常用的函数，例如添加点、移除点、获取点的数量等，以方便对点云数据进行操作和分析。

  在使用 PointCloud 类时，可以使用 pcl::PointCloud::Ptr 来声明和操作 PointCloud 类的指针。**这是因为点云数据可能非常庞大，直接进行拷贝和传递可能会导致性能问题，而使用指针可以避免不必要的拷贝和内存开销。通过使用指针，可以方便地对点云数据进行共享和传递，提高了程序的效率和灵活性**。  

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud = *cloudPointer;
cloudPointer = cloud.makeShared();
 //  makeShared() 函数是 PCL 中 PointCloud 类的成员函数。它返回一个指向当前 PointCloud 对象的共享指针（即 pcl::PointCloudpcl::PointXYZ::Ptr 类型的指针）。通过调用 makeShared() 函数，可以创建一个共享指针，该指针与原始的 PointCloud 对象共享相同的数据。这样做的好处是避免了数据的复制和内存的额外开销，同时可以方便地传递和共享 PointCloud 对象。
```

* **如何查找点云的x，y，z的极值(最大值和最小值)**

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ> ("your_pcd_file.pcd", *cloud);   //  打开点云
pcl::PointXYZ minPt, maxPt;   //  最大值和最小值
pcl::getMinMax3D (*cloud, minPt, maxPt);  //  查询cloud句柄对应点云的最大值和最小值
```



* **知道需要保存点的索引，从原点云中拷贝点到新点云**

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\office3-after21111.pcd", *cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int > indexs = { 1, 2, 5 };
pcl::copyPointCloud(*cloud, indexs, *cloudOut);
```

* **如何从点云里删除和添加点**

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\office3-after21111.pcd", *cloud);
pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();   //  迭代器的形式

cloud->erase(index); // 删除第一个
index = cloud->begin() + 5;

cloud->erase(cloud->begin()); // 删除第5个
pcl::PointXYZ point = { 1, 1, 1 };

 // 在索引号为5的位置1上插入一点，原来的点后移一位
cloud->insert(cloud->begin() + 5, point);

cloud->push_back(point); // 从点云最后面插入一点
std::cout << cloud->points[5].x; // 输出1
```

* **链接两个点云字段（两点云大小必须相同）**

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile("/home/yxg/pcl/pcd/mid.pcd",*cloud);
//点云法线估计器 ne，用于估计点云中点的法线信息
pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;  
ne.setInputCloud(cloud); // 将先前加载的点云设置为法线估计器的输入
// 创建一个 KdTree 数据结构用于最近邻搜索
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
ne.setSearchMethod(tree); // 设置法线估计中的搜索方法为 KdTre

// 创建一个新的点云对象 cloud_normals
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>()); 
// 设置法线估计中的 K 最近邻搜索的参数为 8，即对每个点寻找其 8 个最近的邻居点来估计法线
ne.setKSearch(8);
// ne.setRadisuSearch(0.3);
ne.compute(*cloud_normals);  // 计算法线估计并将结果存储在 cloud_normals 中
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_nomal (new pcl::PointCloud<pcl::PointNormal>);
// 先前加载的原始点云 cloud 和估计出的法线信息 cloud_normals 合并为带有法线信息的新点云 cloud_with_normal
pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_nomal);
```

* **如何从点云中删除无效点**

pcl中的无效点是指：点的某一坐标值为**nan**

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
    
using namespace std;
typedef pcl::PointXYZRGBA point;
typedef pcl::PointCloud<point> CloudType;
    
int main (int argc,char **argv)
{
  CloudType::Ptr cloud (new CloudType);
  CloudType::Ptr output (new CloudType);
              
  pcl::io::loadPCDFile(argv[1],*cloud);
  cout<<"size is:"<<cloud->size()<<endl;
                     
  vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*output,indices);
  // removeNaNFromPointCloud 函数，将原始点云 cloud 中包含 NaN 值的点去除，并将结果存储在新的点云对象 output 中。去除的点的索引将被保存在 indices 中
  cout<<"output size:"<<output->size()<<endl;
             
  pcl::io::savePCDFile("out.pcd",*output);
    
  return 0;
}
```

* **计算质心**

```cpp
Eigen::Vector4f centroid;   // 质心
pcl::compute3DCentroid(*cloud_smoothed,centroid);  // 估计质心的坐标
```

* **从网格提取顶点（将网格转化为点）**

```cpp
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h> // loadPolygonFileOBJ 所属头文件
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
using namespace pcl;
int main(int argc,char **argv)
{
  pcl::PolygonMesh mesh;
   //    pcl::io::loadPolygonFileOBJ(argv[1], mesh);
  pcl::io::loadPLYFile(argv[1],mesh);  // 从指定的PLY文件加载多边形网格数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // fromPCLPointCloud2 从多边形网格的pcl::PCLPointCloud2 表示中提取点云数据，并将其转换为 pcl::PointCloud<pcl::PointXYZ>的格式
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  //  将点云数据保存为ASCII格式的PCD文件。文件名为 "result.pcd"，并且保存的点云数据来自于前面从PLY文件中提取的数据
  pcl::io::savePCDFileASCII("result.pcd", *cloud);
  return 0;
}
```

以上代码可以从.obj或.ply面片格式转化为点云类型。

