# LeGO_LOAM
一、LeGO _LOAM安装

# 下载gtsam

cd ~
git clone https://bitbucket.org/gtborg/gtsam.git
#编译

cd ~/gtsam
mkdir build
cd build
cmake ..
make check   #可选的，运行单元测试，我没执行这个命令，因为在TX2上编译太慢了，太慢了，太慢了
make install
#下载编译Lego_loam

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make -j1
 当第一次编译代码时，需要在“catkin_make”后面添加“-j1”以生成一些消息类型。将来的编译不需要“-j1”。

编译时出现错误，找不到Cmake文件，则需要在路径中添加相应的环境变量

catkin_make编译错误_weixin_30485291的博客-CSDN博客

Ubuntu下三种方法设置环境变量 - cv_gordon - 博客园

#运行

运行lego_loam程序

roslaunch lego_loam run.launch
运行数据包

一定要加--clock时间，这样才能生成完整的全局地图

rosbag play 3-1.bag --clock


#转存为PCD文件

在代码建图过程中保存数据

rosbag record -o out /laser_cloud_surround
转化为pcd文件

rosrun pcl_ros bag_to_pcd input.bag /laser_cloud_surround pcd
查看pcd文件

pcl_viewer xxxxxx.pcd

二、LeGO _LOAM调试

utility文件修改

//1、激光雷达选型
// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;
 
 
//pandar-40
extern const int N_SCAN =40;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 0.33;
extern const float ang_bottom = 3.0+0.1;
extern const int groundScanInd = 10;
 
//VLP-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 20;
 
// 2、特征点提取分割
extern const int edgeFeatureNum = 6;
extern const int surfFeatureNum = 10;
extern const int sectionsTotal = 2;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;
 
//3、输入点云类型定义
struct EIGEN_ALIGN16 PointXYZRGBI
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    float intensity;     //intensity
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
 
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (uint8_t,rgb,rgb)
                                  (float,intensity,intensity)
)
typedef PointXYZRGBI  PointType;
