## PCL Installation and Simple Usage Guide
> Author: Lifan Chen  
Email: lifan.chen@tusimple.ai    
Date: 05/17/2017


## Installation on Linux (Using PPA) (Recommedned)
Reference: [Prebuilt binaries for Linux](http://pointclouds.org/downloads/linux.html)

### Ubuntu
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```


### Debian
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 19274DEF
sudo echo "deb http://ppa.launchpad.net/v-launchpad-jochen-sprickerhof-de/pcl/ubuntu maverick main" >> /etc/apt/sources.list
sudo apt-get update
sudo apt-get install libpcl-all

```

## Installation on Linux (Make from source)

#### Step 1: Install PCL
```
sudo apt-get update  
sudo apt-get install git  

cd ~/Documents    
git clone https://github.com/PointCloudLibrary/pcl.git pcl-trunk    
ln -s pcl-trunk pcl    
```

#### Step 2: Install Prerequisites
```
sudo apt-get install g++
sudo apt-get install cmake cmake-gui
sudo apt-get install doxygen   
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common   
sudo apt-get install libflann1.8 libflann-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libvtk5.8-qt4 libvtk5.8 libvtk5-dev
sudo apt-get install libqhull*
sudo apt-get install libusb-dev
sudo apt-get install libgtest-dev
sudo apt-get install git-core freeglut3-dev pkg-config
sudo apt-get install build-essential libxmu-dev libxi-dev  
sudo apt-get install libusb-1-0-dev graphviz mono-complete
sudo apt-get install qt-sdk openjdk-7-jdk openjdk-7-jre
sudo apt-get install phonon-backend-gstreamer
sudo apt-get install phonon-backend-vlc
```

#### Step 3: Compile and Install PCL:
```
cd ~/Documents/pcl
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=None -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON ..
make
sudo make install
```


## Using PCL for visualization point cloud (.pcd) files
Open a terminal in the folder that contains __.pcd__ files. Type in the following commands (support single frame and multi-frame).
```
pcl_viewer 1486170805.2_FullRes.pcd 1486170805.3_FullRes.pcd 1486170805.4_FullRes.pcd
```

#### Using PCL in you CPP project
Add following commands in _CMakeLists.txt_
```
find_package(PCL 1.8 REQUIRED)
# PCL linking
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(${SOURCE_FILES})
TARGET_LINK_LIBRARIES(${PCL_LIBRARIES})
```

#### Basic includes for PCL in .cpp/.h files:
```cpp
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
```

#### Load PointCloud in code
One way:
```cpp
typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointer;
CloudPointer loadCloudFromPath(std::string vPath){
    CloudPointer curCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(vPath.c_str(), cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *curCloud);
    return curCloud;
}
```  

Another way:
```cpp
typedef PointXYZI PointType;
typedef PointCloud<PointType>::Ptr CloudPointer;
const string cloudPath = "./x.pcd";
CloudPointer curCloud(new pcl::PointCloud<PointType>());
(io::loadPCDFile<PointXYZI>)(cloudPath, *curCloud);
```


#### Visualize PointCloud in cpp project. (Use of visualizer)
```cpp
typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointer;
void visualizeCloud(CloudPointer first, const std::string msg){
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (first, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(first, tgt_h, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    if (msg != "") {
        viewer->addText(msg, 5, 5, 10, 255, 0, 0,  "text");
    }
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
```

## Key class in PCL (important):

#### Kdtree
* Usage: radius search/k nearest search with fast speed.
* Tutorial: [Kdtree](http://pointclouds.org/documentation/tutorials/kdtree_search.php)

#### Eucliean Cluster Extraction
* Usage: cluster based on eucliean distance between points.
* Tutorial: [Eucliean Cluster Extraction](http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)

#### Octree
* Usage: Voxelize point cloud
* Tutorial: [Octree](http://docs.pointclouds.org/1.7.1/group__octree.html)

#### SACSegmentation
* Usage: remove ground plane/fit other generic model in point cloud
* Tutorial: [Plane removal](http://pointclouds.org/documentation/tutorials/planar_segmentation.php)


---
## Play _.bag_ files with RVIZ
__Before this tutorial, make sure your computer has correctly install ROS, (Type roscore in temrinal to check)__ or see this link for help [Ubuntu ROS Installation](http://wiki.ros.org/indigo/Installation/Ubuntu).

#### Step I
Open the terminal and type
```
rviz
```
The window of rviz will pop up.


#### Step II
Open another terminal and type
```
rosbag info <your bagfile>
```
Here, _your bagfile_ is the file name of the bag file that you want to open. You will see the information of this bag file. Specifically, you could find the data type contains in this bag file, e.g. image, point clouds, etc.

#### Step III
In the terminal type
```
rosbag play <your bagfile>
```

Now, you bag file starts to play. You could stop at any time by pressing __space__ on your keyboard. But there is still nothing shows up in rviz. Don't worry, you will see it in the next step.

#### Step IV
Open rviz. Set the __fixed frame__ attribute in _Global Options_ to velodyne.  Press the __add__ button, and add the topics that you want to visualize and you see in the bag information. For image, it is in the _image raw_ and for point cloud it is in the _velodyne points - point cloud_.

Now you should see image/point cloud playing in your rviz.
