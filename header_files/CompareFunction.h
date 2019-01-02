#include<bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace std;

double compareFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr parentCloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr childCloud);