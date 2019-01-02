#include<bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace std;

double compareFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr parentCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr childCloud)
{
	
	// d(A,B)=1/(|A|+|B|)(sigma(a,B)+sigma(b,A))
	// kdtree is used for distance measure

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_parentColud;
    kdtree_parentColud.setInputCloud (parentCloud);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_childColud;
    kdtree_childColud.setInputCloud (childCloud);

    double sigmaaB=0,sigmabA=0;
    double answer=0;
    int iter;

    // finding sigma(a,B)

   long long int  totalPointsParent = parentCloud->points.size ();


 for( iter = 0; iter < totalPointsParent; iter++)
{

 std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);

     kdtree_childColud.nearestKSearch (parentCloud->points[iter], 1, indices, sqr_distances);
     sigmaaB += sqrt(sqr_distances[0]);
}


 // finding sigma(b,A)

    long long int  totalPointsChild = childCloud->points.size ();


 for( iter = 0; iter < totalPointsChild; iter++)
{

 std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);

     kdtree_parentColud.nearestKSearch (childCloud->points[iter], 1, indices, sqr_distances);
     sigmabA += sqrt(sqr_distances[0]);
}

answer=1/(totalPointsChild+totalPointsParent);
answer*=(sigmabA+sigmaaB);
return answer;
}