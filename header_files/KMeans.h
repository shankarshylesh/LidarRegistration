#include <bits/stdc++.h>
#include "Cluster.h"
#include "ClusterNode.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

class KMeans
{
private:
int K; // number of clusters
int total_values, total_points, max_iterations;
vector<Cluster> clusters;
// return ID of nearest center (uses euclidean distance)
int getIDNearestCenter(Point point);

public:
KMeans(int K, int total_points, int total_values, int max_iterations);
queue<ClusterNode> run(vector<Point> & points,pcl::PointCloud<pcl::PointXYZ>::Ptr clouder);
queue<ClusterNode> runEnhanced(vector<Point> & points);
queue<ClusterNode> runNew(vector<Point> & points);
};
