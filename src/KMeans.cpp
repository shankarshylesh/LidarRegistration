#include<bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "KMeans.h"
#include <pcl/filters/filter.h>

using namespace std;

// return ID of nearest center (uses euclidean distance)
int KMeans::getIDNearestCenter(Point point)
{
double sum = 0.0, min_dist;
int id_cluster_center = 0;

for(int i = 0; i < total_values; i++)
{
sum += pow(clusters[0].getCentralValue(i) -
point.getValue(i), 2.0);
}

min_dist = sqrt(sum);

for(int i = 1; i < K; i++)
{
double dist;
sum = 0.0;

for(int j = 0; j < total_values; j++)
{
sum += pow(clusters[i].getCentralValue(j) -
point.getValue(j), 2.0);
}

dist = sqrt(sum);

if(dist < min_dist)
{
min_dist = dist;
id_cluster_center = i;
}
}

return id_cluster_center;
}


KMeans::KMeans(int K, int total_points, int total_values, int max_iterations)
{
this->K = K;
this->total_points = total_points;
this->total_values = total_values;
this->max_iterations = max_iterations;
}

queue<ClusterNode> KMeans::run(vector<Point> & points,pcl::PointCloud<pcl::PointXYZ>::Ptr clouder)
{

queue<ClusterNode> empty;

if(K > total_points)
return empty ;

vector<int> prohibited_indexes;

////////////////////////

pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
tempCloud=clouder;

// choose K distinct values for the centers of the clusters

for(int i=0;i<K;i++)
{

pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_clouder;
kdtree_clouder.setInputCloud (tempCloud);
long long int  totalPointsClouder = tempCloud->points.size ();

//define NaN
float nan  = std::numeric_limits<float>::quiet_NaN(); 


float _radius= 10 ;

// initial centroid selection using kdtree 
int index_point= rand() % totalPointsClouder;


//searchPoint
pcl::PointXYZ searchPoint = tempCloud->at(index_point) ;
points[index_point].setCluster(i);
Cluster cluster(i, points[index_point]);
clusters.push_back(cluster);

//result
std::vector<int> pointIdxRadiusSearch;
std::vector<float> pointRadiusSquaredDistance;

if ( kdtree_clouder.radiusSearch (searchPoint, _radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
{      
//set all found points to NaN
for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
{
tempCloud->at(pointIdxRadiusSearch[j]).x = nan;
tempCloud->at(pointIdxRadiusSearch[j]).z = nan;
tempCloud->at(pointIdxRadiusSearch[j]).y = nan;
}
}
tempCloud->at(index_point).x=nan;
tempCloud->at(index_point).y=nan;
tempCloud->at(index_point).z=nan;

//remove all invalid points after all radius search
tempCloud->is_dense = false;  //otherwise removeNaNFromPointCloud does nothing
std::vector< int > index ;
pcl::removeNaNFromPointCloud(*(tempCloud), *(tempCloud), index);  

}

/*

// int index_point = rand() % totalPointsClouder;
for(;index_point < totalPointsClouder;index_point++)
{
if(mp[index_point]==0)
{
std::vector<int> indices (densityLimit);
std::vector<float> sqr_distances (densityLimit);
kdtree_clouder.nearestKSearch (clouder->points[index_point], densityLimit, indices, sqr_distances);
for(long long int j=0;j<densityLimit;j++)
{
mp[indices[j]]++;
}

points[index_point].setCluster(i);
Cluster cluster(i, points[index_point]);
clusters.push_back(cluster);
break; 
}
}
}
*/

//////////////////////////////////////
/*
for(int i = 0; i < K; i++)
{
while(true)
{
int index_point = rand() % total_points;

if(find(prohibited_indexes.begin(), prohibited_indexes.end(),
index_point) == prohibited_indexes.end())
{
prohibited_indexes.push_back(index_point);
points[index_point].setCluster(i);
Cluster cluster(i, points[index_point]);
clusters.push_back(cluster);
break;
}
}
}
*/

int iter = 1;

while(true)
{
bool done = true;

// associates each point to the nearest center
for(int i = 0; i < total_points; i++)
{
int id_old_cluster = points[i].getCluster();
int id_nearest_center = getIDNearestCenter(points[i]);

if(id_old_cluster != id_nearest_center)
{
if(id_old_cluster != -1)
clusters[id_old_cluster].removePoint(points[i].getID());

points[i].setCluster(id_nearest_center);
clusters[id_nearest_center].addPoint(points[i]);
done = false;
}
}

// recalculating the center of each cluster
for(int i = 0; i < K; i++)
{
for(int j = 0; j < total_values; j++)
{
int total_points_cluster = clusters[i].getTotalPoints();
double sum = 0.0;

if(total_points_cluster > 0)
{
for(int p = 0; p < total_points_cluster; p++)
sum += clusters[i].getPoint(p).getValue(j);
clusters[i].setCentralValue(j, sum / total_points_cluster);
}
}
}

if(done == true || iter >= max_iterations)
{
cout << "Break in iteration " << iter << "\n\n";
break;
}

iter++;
}

// shows elements of clusters
for(int i = 0; i < K; i++)
{
int total_points_cluster =  clusters[i].getTotalPoints();

cout << "Cluster " << clusters[i].getID() + 1 << endl;
cout<<total_points_cluster<<endl;


for(int j = 0; j < total_points_cluster; j++)
{
cout << "Point " << clusters[i].getPoint(j).getID() + 1 << ": ";
for(int p = 0; p < total_values; p++)
cout << clusters[i].getPoint(j).getValue(p) << " ";


cout << endl;
}


cout << "Cluster values: ";

for(int j = 0; j < total_values; j++)
cout << clusters[i].getCentralValue(j) << " ";

cout << "\n\n";
}

cout<<"moving to rejection part"<<endl;

vector<int> threshold;
for(int i = 0; i < K; i++)
{
int total_points_cluster = clusters[i].getTotalPoints();
double sum = 0.0;
double maxm,minm;
double x,y,z;
x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);

minm=1000000000;
maxm=0;
for(int p = 0; p < total_points_cluster; p++)
{
sum+=pow(clusters[i].getPoint(p).getValue(0)-x,2);
sum+=pow(clusters[i].getPoint(p).getValue(1)-y,2);
sum+=pow(clusters[i].getPoint(p).getValue(2)-z,2);
sum=sqrt(sum);
if(sum<minm)
minm=sum;
if(sum>maxm)
maxm=sum;
sum=0;
}
// 2 from here
double ans=abs(maxm-minm);
// ans=ans*2;

for(int p = 0; p < total_points_cluster; p++)
{
sum+=pow(clusters[i].getPoint(p).getValue(0)-x,2);
sum+=pow(clusters[i].getPoint(p).getValue(1)-y,2);
sum+=pow(clusters[i].getPoint(p).getValue(2)-z,2);
sum=sqrt(sum);
if(sum>ans)
{
points.erase(points.begin() + clusters[i].getPoint(p).getID());
}
}
}

cout<<"number of remaining points "<<points.size()<<endl;


/*
//  finding the weighted central value

double xcent=0,ycent=0,zcent=0;
int total_count=0;
for(int i = 0; i < K; i++)
{

int total_points_cluster = clusters[i].getTotalPoints();
total_count+=total_points_cluster;
double x,y,z;
x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);
xcent+=total_points_cluster*x;
ycent+=total_points_cluster*y;
zcent+=total_points_cluster*z;
}
xcent/=total_count;
ycent/=total_count;
zcent/=total_count;
std::vector<double> weighted_central_values;
weighted_central_values.push_back(xcent);
weighted_central_values.push_back(ycent);
weighted_central_values.push_back(zcent);

return weighted_central_values;

*/



queue<ClusterNode> pq;
int total_count=0;
double x,y,z;
for(int i = 0; i < K; i++)
{

int total_points_cluster = clusters[i].getTotalPoints();
total_count=total_points_cluster;

x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);
// cout<<x<<" "<<y<<" "<<z<<endl;
pq.push(ClusterNode(x,y,z,total_count));
}
return pq;
}

//Enhance Kmean function
queue<ClusterNode> KMeans::runEnhanced(vector<Point> & points)
{
queue<ClusterNode> empty;

if(K > total_points)
return empty ;

vector<int> prohibited_indexes;

// choose K distinct values for the centers of the clusters

map<int,int>mp;
int first,second,third;
double s,minm,counter,limit,xcor,ycor,zcor;

for(int i=0;i<K;i++)
{
minm=10000000000;

for(int j = 0; j < total_points; j++)
{
if(mp[j]==0)
{
for(int p=j+1;p<total_points;p++)
{
if((mp[p]==0)&&(p!=j))
{
s=sqrt((points[j].getValue(0)-points[p].getValue(0))*(points[j].getValue(0)-points[p].getValue(0))
+((points[j].getValue(1)-points[p].getValue(1))*(points[j].getValue(1)-points[p].getValue(1)))
+((points[j].getValue(2)-points[p].getValue(2))*(points[j].getValue(2)-points[p].getValue(2))));
if(s<minm)
{
minm=s;
first=j;
second=p;
}
}
}
}
}

xcor=(points[first].getValue(0)+points[second].getValue(0))/2;
ycor=(points[first].getValue(1)+points[second].getValue(1))/2;
zcor=(points[first].getValue(2)+points[second].getValue(2))/2;
mp[first]=1;
mp[second]=1;
points[first].setCluster(i);
points[second].setCluster(i);
counter=2;
limit=total_points/K;
limit*=0.15;
while(counter<limit)
{
minm=10000000000;
for(int j = 0; j < total_points; j++)
{
if(mp[j]==0)
{
s=sqrt((points[j].getValue(0)-xcor)*(points[j].getValue(0)-xcor)
+((points[j].getValue(1)-ycor)*(points[j].getValue(1)-ycor))
+((points[j].getValue(2)-zcor)*(points[j].getValue(2)-zcor)));

if(s<minm)
{
minm=s;
third = j;
}
}
}
mp[third]=1;
points[third].setCluster(i);
counter++;
xcor=(points[third].getValue(0)+xcor)/2;
ycor=(points[third].getValue(1)+ycor)/2;
zcor=(points[third].getValue(2)+zcor)/2;
}

Cluster cluster(i, points[first]);
clusters.push_back(cluster);

}

int iter = 1;

while(true)
{
bool done = true;

// associates each point to the nearest center
for(int i = 0; i < total_points; i++)
{
int id_old_cluster = points[i].getCluster();
int id_nearest_center = getIDNearestCenter(points[i]);

if(id_old_cluster != id_nearest_center)
{
if(id_old_cluster != -1)
clusters[id_old_cluster].removePoint(points[i].getID());

points[i].setCluster(id_nearest_center);
clusters[id_nearest_center].addPoint(points[i]);
done = false;
}
}

// recalculating the center of each cluster
for(int i = 0; i < K; i++)
{
for(int j = 0; j < total_values; j++)
{
int total_points_cluster = clusters[i].getTotalPoints();
double sum = 0.0;

if(total_points_cluster > 0)
{
for(int p = 0; p < total_points_cluster; p++)
sum += clusters[i].getPoint(p).getValue(j);
clusters[i].setCentralValue(j, sum / total_points_cluster);
}
}
}

if(done == true || iter >= max_iterations)
{
cout << "Break in iteration " << iter << "\n\n";
break;
}

iter++;
}

// shows elements of clusters
for(int i = 0; i < K; i++)
{
int total_points_cluster =  clusters[i].getTotalPoints();

cout << "Cluster " << clusters[i].getID() + 1 << endl;
cout<<total_points_cluster<<endl;


for(int j = 0; j < total_points_cluster; j++)
{
cout << "Point " << clusters[i].getPoint(j).getID() + 1 << ": ";
for(int p = 0; p < total_values; p++)
cout << clusters[i].getPoint(j).getValue(p) << " ";


cout << endl;
}


cout << "Cluster values: ";

for(int j = 0; j < total_values; j++)
cout << clusters[i].getCentralValue(j) << " ";

cout << "\n\n";
}

cout<<"moving to rejection part"<<endl;

vector<int> threshold;
for(int i = 0; i < K; i++)
{
int total_points_cluster = clusters[i].getTotalPoints();
double sum = 0.0;
double maxm,minm;
double x,y,z;
x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);

minm=1000000000;
maxm=0;
for(int p = 0; p < total_points_cluster; p++)
{
sum+=pow(clusters[i].getPoint(p).getValue(0)-x,2);
sum+=pow(clusters[i].getPoint(p).getValue(1)-y,2);
sum+=pow(clusters[i].getPoint(p).getValue(2)-z,2);
sum=sqrt(sum);
if(sum<minm)
minm=sum;
if(sum>maxm)
maxm=sum;
sum=0;
}
// 2 from here
double ans=abs(maxm-minm);
// ans=ans*2;

for(int p = 0; p < total_points_cluster; p++)
{
sum+=pow(clusters[i].getPoint(p).getValue(0)-x,2);
sum+=pow(clusters[i].getPoint(p).getValue(1)-y,2);
sum+=pow(clusters[i].getPoint(p).getValue(2)-z,2);
sum=sqrt(sum);
if(sum>ans)
{
points.erase(points.begin() + clusters[i].getPoint(p).getID());
}
}
}

cout<<"number of remaining points "<<points.size()<<endl;


/*
//  finding the weighted central value

double xcent=0,ycent=0,zcent=0;
int total_count=0;
for(int i = 0; i < K; i++)
{

int total_points_cluster = clusters[i].getTotalPoints();
total_count+=total_points_cluster;
double x,y,z;
x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);
xcent+=total_points_cluster*x;
ycent+=total_points_cluster*y;
zcent+=total_points_cluster*z;
}
xcent/=total_count;
ycent/=total_count;
zcent/=total_count;
std::vector<double> weighted_central_values;
weighted_central_values.push_back(xcent);
weighted_central_values.push_back(ycent);
weighted_central_values.push_back(zcent);

return weighted_central_values;

*/

queue<ClusterNode> pq;
int total_count=0;
double x,y,z;
for(int i = 0; i < K; i++)
{

int total_points_cluster = clusters[i].getTotalPoints();
total_count=total_points_cluster;

x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);
// cout<<x<<" "<<y<<" "<<z<<endl;
pq.push(ClusterNode(x,y,z,total_count));
}
return pq;
}

// kmeans based on distribution

queue<ClusterNode> KMeans::runNew(vector<Point> & points)
{
queue<ClusterNode> empty;

if(K > total_points)
return empty ;

// choose K distinct values for the centers of the clusters

int i_point =(total_points/(2*K));
points[i_point].setCluster(0);
Cluster cluster(0, points[i_point]);
clusters.push_back(cluster);

for(int i = 1; i < K; i++)
{
int index_point =(i*(total_points/K))+i_point;

points[index_point].setCluster(i);
Cluster cluster(i, points[index_point]);
clusters.push_back(cluster);
}


int iter = 1;

while(true)
{
bool done = true;

// associates each point to the nearest center
for(int i = 0; i < total_points; i++)
{
int id_old_cluster = points[i].getCluster();
int id_nearest_center = getIDNearestCenter(points[i]);

if(id_old_cluster != id_nearest_center)
{
if(id_old_cluster != -1)
clusters[id_old_cluster].removePoint(points[i].getID());

points[i].setCluster(id_nearest_center);
clusters[id_nearest_center].addPoint(points[i]);
done = false;
}
}

// recalculating the center of each cluster
for(int i = 0; i < K; i++)
{
for(int j = 0; j < total_values; j++)
{
int total_points_cluster = clusters[i].getTotalPoints();
double sum = 0.0;

if(total_points_cluster > 0)
{
for(int p = 0; p < total_points_cluster; p++)
sum += clusters[i].getPoint(p).getValue(j);
clusters[i].setCentralValue(j, sum / total_points_cluster);
}
}
}

if(done == true || iter >= max_iterations)
{
cout << "Break in iteration " << iter << "\n\n";
break;
}

iter++;
}

// shows elements of clusters
for(int i = 0; i < K; i++)
{
int total_points_cluster =  clusters[i].getTotalPoints();

cout << "Cluster " << clusters[i].getID() + 1 << endl;
			 cout<<total_points_cluster<<endl;


for(int j = 0; j < total_points_cluster; j++)
{
cout << "Point " << clusters[i].getPoint(j).getID() + 1 << ": ";
for(int p = 0; p < total_values; p++)
cout << clusters[i].getPoint(j).getValue(p) << " ";


cout << endl;
}


cout << "Cluster values: ";

for(int j = 0; j < total_values; j++)
cout << clusters[i].getCentralValue(j) << " ";

cout << "\n\n";
}

cout<<"moving to rejection part"<<endl;

vector<int> threshold;
for(int i = 0; i < K; i++)
{
int total_points_cluster = clusters[i].getTotalPoints();
double sum = 0.0;
double maxm,minm;
double x,y,z;
x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);

minm=1000000000;
maxm=0;
for(int p = 0; p < total_points_cluster; p++)
{
sum+=pow(clusters[i].getPoint(p).getValue(0)-x,2);
sum+=pow(clusters[i].getPoint(p).getValue(1)-y,2);
sum+=pow(clusters[i].getPoint(p).getValue(2)-z,2);
sum=sqrt(sum);
if(sum<minm)
minm=sum;
if(sum>maxm)
maxm=sum;
	sum=0;
}
// 2 from here
double ans=abs(maxm-minm);
// ans=ans*2;

for(int p = 0; p < total_points_cluster; p++)
{
sum+=pow(clusters[i].getPoint(p).getValue(0)-x,2);
sum+=pow(clusters[i].getPoint(p).getValue(1)-y,2);
sum+=pow(clusters[i].getPoint(p).getValue(2)-z,2);
sum=sqrt(sum);
if(sum>ans)
{
points.erase(points.begin() + clusters[i].getPoint(p).getID());
}
}
}

cout<<"number of remaining points "<<points.size()<<endl;


/*
//  finding the weighted central value

double xcent=0,ycent=0,zcent=0;
int total_count=0;
for(int i = 0; i < K; i++)
{

int total_points_cluster = clusters[i].getTotalPoints();
total_count+=total_points_cluster;
double x,y,z;
x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);
xcent+=total_points_cluster*x;
ycent+=total_points_cluster*y;
zcent+=total_points_cluster*z;
}
xcent/=total_count;
ycent/=total_count;
zcent/=total_count;
std::vector<double> weighted_central_values;
weighted_central_values.push_back(xcent);
weighted_central_values.push_back(ycent);
weighted_central_values.push_back(zcent);

return weighted_central_values;

*/



queue<ClusterNode> pq;
int total_count=0;
double x,y,z;
for(int i = 0; i < K; i++)
{

int total_points_cluster = clusters[i].getTotalPoints();
total_count=total_points_cluster;

x=clusters[i].getCentralValue(0);
y=clusters[i].getCentralValue(1);
z=clusters[i].getCentralValue(2);
// cout<<x<<" "<<y<<" "<<z<<endl;
pq.push(ClusterNode(x,y,z,total_count));
}
return pq;
}
