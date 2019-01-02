/*

Copyright © 2018 by Adarsh Sunilkumar Aby Thomas Shankar Shylesh
All rights reserved. This code or any portion thereof
may not be reproduced or used in any manner whatsoever
without the express written permission of the owners,
except in the case of certain noncommercial uses permitted
by copyright law.


*/

#include <bits/stdc++.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "KMeans.h"
#include <cmath>
#include <cstdlib>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include "../header_files/Simplex.h"
#include "../header_files/CompareFunction.h"

int user_data;

#define globalmaximum 100000000


struct CompareCount {
bool operator()(ClusterNode const& p1,ClusterNode const& p2)
{
// return "true" if "p1" is ordered
// before "p2", 
return p1.count_total_points < p2.count_total_points;
}
};



void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
viewer.setBackgroundColor (1.0, 0.5, 1.0);
pcl::PointXYZ o;
o.x = 1.0;
o.y = 0;
o.z = 0;
std::cout << "i only run once" << std::endl;

}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
static unsigned count = 0;
std::stringstream ss;
ss << "Once per viewer loop: " << count++;
viewer.removeShape ("text", 0);
viewer.addText (ss.str(), 200, 300, "text", 0);

//FIXME: possible race condition here:
user_data++;
}


int main (int argc, char** argv)
{

long long int i,j,iter,minimum;
long long int globalTotalPoint=0;


// providing seed for random number genrator

srand (time(NULL));

int total_points, total_attributes, K, max_iterations;
int numberOfInputClouds=6;
int cloudPresent[numberOfInputClouds];
int numberOfCloudsProcessed=0;

for(i=0;i<numberOfInputClouds;i++)
{
cloudPresent[i]=1;
}

cloudPresent[0]=0;


while((numberOfCloudsProcessed+1)<numberOfInputClouds)
{

double compareArray[numberOfInputClouds];
for(i=0;i<numberOfInputClouds;i++)
{
compareArray[i]=globalmaximum;
}  

pcl::PointCloud<pcl::PointXYZ>::Ptr parentCloud (new pcl::PointCloud<pcl::PointXYZ>);

string parentFileName="point"+std::to_string(0)+".pcd";

// Fill in the cloud data
pcl::PCDReader reader;
reader.read<pcl::PointXYZ> (parentFileName, *parentCloud);

std::cerr << "Cloud before filtering: " << std::endl;
std::cerr << *parentCloud << std::endl;

for(iter=1;iter<numberOfInputClouds;iter++)
{
if(cloudPresent[iter]==1)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr childCloud (new pcl::PointCloud<pcl::PointXYZ>);

string childFileName="point"+std::to_string(iter)+".pcd";

// Fill in the cloud data
pcl::PCDReader reader;
reader.read<pcl::PointXYZ> (childFileName, *childCloud);

std::cerr << "Cloud before filtering: " << std::endl;
std::cerr << *childCloud << std::endl;

compareArray[iter]=compareFunction(parentCloud,childCloud);
} 
}

minimum=globalmaximum;
int answer=1;

for(iter=1;iter<numberOfInputClouds;iter++)
{
if(compareArray[iter]<minimum)
{
minimum=compareArray[iter];
answer=iter;
}
}
// answer=numberOfCloudsProcessed+1;
cloudPresent[answer]=0;

total_attributes=3;
total_points=parentCloud->points.size ();
K=6;
max_iterations=10;
vector<Point> pointcollection;

for( iter = 0; iter < total_points; iter++)
{
vector<double> values;
values.push_back(parentCloud->points[iter].x);
values.push_back(parentCloud->points[iter].y);
values.push_back(parentCloud->points[iter].z);

Point p(iter, values);
pointcollection.push_back(p);
}


priority_queue<ClusterNode, vector<ClusterNode>, CompareCount> Q;
queue<ClusterNode> pq;

KMeans kmeans(K, total_points, total_attributes, max_iterations);
pq = kmeans.run(pointcollection,parentCloud);

while (!pq.empty()) {
ClusterNode cn = pq.front();
pq.pop();
Q.push(cn);
// cout << cn.xcent << " "<<cn.ycent<<" "<<cn.zcent<<" "<< cn.count_total_points << "\n";
}

// writing original points
std::vector<Point> updatedpoints = pointcollection;
globalTotalPoint=updatedpoints.size();
pcl::PointCloud<pcl::PointXYZ> updatedcloud;

// Fill in the cloud data
updatedcloud.width    = updatedpoints.size();
updatedcloud.height   = 1;
updatedcloud.is_dense = false;
updatedcloud.points.resize (updatedcloud.width * updatedcloud.height);

for (size_t jj = 0; jj < updatedcloud.points.size (); ++jj)
{
updatedcloud.points[jj].x = updatedpoints[jj].getValue(0);
updatedcloud.points[jj].y = updatedpoints[jj].getValue(1);
updatedcloud.points[jj].z = updatedpoints[jj].getValue(2);
}
pcl::io::savePCDFileASCII ("output0.pcd", updatedcloud);

////////////// start reading files and processing ////////////////////
priority_queue<ClusterNode, vector<ClusterNode>, CompareCount> tempQ;
tempQ=Q;

string filename="point"+std::to_string(answer)+".pcd";
pcl::PointCloud<pcl::PointXYZ>::Ptr clouder (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtereder (new pcl::PointCloud<pcl::PointXYZ>);

// Fill in the cloud data
pcl::PCDReader readerer;
readerer.read<pcl::PointXYZ> (filename, *clouder);
std::cerr << "Cloud before filtering: " << std::endl;
std::cerr << *clouder << std::endl;

int total_pointser;
total_pointser=clouder->points.size ();
K=6;
max_iterations=10;
vector<Point> pointcollectioner;
for( iter = 0; iter < total_pointser; iter++)
{
vector<double> values;
values.push_back(clouder->points[iter].x);
values.push_back(clouder->points[iter].y);
values.push_back(clouder->points[iter].z);

Point p(iter, values);
pointcollectioner.push_back(p);

}
priority_queue<ClusterNode, vector<ClusterNode>, CompareCount> Qw;
queue<ClusterNode> pqer;

KMeans kmeanser(K, total_pointser, total_attributes, max_iterations);
pqer = kmeanser.run(pointcollectioner,clouder);

while (!pqer.empty()) {
ClusterNode cn = pqer.front();
pqer.pop();
Qw.push(cn);
// cout << cn.xcent << " "<<cn.ycent<<" "<<cn.zcent<<" "<< cn.count_total_points << "\n";
}

////////////////////////// processing /////////////////////////////////

int m, n;

m=9;       // How many constraints ?
n=15;       // How many variables ?
vector <vector <double> > A (m, vector<double>(n,0.0));
vector <double> B (m,0.0), C(n,0.0);
// "Enter the  coefficients of the "  n " variables in the left hand side of equality constraints
i=0;
int counter=6;
while(i<m)
{
ClusterNode cn = Qw.top();
Qw.pop();
ClusterNode cnActual=tempQ.top();
tempQ.pop();

A[i][0]=0; A[i][1]=0; A[i][2]=cn.xcent; A[i][3]=cn.zcent; A[i][4]=cn.xcent; A[i][5]=-cn.ycent; A[i][counter]=1;
// Enter the constants on the right side of equality constraints
B[i]=cnActual.xcent-cn.xcent;
i++;
counter++;
A[i][0]=cn.ycent; A[i][1]=-cn.zcent; A[i][2]=0; A[i][3]=0; A[i][4]=cn.ycent; A[i][5]=cn.xcent;  A[i][counter]=1;
B[i]=cnActual.ycent-cn.ycent;
i++;
counter++;
A[i][0]=cn.zcent; A[i][1]=cn.ycent; A[i][2]=cn.zcent; A[i][3]=-cn.xcent; A[i][4]=0; A[i][5]=0;  A[i][counter]=1;
B[i]=cnActual.zcent-cn.zcent;
i++;
counter++;
}

//  Enter the coefficients of the variables of the objective function
for (int i=0; i<n; i++) C[i]=1;
vector <double> X;
double obj;
cout << simplex (A, B, C, X, obj) << endl;
cout << "\nOptimal objective value = " << obj << endl;
cout << "\nOptimal solution: ";
for (int i=0; i<n; i++)
  cout << X[i] << "\t";
cout << endl;

/////updating the points using rotational and transilational matrix //////////////////////////

double a,b,c,d,e,f,t,r,s;
a=X[0]; b=X[1]; c=X[2]; d=X[3]; e=X[4]; f=X[5]; t=(X[6]+X[9]+X[12])/3;
r=(X[7]+X[10]+X[13])/3; s=(X[8]+X[11]+X[14])/3;

std::vector<Point> updatedpointser = pointcollectioner;
globalTotalPoint+=updatedpointser.size();
// writing the updated points to files
pcl::PointCloud<pcl::PointXYZ> updatedclouder;

   // Fill in the cloud data
   updatedclouder.width    = updatedpointser.size();
   updatedclouder.height   = 1;
   updatedclouder.is_dense = false;
   updatedclouder.points.resize (updatedclouder.width * updatedclouder.height);

for (size_t g = 0; g < updatedpointser.size (); ++g)
{
 double x,y,z;
 x = updatedpointser[g].getValue(0);
 y = updatedpointser[g].getValue(1);
 z = updatedpointser[g].getValue(2);
  updatedclouder.points[g].x =(1+c+e)*x+d*z-f*y+t;
  updatedclouder.points[g].y =f*x+(1+a+e)*y-b*z+r;
  updatedclouder.points[g].z =-d*x+b*y+(1+a+c)*z+s;
}

/////////////////////////////////////////end of processing/////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZ> cloud_a;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
string finalFile="output"+std::to_string(answer)+".pcd";
pcl::io::savePCDFileASCII (finalFile, updatedclouder);
reader.read<pcl::PointXYZ> (finalFile, *cloud_filtered);
reader.read<pcl::PointXYZ> ("point0.pcd", cloud_a);
cloud_a+=*cloud_filtered;
pcl::io::savePCDFileASCII ("point0.pcd", cloud_a);     
numberOfCloudsProcessed++;
}


/////////////////////////////////////////////////////////////////////

// Now reading from all files and showing the final image /////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr finalClouder (new pcl::PointCloud<pcl::PointXYZ>);

/*
// Fill in the cloud data
finalClouder.width    = globalTotalPoint;
finalClouder.height   = 1;
finalClouder.is_dense = false;
finalClouder.points.resize (finalClouder.width * finalClouder.height);
*/
pcl::PCDReader reader;
reader.read<pcl::PointXYZ> ("point0.pcd", *finalClouder);
////////////////////////////////////////////////////////////////

pcl::visualization::CloudViewer viewer("Cloud Viewer");


//blocks until the cloud is actually rendered
viewer.showCloud(finalClouder);

//use the following functions to get access to the underlying more advanced/powerful
//PCLVisualizer

//This will only get called once
viewer.runOnVisualizationThreadOnce (viewerOneOff);


//This will get called once per visualization iteration
viewer.runOnVisualizationThread (viewerPsycho);

// for infinite loop
while(1)
{

}


return (0);
}