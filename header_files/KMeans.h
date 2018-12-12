#include <bits/stdc++.h>
#include "Cluster.h"
#include "ClusterNode.h"

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
	queue<ClusterNode> run(vector<Point> & points);
  queue<ClusterNode> runEnhanced(vector<Point> & points);
};
