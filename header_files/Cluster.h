#include <bits/stdc++.h>
#include "Point.h"

using namespace std;

class Cluster
{
private:
	int id_cluster;
	vector<double> central_values;
	vector<Point> points;

public:
	Cluster(int id_cluster, Point point);

	void addPoint(Point point);


	bool removePoint(int id_point);


	double getCentralValue(int index);

	void setCentralValue(int index, double value);


	Point getPoint(int index);

	int getTotalPoints();

	int getID();
};
