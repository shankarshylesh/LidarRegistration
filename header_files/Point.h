#include <bits/stdc++.h>
using namespace std;

class Point
{
private:
int id_point, id_cluster;
vector<double> values;
int total_values;

public:
Point(int id_point, vector<double>& values);
int getID();
void setCluster(int id_cluster);
int getCluster();
double getValue(int index);
int getTotalValues();
void addValue(double value);
};
