class ClusterNode
{
public:
double xcent,ycent,zcent,count_total_points;
ClusterNode(double xcent,double ycent,double zcent,int count_total_points);
bool operator < (const ClusterNode& a);
};
