#include<bits/stdc++.h>
using namespace std;


int simplex ( const vector <vector <double> > & A,  // constraint matrix
const vector <double>& B,            // right hand side
const vector <double>& C,            // objective vector
vector <double>& X,                  // unknowns
double & obj                          // objective value
);
