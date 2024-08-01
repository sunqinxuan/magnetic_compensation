#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <complex>

using namespace std;

#define PI 3.14159

vector<double> ComputeDenCoeffs(int FilterOrder, double Lcutoff,
                                double Ucutoff);

vector<double> TrinomialMultiply(int FilterOrder, vector<double> b,
                                 vector<double> c);

vector<double> ComputeNumCoeffs(int FilterOrder, double Lcutoff, double Ucutoff,
                                vector<double> DenC);

vector<double> ComputeLP(int FilterOrder);

vector<double> ComputeHP(int FilterOrder);

// vector<double> filter(int ord, vector<double> a, vector<double> b, int np,
// vector<double> x);

vector<double> filter(vector<double> x, vector<double> coeff_b,
                      vector<double> coeff_a);
