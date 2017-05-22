#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <math.h>
using namespace std;
//CHANGE ALL INT TO DOUBLE

class coordTrans
{
public:
    coordTrans();
    coordTrans(vector<int> afrobitOffset, const double atheta);
    ~coordTrans();
    void setParams(vector<int> afrobitOffset, const double atheta);
    vector<double> rotatePoint(vector<double> point);
    vector<double> translatePoint(vector<double> point);
    vector<double> calcPPrime(vector<double> point);
    int testThis();
    //Parameters for the frobits coordinate system
    vector<int> frobitOffset;
    double theta; //Running from -pi to +pi
    vector<vector<double>> rotationMatrix;
    vector<vector<double>> translateMatrix;
private:

    int objectId;

};
