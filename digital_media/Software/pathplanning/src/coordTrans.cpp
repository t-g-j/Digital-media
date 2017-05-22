#include "../include/pathplanning/coordTrans.h"

coordTrans::coordTrans()
{

}


coordTrans::coordTrans(vector<int> afrobitOffset, const double atheta)
{
    frobitOffset = afrobitOffset;
    theta = atheta;
    rotationMatrix = { { atheta, atheta, 0 },{ atheta, atheta, 0 },{0, 0, 1} /* {(double)frobitOffset[0], (double)frobitOffset[1], 1} */ };
    translateMatrix = {{1, 0, (double)afrobitOffset[0]}, {0, 1, afrobitOffset[1]}, {0, 0, 1}};
}

coordTrans::~coordTrans()
{

}

void coordTrans::setParams(vector<int> afrobitOffset, const double atheta)
{
    frobitOffset = afrobitOffset;
    theta = atheta;
    rotationMatrix = { { atheta, atheta, 0 },{ atheta, atheta, 0 }, {0, 0, 1} };
    translateMatrix = {{1, 0, (double)afrobitOffset[0]}, {0, 1, afrobitOffset[1]}, {0, 0, 1}};
}



vector<double> coordTrans::translatePoint(vector<double> point)
{
    vector<double> output = { rotationMatrix[0][0] * point[0] + rotationMatrix[0][1] * point[1] + rotationMatrix[0][2] * point[2],
        rotationMatrix[1][0] * point[0] + rotationMatrix[1][1] * point[1] + rotationMatrix[1][2] * point[2],
        rotationMatrix[2][0] * point[0] + rotationMatrix[2][1] * point[1] + rotationMatrix[2][2] * point[2] };
    return output;
}

vector<double> coordTrans::rotatePoint(vector<double> point) //counter clockwise rotation!!
{
    vector<double> output = { cos(rotationMatrix[0][0]) * point[0] + -sin(rotationMatrix[0][1]) * point[1] + rotationMatrix[0][2] * point[2],
        -(sin(rotationMatrix[1][0]) * point[0] + cos(rotationMatrix[1][1]) * point[1] + rotationMatrix[1][2] * point[2]),
        rotationMatrix[2][0] * point[0] + rotationMatrix[2][1] * point[1] + rotationMatrix[2][2] * point[2] };
    return output;
}


vector<double> coordTrans::calcPPrime(vector<double> point)
{
    vector<double> output = rotatePoint(point);
    vector<double> tmp = rotatePoint({(double)frobitOffset[0], (double)frobitOffset[1], 1.});
     //cout << "Translated point" << output[0]  << ", " << output[1] << endl;
    output[0] -= tmp[0];
    output[1] -= tmp[1];
   //  output[1] = -output[1];
    return output;
}

//CalcPPRime before refining equation
/*



    vector<double> output = { cos(rotationMatrix[0][0]) * point[0] + -sin(rotationMatrix[0][1]) * point[1] + rotationMatrix[0][2] * point[2] +(double)frobitOffset[0],
        -(sin(rotationMatrix[1][0]) * point[0] + cos(rotationMatrix[1][1]) * point[1] + rotationMatrix[1][2] * point[2] + (double)frobitOffset[1]),
        rotationMatrix[2][0] * point[0] + rotationMatrix[2][1] * point[1] + rotationMatrix[2][2] * point[2] };
    return output;


vector<double> coordTrans::calcPPrime(vector<double> point)
{

    vector<double> output = { cos(rotationMatrix[0][0]) * point[0] + -sin(rotationMatrix[0][1]) * point[1] + rotationMatrix[0][2] * point[2],
        sin(rotationMatrix[1][0]) * point[0] + cos(rotationMatrix[1][1]) * point[1] + rotationMatrix[1][2] * point[2],
        rotationMatrix[2][0] * point[0] + rotationMatrix[2][1] * point[1] + rotationMatrix[2][2] * point[2] };
   // output[0] += (double)frobitOffset[0];
    //output[1] += (double)frobitOffset[1];
   // output[1] = -output[1];
    return output;
}

    */


int coordTrans::testThis()
{
    return frobitOffset[0];
}
