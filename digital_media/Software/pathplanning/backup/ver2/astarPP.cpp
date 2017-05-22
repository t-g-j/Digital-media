//ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <ros/callback_queue.h>
//Include for messages
#include <pathplanning/markerpose.h>
#include <feature_detection/coord.h>
//Opencv includes
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
//Pathplanning includes
#include <pathplanning/astar.h>
#include <pathplanning/coordTrans.h>
//General CPP includes
#include <vector>
#include <string>

using namespace std;
using namespace cv;

//Global flags
bool pathplanningDone;
bool coordsObtained;
bool init;
//Global variables
coordTrans transObject;
int tx, ty, fx, fy;
float theta;
vector<double> wayPointX;
vector<double> wayPointY;

//Printing a cv::mat
void printMat(Mat aMat)
{
    for (int x = 0; x < aMat.rows; x++ )
    {
        for (int y = 0; y < aMat.cols; y++)
        {
           //if ( mImg.at<int>(x,y) < 0 )
             //  cout << "row: " << x << " col: " << y << " data: " <<
           cout << aMat.at<int>(x, y) << " ";
        }
            cout << endl;
    }
    cout << "Cols: " << aMat.cols << " Rows: " << aMat.rows << endl;
}


//Pixel is a struct in astar.h with F, G, H, x, y, (Pixel)value and (*PixelPointer) members, variables needed for A* algorithm
vector<vector<Pixel> > * intToPixel(Mat * aMat)
{
    //This method maps the values of a Mat object to a vec<vec<Pixel>>
    //vector<vector<int>> & inputRef = *input;
    int width = (*aMat).rows;
    int height = (*aMat).cols;
    vector<vector<Pixel> > * output = new vector<vector<Pixel> >; //Initiliasizes empty vector
    //vector<vector<Pixel>> & outputRef = *output;
    (*output).resize(width); //Resizes the width
    for (int w = 0; w < width; w++)
    {
        (*output)[w].resize(height); //Resizes the heigth;
    }

    for (int xi = 0; xi < width; xi++)
    {
        for (int yi = 0; yi < height; yi++)
        {

            (*output)[xi][yi].x = xi; //Setting x pixel values
            (*output)[xi][yi].y = yi; //Setting y pixel values
            (*output)[xi][yi].value = (*aMat).at<int>(xi, yi); //Setting the value from the input to the pixels
        }
    }
	
    return output;
}
//Callback for initializing A* and running it
void callback(const sensor_msgs::Image::ConstPtr &msg)
{

    if (coordsObtained)
    {
        cout << "Image subsriber entered" << endl;
        cv_bridge::CvImageConstPtr cv_ptr;
        try {

           cv_ptr = cv_bridge::toCvShare(msg, "8UC1");
           //cv::imshow("view", cv_bridge::toCvShare(msg, enc::MONO8));
           //cv::imshow("view", cv_ptr->image);
          // cv::waitKey(30);

        }
        //for (int i = 0; i < 5; i++)
         //   std::cout << msg->data[i] << " " ;
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s* to 'bgr8'.",msg->encoding.c_str());
        }

        //Mat CvImagePtr = toCvCopy(sensor_msgs::msg, "8UC1");
        //cv_bridge::CvImagePtr cv_ptr; //Declaring pointer to acces image
        //cv_ptr = cv_bridge::toCvCopy(msg, "8UC1"); //Copy content from message to pointer
        //cout << msg->data[0] << " image[0]" << endl;

        Mat_<int>  mImg;
        mImg = cv_ptr->image;
        Mat_<int> * mImgPtr = &mImg;
        //intToPixel(mImgPtr)
        aStar anObject(intToPixel(mImgPtr), fx, fy, tx, ty);
        //Getting the waypoints from the aStar object and transforming their coordinates into the frobits coordinate system
        vector<double> wpY = anObject.getYWP();
        vector<double> wpX = anObject.getXWP();
        for (int i = 0; i < wpY.size(); i++)
        {
            vector<double> point = {wpX[i], wpY[i], 0};
            //cout << "test: " << transPtr->testThis() << endl;
            point = transObject.calcPPrime(point);
            wayPointX.push_back(point[0]);
            wayPointY.push_back(point[1]);
            cout << wayPointX[i] << ", " << wayPointY[i] << endl;
        }
        coordsObtained = false;
    }

}

void callbackCoord(const feature_detection::coord::ConstPtr &msg)
{


    cout << "Coords entered: " << msg->tx << " " << msg->ty << "frobit: "<< msg->fx<< " " << msg->fy <<", " << msg->theta<< endl;
    tx = msg->tx;
    ty = msg->ty;
    fx = msg->fx;
    fy = msg->fy;
    vector<int> frobitOffset;
    frobitOffset.push_back(fx);
    frobitOffset.push_back(fy);
    //cout << "Pushed back coords" << endl;
    //coordTrans transObject(frobitOffset, msg->theta);
    transObject.setParams(frobitOffset, msg->theta);
    //cout << "Made transObject" << endl;
   // transPtr = & transObject;
    vector<double> point = {(double)tx, (double)ty, 0};
    //cout << "Made int to double cast" << endl;
    point = transObject.calcPPrime(point);
    //cout << "Calculated pprime" << endl;
    //cout << "Transformed point:" << point[0] << ", " << point[1] << endl;
    coordsObtained = true;
}

void objectInit()
{
    coordsObtained = false;
    pathplanningDone = false;
    init = true;
    cout <<"init true" << endl;
}

int main(int argc, char **argv)
{
    //ROS node init
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;
    ros::Subscriber sub =  n.subscribe("/output/image_treated", 1, callback);
    ros::Subscriber subCoord = n.subscribe("/output/target_frobit", 1, callbackCoord);

    ros::spin();

    //sub.shutdown();
    //cv::waitKey(0);
   // cv::destroyWindow("view");
    return 0;
}


