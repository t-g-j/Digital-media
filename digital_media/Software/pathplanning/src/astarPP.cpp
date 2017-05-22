//ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <ros/callback_queue.h>
//Include for messages
#include <markerlocator/markerpose.h>
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
#include <cmath>

using namespace std;
using namespace cv;

//Global flags
bool pathplanningDone = false;
bool coordsObtained;
bool init;
bool pixel = false;
bool wpPublished = false;

//Global variables
coordTrans transObject;
float tx, ty, fx, fy;
float theta;
vector<double> wayPointX;
vector<double> wayPointY;
double trueWidth = 2.495;
double pixelTM = 0.;
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

    for (double xi = 0; xi < width; xi++)
    {
        for (double yi = 0; yi < height; yi++)
        {

            (*output)[xi][yi].x = xi; //Setting x pixel values
            (*output)[xi][yi].y = yi; //Setting y pixel values
            (*output)[xi][yi].value = (*aMat).at<int>(xi, yi); //Setting the value from the input to the pixels

                //cout << "REEEED LOADED INTO MAP " << (*aMat).at<int>(xi, yi) << endl;
        }
    }
	
    return output;
}
//Callback for initializing A* and running it
void imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    wayPointX.clear();
    wayPointY.clear();
    if (coordsObtained)
    {
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
        aStar anObject(intToPixel(mImgPtr), fx, fy, tx, ty, trueWidth);
        //Getting the waypoints from the aStar object and transforming their coordinates into the frobits coordinate system
        vector<double> wpY = anObject.getYWP();
        vector<double> wpX = anObject.getXWP();
        pixelTM = anObject.pixelsToMeters;
        for (int i = wpY.size()-1; i >= 0; i--)
        {
            vector<double> point = {wpX[i], wpY[i], 1};
            point = transObject.calcPPrime(point);
            wayPointX.push_back(point[0]*anObject.pixelsToMeters);
            wayPointY.push_back(point[1]*anObject.pixelsToMeters);


        }

        }
        for (int i = 0; i < wayPointX.size(); i++)
             cout << "Waypoints in meters:: " << wayPointX[i]<< ", " << wayPointY[i]<<" hypotenuse: " << sqrt(wayPointX[i]*wayPointX[i]+wayPointY[i]*wayPointY[i]) <<" i: " << i << endl;
        coordsObtained = false;
        pathplanningDone = true;


}

void coordCallback(const feature_detection::coord::ConstPtr &msg)
{


    tx = msg->tx;
    ty = msg->ty;
    fx = msg->fx;
    fy = msg->fy;
    //if (msg->theta < 0)
    //    theta = 3.14159265359+msg->theta + 3.14159265359;
    //else
        theta = msg->theta;
    cout << "This is theta: " << msg->theta << endl;
    vector<int> frobitOffset;
    frobitOffset.push_back(fx);
    frobitOffset.push_back(fy);
    frobitOffset.push_back(1);
    //cout << "Pushed back coords" << endl;
    //coordTrans transObject(frobitOffset, msg->theta);
    transObject.setParams(frobitOffset, theta);
    //cout << "Made transObject" << endl;
   // transPtr = & transObject;
    vector<double> point = {(double)tx, (double)ty, 1};
    //cout << "Made int to double cast" << endl;
    point = transObject.calcPPrime(point);

    //Next line goes from pixels to meters
    //cout << "Transformed point:" << point[0]*pixelTM << ", " << point[1]*pixelTM << endl;
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
    ros::Subscriber sub =  n.subscribe("/output/image_treated", 1, imageCallback);
    ros::Subscriber subCoord = n.subscribe("/output/target_frobit", 1, coordCallback);
    ros::Publisher wpPub = n.advertise<feature_detection::coord>("/waypoints", 1);
    cout << "About to check out dat publish" << endl;
    while (ros::ok())
    {
        if (!wpPublished && pathplanningDone)
        {
            for (int i = 0; i<wayPointX.size(); i++)
            {
                cout <<"entered the for loop" << endl;
                feature_detection::coord pubMsg;
                pubMsg.tx = wayPointX[i];
                pubMsg.ty = wayPointY[i];
                wpPub.publish(pubMsg);
            }
        wpPublished = true;
        }
        ros::spinOnce();
    }

    //sub.shutdown();
    //cv::waitKey(0);
   // cv::destroyWindow("view");
    return 0;
}


