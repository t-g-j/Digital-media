#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>

#include <pathplanning/markerpose.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/Image>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pathplanning/astar.h>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

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

void callback(const sensor_msgs::Image::ConstPtr &msg)
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
        aStar anObject(intToPixel(mImgPtr), 10, 10, 20, 25);
        //anObject.printMap();
        vector<int> xlist = anObject.getXList();
        cout << xlist.size() << endl;
        vector<int> ylist = anObject.getYList();
        for(int i = 0; i < xlist.size(); i++)
            cout << xlist[i] << ", " << ylist[i] << endl;



    /*
     vector<int> getXList();
    vector<int> getYList();
*/

    //printMat(mImg);
    /*for (int x = 0; x < mImg.rows; x++ )
    {
        for (int y = 0; y < mImg.cols; y++)
        {
           //if ( mImg.at<int>(x,y) < 0 )
             //  cout << "row: " << x << " col: " << y << " data: " <<
           cout << mImg.at<int>(x, y) << " ";
        }
            cout << endl;
    }
    */
    //cout << "Cols: " << mImg.cols << " Rows: " << mImg.rows << endl;


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;

    ros::Subscriber sub =  n.subscribe("/output/image_treated", 1, callback);
    ros::spin();
    //cv::waitKey(0);
   // cv::destroyWindow("view");
    return 0;
}


