#pragma once
//ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <ros/callback_queue.h>
//Include for messages
#include <pathplanning/markerpose.h>
#include <feature_detection/coord.h>

#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <math.h>
using namespace std;

struct coordMsg{
    int tx, ty, fx, fy;
    float theta;

};

struct waypoint{
    vector<double> wpX;
    vector<double> wpY;
};

class pathplanningControl
{
public:
    pathplanningControl();
    ~pathplanningControl();

    void readMsg(const feature_detection::coord::ConstPtr &msg);



private:
    coordMsg *current;



};
