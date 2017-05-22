#include "../include/pathplanning/pathplanningControl.h"

pathplanningControl::pathplanningControl()
{
    current = nullptr;
}

pathplanningControl::~pathplanningControl()
{

}


void pathplanningControl::readMsg(coordMsg * msgRecieved)
{
    current = msgRecieved;
}
