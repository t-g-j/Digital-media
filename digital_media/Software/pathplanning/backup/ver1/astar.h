#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
using namespace std;


struct Pixel {
    int x;
    int y;
    double F;
    double G;
    double H;
    Pixel * backPointer = NULL;
    double value;
};

#define shortestPath 5;
#define startPos 4;
#define desPos 6;
class aStar
{
public:
    aStar(vector<vector<Pixel> > * aRoadMap,int aStartX, int aStartY, int aEndX, int aEndY);
	~aStar();
    aStar(int test);
    void navigate(int startX, int startY);
    Pixel * getLowestF(); //Priority que?
	void removeFromOpenList(Pixel * toBeRemoved);
	void updateAdjecentSquares(Pixel * currentPixel);
    bool closedListContains(Pixel * toBeChecked); //Anden data struktur?
    bool openListContains(Pixel * toBeChecked); //Anden data struktur?
	int getClosedListSize();
    Pixel * getPointer(int x, int y);
	Pixel * getClosedListPointer(int x, int y);
	void findShortestPath();
	vector<int> getXList();
	vector<int> getYList();
    void calcScore(Pixel* aPixel);
    void calcG(Pixel* aPixel);
	bool withinMap(int x, int y);
	vector<vector<Pixel> > *  intToPixel(vector<vector<int> > * input);
	void printMap();
    vector<vector<Pixel> > * getMap();
    int width;
    int height;
    float widthFactor;
    float heightFactor;
private:
	//Data for A*
	vector<Pixel *> openList;
	vector<Pixel *> closedList;
	vector<Pixel *> adjecentSquares;
	vector<int> shortPX;
	vector<int> shortPY;
	//Maps and their properties
    vector<vector<Pixel> > * roadmap; //A* uses this map
    vector<vector<Pixel> > * inputMap; //This map is used for input and output
    int walkAble;
    int endX;
    int endY;
	int startX, startY;
    vector<string> wayPoints;

};

