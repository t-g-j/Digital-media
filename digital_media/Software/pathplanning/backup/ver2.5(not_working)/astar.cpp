#include "../include/pathplanning/astar.h"
#include <string>

aStar::aStar(vector<vector<Pixel> > * aRoadMap, int aStartX, int aStartY, int aEndX, int aEndY)
{
    walkAble = 0;
    roadmap = aRoadMap;
	endX = aEndX;
    endY = aEndY;
	inputMap = aRoadMap;
	startX = aStartX;
	startY = aStartY;
    width = (*aRoadMap).size();
    cout << "width: " << width;
    height = (*aRoadMap)[0].size();
    cout << " height: " << height << endl;
    (*roadmap)[startX][startY].value = startPos;
    //(*roadmap)[endX][endY].value = desPos;
   // printMap();
    heightFactor = 10./height; //That is if the lenght and height of the picture is 10 and 14
    widthFactor = 14./width;
    navigate(startX, startY);
}


aStar::~aStar()
{
}

aStar::aStar(int test)
{

}

int aStar::addTwo(int * one, int * two)
{
	return (*one) + (*two);
}

void aStar::navigate(int startX, int startY)
{
    Pixel * start = new Pixel; 
    start->x = startX;
    start->y = startY;
	start->G = 0;
    start->F = start->G +  abs(endX - start->x) + abs(endY - start->y);
	openList.push_back(start);
	Pixel * current; //Pointer used to point to the current pixel
	while (!openList.empty()) //If the openList is empty, all possible paths are explored
	{
		current = getLowestF(); //Update current with the pixel with the lowest Fscore in the openlist
		closedList.push_back(current); //We dont consider the current pixel for movement anymore
		removeFromOpenList(current);
        if (current->x == endX && current->y == endY) //If we're at the end pixel, aStar is completed and we dont want to run anymore
        {

            break;
        }
		updateAdjecentSquares(current); //Adjecentsquares updated with the squares adjecent to the current one
		for (int i = 0; i < adjecentSquares.size(); i++)
		{

            if (closedListContains(adjecentSquares[i])) //If it's in the closed list, we dont elaborate further on this pixel
				continue;
			if (!openListContains(adjecentSquares[i])) //If the adjecent pixel isn't in the openlist
			{
                adjecentSquares[i]->backPointer = current; //assign back pointer
                calcScore(adjecentSquares[i]); //calculate score
				openList.push_back(adjecentSquares[i]); //Pushed it back on the open list
            }
            /*else //If it is in the open list, consider if using the current pixel as parent is a shorter path //IMplement this when everything else works
			{
				
                Pixel * tmp = getPointer(adjecentSquares[i]->x, adjecentSquares[i]->y); //MAYBE REWRITE THIS???
		
				
                double val = tmp->G;
				
				double val2 = current->G + sqrt(pow((abs(current->x - adjecentSquares[i]->x)), 2) + pow(abs(current->y - adjecentSquares[i]->y), 2));
				if (val2 < val) //If the score is lower with the current pixel as parent
					tmp->backPointer = current; //Asign the parent as the pixel it originated from
            }*/
        }
	}
	findShortestPath(); //Start in the end pixel and follow backpointers all the way to start in order to find the shortest path
}


Pixel * aStar::getLowestF()
{
	Pixel * rtn = NULL;
	double tmp = 100000; //This value is used to compare, making sure it's bigger than the highst possible value
	for (int i = 0; i < openList.size(); i++)
	{
        if (openList[i]->F <= tmp)
		{
			rtn = openList[i];
            tmp = openList[i]->F;
		}
	}
	return rtn;
}

bool aStar::openListContains(Pixel * toBeChecked)
{
    int tmpX = toBeChecked->x; //Use the x and y values as identifiers for the pixel pointer passed
    int tmpY = toBeChecked->y;
	for (int i = 0; i < openList.size(); i++)
	{
        if (tmpX == openList[i]->x && tmpY == openList[i]->y)
			return true;
	}
	return false;
}

int aStar::getClosedListSize()
{
	return closedList.size();
}

Pixel * aStar::getPointer(int x, int y)
{
	Pixel * ptr = NULL;
	for (int i = 0; i < openList.size(); i++)
	{
		if (openList[i]->x == x && openList[i]->y == y) //Using x and y values as identifiers
		{
			ptr = openList[i];
		}
	}
	
	return ptr;
}

Pixel * aStar::getClosedListPointer(int x, int y)
{
	Pixel * ptr = NULL;
	for (int i = 0; i < closedList.size(); i++)
	{
        if (closedList[i]->x == x && closedList[i]->y == y) //Using x and y values as identifiers
			ptr = closedList[i];
	}
	return ptr;
}

void aStar::findShortestPath()
{
    cout << widthFactor << " " << heightFactor << " Debug for map factors" << endl;
    string tmpString;
    string waypoint;
    int deltaX, deltaY;
	Pixel * current = getClosedListPointer(endX, endY); //Starting at the destination
    deltaX = current->x - current->backPointer->x;
    deltaY = current->y - current->backPointer->y;
    wayPointsX.push_back(current->x);
    wayPointsY.push_back(current->y);
    while (current->backPointer != NULL) //Start will have a nullptr, so stop when it's reached
	{
        shortPX.push_back(current->x); //Store x and y values
        shortPY.push_back(current->y);
       if(current->x - current->backPointer->x  != deltaX|| current->y - current->backPointer->y != deltaY)
       {
           wayPointsX.push_back(current->x);
           wayPointsY.push_back(current->y);
           deltaX = current->x - current->backPointer->x;
           deltaY = current->y - current->backPointer->y;
       }
        current = current->backPointer; //Work the way back to the start pixel
	}
    wayPointsX.push_back(current->x);
    wayPointsY.push_back(current->y);
	for (int i = 0; i < shortPX.size(); i++)
	{
        (*inputMap)[shortPX[i]][shortPY[i]].value = shortestPath;
	}
    (*inputMap)[startX][startY].value = startPos;
    (*inputMap)[endX][endY].value = desPos;

    //printMap();
}

vector<double> aStar::getXWP()
{
    return wayPointsX;
}

vector<double> aStar::getYWP()
{
    return wayPointsY;
}

vector<int> aStar::getXList()
{
	return shortPX;
}

vector<int> aStar::getYList()
{
	return shortPY;
}

bool aStar::closedListContains(Pixel * toBeChecked)
{
    int tmpX = toBeChecked->x; //Using x and y as identifiers
    int tmpY = toBeChecked->y;
	for (int i = 0; i < closedList.size(); i++)
	{
        if (tmpX == closedList[i]->x && tmpY == closedList[i]->y)
			return true;
	}
	return false;
}

void aStar::removeFromOpenList(Pixel * toBeRemoved)
{
    int tmpX = toBeRemoved->x;  //Using x and y as identifiers
    int tmpY = toBeRemoved->y;
	for (int i = 0; i < openList.size(); i++)
	{
        if (openList[i]->x == tmpX && openList[i]->y == tmpY)
		{
			openList.erase(openList.begin() + i); //Erase the position the pixel is located at
			break;
		}
	}
}

void aStar::updateAdjecentSquares(Pixel * currentPixel)
{
	adjecentSquares.clear();
    for (int i = -1; i<2; i++)
        for(int j = -1; j<2; j++)
    {
			
			if (withinMap(currentPixel->x + j, currentPixel->y + i))
			{
				
				if (!(j == 0 && i == 0))
				{
                if ((*roadmap)[currentPixel->x+j][currentPixel->y+i].value == 0)
				{
					Pixel * aPixel = new Pixel;
					aPixel->x = currentPixel->x+j;
					aPixel->y = currentPixel->y+i;
					adjecentSquares.push_back(aPixel);
				}
				}
			}
    }

  
	/*
     //Clear the vector making sure no leftovers from last pixel is in it
    if (roadmap[currentPixel->x - 1][currentPixel->y].value == walkAble) //If the left pixel has the value 100 it's within the roadmap, and can be considered for the path
    {
        Pixel * left = new Pixel;
        left->x = currentPixel->x - 1;
        left->y = currentPixel->y;
        adjecentSquares.push_back(left);

    }

    if (roadmap[currentPixel->x + 1][currentPixel->y].value == walkAble) //Same as above, just adjecent in another direction
	{
        Pixel * right = new Pixel;
        right->x = currentPixel->x + 1;
        right->y = currentPixel->y;
		adjecentSquares.push_back(right);
	}

    if (roadmap[currentPixel->x][currentPixel->y - 1].value == walkAble)
	{
        Pixel * down = new Pixel;
        down->x = currentPixel->x;
        down->y = currentPixel->y - 1;
		adjecentSquares.push_back(down);

	}

    if (roadmap[currentPixel->x][currentPixel->y + 1].value == walkAble)
	{
        Pixel * up = new Pixel;
        left->x = currentPixel->x;
        left->y = currentPixel->y+1;
		adjecentSquares.push_back(up);
	}

    if (roadmap[currentPixel->x - 1][currentPixel->y - 1].value == walkAble)
	{
        Pixel * leftDown = new Pixel(currentPixel->x - 1, currentPixel->y - 1);
		adjecentSquares.push_back(leftDown);
	}

    if (roadmap[currentPixel->x - 1][currentPixel->y + 1]->value == walkableV)
	{
        Pixel * leftUp = new Pixel(currentPixel->x - 1, currentPixel->y + 1);
		adjecentSquares.push_back(leftUp);
	}

    if (roadmap[currentPixel->x + 1][currentPixel->y + 1]->value == walkableV)
	{
        Pixel * rightUp = new Pixel(currentPixel->x + 1, currentPixel->y + 1);
		adjecentSquares.push_back(rightUp);
	}

    if (roadmap[currentPixel->x + 1][currentPixel->y - 1]->value == walkableV)
	{
        Pixel * rightDown = new Pixel(currentPixel->x + 1, currentPixel->y - 1);
		adjecentSquares.push_back(rightDown);
    } */
 }

 void aStar::calcScore(Pixel* aPixel)
    {
	 if (aPixel->backPointer != NULL)
		 aPixel->G = aPixel->backPointer->G + sqrt(pow((abs(aPixel->x - aPixel->backPointer->x)), 2) + pow(abs(aPixel->y - aPixel->backPointer->y), 2));
	 else
	 {
         cout << "First pixel G value = 0" << endl;
		aPixel->G = 0;
	 }
     aPixel->H = abs(endX - aPixel->x) + abs(endY - aPixel->y);
     //cout << "H value " << aPixel->H << " G value: " << aPixel->G << endl;
     //aPixel->H = sqrt(pow(abs(endX - aPixel->x),2) + pow(abs(endY - aPixel->y), 2));  //Bird flight distance
     aPixel->F = aPixel->H + aPixel->G;
    }

 void aStar::calcG(Pixel * aPixel) //Calc G, is going to be one for a horizontal or vertical movement, and 1.44 something for a diagonal movement
 {
     aPixel->G= sqrt(pow((abs(aPixel->x - aPixel->backPointer->x)), 2) + pow(abs(aPixel->y - aPixel->backPointer->y), 2));
 }

 bool aStar::withinMap(int x, int y)
 {
	 if (x > -1 && y >-1 && y < height && x < width)
		 return true;
	 return false;
 }

 vector<vector<Pixel> > * aStar::intToPixel(vector<vector<int> > * input)
 {
	 //This method maps the values of a vec<vec<int>> to vec<vec<pixel>>
	 //vector<vector<int>> & inputRef = *input;
	 width = input->size();
	 height = (*input)[0].size();
     vector<vector<Pixel> > * output = new vector<vector<Pixel> >; //Initiliasizes empty vector
	 //vector<vector<Pixel>> & outputRef = *output;
	 (*output).resize(width); //Resizes the width
	 for (int w = 0; w < width; w++)
	 {
		 (*output)[w].resize(height); //Resizes the heigth;
	 }

	 for (int yi = 0; yi < height; yi++)
	 {
		 for (int xi = 0; xi < width; xi++)
		 {

			 (*output)[xi][yi].x = xi; //Setting x pixel values
			 (*output)[xi][yi].y = yi; //Setting y pixel values
			 (*output)[xi][yi].value = (*input)[xi][yi]; //Setting the value from the input to the pixels
		 }
	 }

	 return output;
 }

 void aStar::printMap()
 {
	 cout << "Printing map with width: " << width << " and height: " << height << endl;
	 for (int yi = 0; yi < height; yi++)
	 {
		 for (int xi = 0; xi < width; xi++)
		 {
             cout << (*roadmap)[xi][yi].value << " ";
		 }
		 cout << endl;
		
	 }
 }

 vector<vector<Pixel> >* aStar::getMap()
 {
	 return inputMap;
 }


