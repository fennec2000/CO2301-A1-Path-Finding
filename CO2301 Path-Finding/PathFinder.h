#pragma once
#include <vector>		// vector
#include <utility>		// pair
#include <fstream>		// ifstream
#include <string>		// string getline
#include <sstream>		// stringstream
#include <iostream>		// cout
#include <iterator>		// istream_iterator
#include <memory>		// unique_ptr
#include <deque>		// deque
#include <algorithm>	// sort
using namespace std;

#define DEBUG

enum dirrection
{
	North = 0, East, South, West, NumberOfDirections
};

struct coords
{
	pair<int, int> location;	// x, y coordinats on the map
	int manhattanDist;			// the manhattan distance to the goal
	int runningDist;			// distance from the starting point

	coords* parent;				// previous node
};

class CPathFinder
{
private:
	vector<vector<int>> mMap;	// the current map
	pair<int, int> mStart;		// starting point
	pair<int, int> mEnd;		// ending point / goal
	pair<int, int> mMapSize;	// the size of the map as a rectangle

	// private func
	void LoadCoords(string givenMapName);	// Loads coords
	void LoadMap(string givenMapName);		// Loads map
	void SolveAStar();						// Solve the current map
	int CalcManDist(pair<int, int> Loc);	// Calc manhattan distance
	int CalcRunDist(unique_ptr <coords>& givenPoint);	// Calc current distance from start
	void DisplayMap();									// used to display the map to console for debugging
	bool Find(deque<unique_ptr<coords>>& myList, pair<int, int> Loc);	// find is location is already in list
	void SwapFirstWithCheck(deque<unique_ptr<coords>>& myList, unique_ptr <coords>& givenPoint);	// swap the given point with the first match in the list
	bool CompareCoords(unique_ptr<coords>& lhs, unique_ptr<coords>& rhs);	// compare two coords


public:
	CPathFinder();		// constructor
	~CPathFinder();		// deconstructor

	void Load(string mapName);	// Loads the named map
};

