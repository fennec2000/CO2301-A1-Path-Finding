#pragma once
#include <vector>	// vector
#include <utility>	// pair
#include <fstream>	// ifstream
#include <string>	// string getline
#include <sstream>	// stringstream
#include <iostream> // cout
#include <iterator> // istream_iterator
#include <memory>	// unique_ptr
#include <deque>	// deque
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

public:
	CPathFinder();		// constructor
	~CPathFinder();		// deconstructor

	void Load(string mapName);	// Loads the named map

	void DisplayMap();	// used to display the map to console for debugging
};

