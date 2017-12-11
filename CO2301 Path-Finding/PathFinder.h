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

enum dirrection
{
	North = 0, East, South, West
};

struct coords
{
	pair<int, int> location;	// x, y coordinats on the map
	int manhattanDist;			// the manhattan distance to the goal
	int runningDist;			// distance from the starting point

	coords* parent;				// 
};

class CPathFinder
{
private:
	vector<vector<int>> mMap;	// the current map
	pair<int, int> mStart;		// starting point
	pair<int, int> mEnd;		// ending point / goal

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

