#pragma once
#include <vector>		// vector
#include <utility>		// pair
#include <fstream>		// ifstream
#include <string>		// string getline
#include <sstream>		// stringstream
#include <iostream>		// std::cout
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
	vector<vector<int>> mMap;		// the current map
	pair<int, int> mStart;			// starting point
	pair<int, int> mEnd;			// ending point / goal
	pair<int, int> mMapSize;		// the size of the map as a rectangle
	vector<pair<int, int>> mPath;	// path from start to end

	int NumOfSorts;			// counts the number of sorts
	int NumOfNodesVisited;	// counts the number of nodes visited
	int NumOfNodesSeen;		// counts the number of nodes seen
	string fileName;		// name of the file // the prefix e.g. d - dMaps.txt, dCoords.txt, dOutput.txt, dStats.txt

	// private func
	void LoadCoords(string givenMapName);	// Loads coords
	void LoadMap(string givenMapName);		// Loads map
	void SolveAStar();						// Solve the current map
	int CalcManDist(pair<int, int> Loc);	// Calc manhattan distance
	int CalcRunDist(unique_ptr <coords>& givenPoint);	// Calc current distance from start
	void DisplayMap();									// used to display the map to console for debugging
	bool Find(deque<unique_ptr<coords>>& myList, pair<int, int> Loc);	// find is location is already in list
	void SwapFirstWithCheck(deque<unique_ptr<coords>>& myList, unique_ptr <coords>& givenPoint);	// swap the given point with the first match in the list
	void DisplayList(deque<unique_ptr<coords>>& myList);	// Display the given list in the console
	void ReturnPath(unique_ptr <coords>& givenPoint);		// Puts the path from givenPoint to start into mPath
	void CPathFinder::WriteResult();		// Write the restlts to files xOutput.txt and xStats.txt x = filename

public:
	CPathFinder(string givenFileName);		// constructor
	~CPathFinder();		// deconstructor

	void Load(string mapName);	// Loads the named map

	pair<int, int> GetMapSize() { return mMapSize; };	// returns the map size
	vector<vector<int>> GetMap() {};
};

