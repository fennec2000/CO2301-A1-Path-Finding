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
#include <TL-Engine.h>	// tl engine
#include <Windows.h>	// file system
using namespace std;
using namespace tle;

#define DEBUG

enum cubeTypes { wall, clear, wood, water, start, end, numOfCubeTypes };
enum cubeStatus { unknown, seen, visited, path, numOfStatus };
enum dirrection { North, East, South, West, NumberOfDirections };

// class global const
const float waitTimer = 0.33f;	// time between movement

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
	I3DEngine* mpMyEngine;			// pointer to the TL engine

	int NumOfSorts;			// counts the number of sorts
	int NumOfNodesVisited;	// counts the number of nodes visited
	int NumOfNodesSeen;		// counts the number of nodes seen
	string fileName;		// name of the file // the prefix e.g. d - dMaps.txt, dCoords.txt, dOutput.txt, dStats.txt
	float currentWaitTime;	// current time left to wait, decreases

	// private func
	void(*SetMapSquare)(int i, int j, cubeTypes newType, cubeStatus newStatus);	// function pointer to the set map square function
	void LoadCoords(string givenMapName);	// Loads coords
	void LoadMap(string givenMapName);		// Loads map
	int CalcManDist(pair<int, int> Loc);	// Calc manhattan distance
	int CalcRunDist(unique_ptr <coords>& givenPoint);	// Calc current distance from start
	void DisplayMap();									// used to display the map to console for debugging
	bool Find(deque<unique_ptr<coords>>& myList, pair<int, int> Loc);	// find is location is already in list
	void SwapFirstWithCheck(deque<unique_ptr<coords>>& myList, unique_ptr <coords>& givenPoint);	// swap the given point with the first match in the list
	void DisplayList(deque<unique_ptr<coords>>& myList);	// Display the given list in the console
	void ReturnPath(unique_ptr <coords>& givenPoint);		// Puts the path from givenPoint to start into mPath
	void CPathFinder::WriteResult(bool live);		// Write the restlts to files xOutput.txt and xStats.txt x = filename


public:
	CPathFinder(I3DEngine* givenEngine, string givenFileName);		// constructor
	~CPathFinder();		// deconstructor

	void Load(string mapName);	// Loads the named map

	pair<int, int> GetMapSize() { return mMapSize; };	// returns the map size
	vector<vector<int>> GetMap() { return mMap; };		// returns the map
	pair<int, int> GetMapStart() { return mStart; };	// returns the start
	pair<int, int> GetMapEnd() { return mEnd; };		// returns the end
	vector<pair<int, int>> GetPath() { return mPath; };	// returns the path
	void SetMap(string givenFileName);					// Set a new map
	void SolveAStar(bool live = false);						// Solve the current map
	void PassFunc(void(*function)(int i, int j, cubeTypes newType, cubeStatus newStatus));
};

