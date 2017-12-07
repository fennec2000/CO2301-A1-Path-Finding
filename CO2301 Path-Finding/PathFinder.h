#pragma once
#include <vector>	// vector
#include <utility>	// pair
#include <fstream>	// ifstream
#include <string>	// string getline
#include <sstream>	// stringstream
#include <iostream> // cout
#include <iterator> // istream_iterator
using namespace std;

class CPathFinder
{
private:
	vector<vector<int>> mMap;	// the current map
	pair<int, int> mStart;		// starting point
	pair<int, int> mEnd;			// ending point / goal

	// private func
	void LoadCoords(string givenMapName); //Loads coords
	void LoadMap(string givenMapName); //Loads map

public:
	CPathFinder();		// constructor
	~CPathFinder();		// deconstructor

	void Load(string mapName); // Loads the named map
};

