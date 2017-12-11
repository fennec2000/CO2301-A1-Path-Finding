#include "PathFinder.h"

CPathFinder::CPathFinder()
{
	Load("d");
}


CPathFinder::~CPathFinder()
{
}

void CPathFinder::Load(string mapName)
{
	LoadCoords(mapName);
	LoadMap(mapName);
}

void CPathFinder::LoadCoords(string givenMapName)
{
	string inputString;

	ifstream myCoords("maps/" + givenMapName + "Coords.txt"); // all map files are in the maps folder and must follow convention

	if (myCoords.is_open())
	{
		// magic number is here due to only wanting the first 2 and is never used again
		for (int i = 0; i < 2; ++i)
		{
			getline(myCoords, inputString);
			// using this turn the numbers from a string to two ints
			istringstream buf(inputString);
			istream_iterator<int> beg(buf), end;
			vector<int> tokens(beg, end);

			if (i == 0) // is its the start point
			{
				mStart.first = tokens[0];
				mStart.second = tokens[1];
			}
			else // this must be the end
			{
				mEnd.first = tokens[0];
				mEnd.second = tokens[1];
			}
		}
		myCoords.close();
	}
}

void CPathFinder::LoadMap(string givenMapName)
{
	vector<int> tmp;
	string inputString;

	ifstream myMap("maps/" + givenMapName + "Map.txt"); // all map files are in the maps folder and must follow convention

	if (myMap.is_open())
	{
		while (getline(myMap, inputString))
		{
			for (int i = 0; i < inputString.length(); ++i)
			{
				tmp.push_back(inputString[i] - '0');
			}
			mMap.push_back(tmp);
			tmp.clear();
		}
		myMap.close();
	}
}

void CPathFinder::DisplayMap()
{
	cout << "Display start" << endl;
	vector<vector<int>>::iterator row;
	vector<int>::iterator col;
	for (row = mMap.begin(); row != mMap.end(); row++)
	{
		for (col = row->begin(); col != row->end(); col++)
		{
			cout << to_string(*col);
		}
		cout << endl;
	}
	cout << "Display end" << endl;
}

void CPathFinder::SolveAStar()
{

}