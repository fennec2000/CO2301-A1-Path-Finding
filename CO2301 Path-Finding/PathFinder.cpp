#include "PathFinder.h"

CPathFinder::CPathFinder()
{
	Load("d");

#ifdef DEBUG
	DisplayMap();
#endif // _DEBUG

	SolveAStar();
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
	int longestRow = 0;

	ifstream myMap("maps/" + givenMapName + "Map.txt"); // all map files are in the maps folder and must follow convention

	if (myMap.is_open())
	{
		// for each line
		while (getline(myMap, inputString))
		{
			// get each char
			for (int i = 0; i < inputString.length(); ++i)
			{
				tmp.push_back(inputString[i] - '0');
			}

			// track longest row
			if (inputString.length() > longestRow)
			{
				longestRow = inputString.length();
			}

			mMap.push_back(tmp);
			tmp.clear();
		}
		myMap.close();
	}
	// set the map size
	mMapSize.first = longestRow;
	mMapSize.second = mMap.size();
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
	deque <unique_ptr <coords>> openList, closedList;
	unique_ptr <coords> current(new coords), tmp(new coords);

	// put the start into open list
	current->location = mStart;
	current->manhattanDist = CalcManDist(mStart);
	current->runningDist = 0;
	current->parent = new coords;
	current->parent = 0;
	openList.push_back(move(current));
	current.reset(new coords);

	// while !openList.empty
	while (!openList.empty())
	{
		// pick best option (first)
		current = move(openList.front());
		openList.pop_front();
#ifdef DEBUG
		cout << "Moved front of openList to current" << endl;
		cout << "current: x: " << current->location.first << ", y: " << current->location.first << endl;
#endif // DEBUG


		// is goal?
		if (current->location == mEnd)
		{
			// goal found
			break;
		}

		// check arround
		for (int i = 0; i < dirrection::NumberOfDirections; ++i)
		{
			// set tmp's similar variables
			tmp->location = current->location;
			tmp->parent = current.get();

			switch (i)
			{
			case dirrection::North:
				++(tmp->location.second);
				break;
			case dirrection::East:
				++(tmp->location.first);
				break;
			case dirrection::South:
				--(tmp->location.second);
				break;
			case dirrection::West:
				--(tmp->location.first);
				break;
			default:
				break;
			}


#ifdef DEBUG
			cout << "tmp: x: " << tmp->location.first << ", y: " << tmp->location.second << ". Parent: " << tmp->parent << " ";
#endif // DEBUG

			// is valid?
			if (tmp->location.first < 0 || tmp->location.first >= mMapSize.first ||
				tmp->location.second < 0 || tmp->location.second >= mMapSize.second)
			{
				continue; // cout of bounds go to next itt
			}

			// is goal?
			if (tmp->location.first == mEnd.first && tmp->location.second == mEnd.second)
			{
				break; // goal found
			}

			// calc manhattan dist
			tmp->manhattanDist = CalcManDist(tmp->location);
#ifdef DEBUG
			cout << "manDist: " << tmp->manhattanDist << " ";
#endif // DEBUG

			// calc running dist
			tmp->runningDist = CalcRunDist(tmp);
#ifdef DEBUG
			cout << "runDist: " << tmp->runningDist << " ";
#endif // DEBUG

			// push to openList
			// is in open list?
			// yes - it is better?

			// sort openList

			// push current to closedList

#ifdef DEBUG // neaten the debug output
			cout << endl;
#endif // DEBUG
		}
	}
}

int CPathFinder::CalcManDist(pair<int, int> Loc)
{
	return abs(mEnd.first - Loc.first) + abs(mEnd.second - Loc.second);
}

int CPathFinder::CalcRunDist(unique_ptr <coords>& givenPoint)
{
	return givenPoint->parent->runningDist + mMap[givenPoint->location.first][givenPoint->location.second];
}

bool Find(deque<unique_ptr<coords>>& myList, pair<int, int> Loc)
{
	for (auto it = myList.begin(); it != myList.end(); ++it)
	{
		if ((*it)->location.first == Loc.first && (*it)->location.second == Loc.second)
			return true;
	}
	return false;
}