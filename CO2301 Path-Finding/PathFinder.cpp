#include "PathFinder.h"

bool CompareCoords(unique_ptr<coords>& lhs, unique_ptr<coords>& rhs)
{
	return lhs->manhattanDist + lhs->runningDist < rhs->manhattanDist + rhs->runningDist;
}

CPathFinder::CPathFinder(I3DEngine* givenEngine, string givenFileName)
{
	mpMyEngine = givenEngine;
	SetMap(givenFileName);
}

void CPathFinder::SetMap(string givenFileName)
{
	// Reset the stats
	mNumOfSorts = 0;
	mNumOfNodesVisited = 0;
	mNumOfNodesSeen = 0;
	mCurrentWaitTime = waitTimer;

	mMap.clear();
	mPath.clear();

	mFileName = givenFileName;
	Load(mFileName);

#ifdef DEBUG
	std::cout << "Start: x: " << mStart.first << " y: " << mStart.second << endl;
	std::cout << "End: x: " << mEnd.first << " y: " << mEnd.second << endl;
	std::cout << "Maps size: " << mMapSize.first << "x" << mMapSize.second << endl;
	DisplayMap();
#endif // _DEBUG
}

void CPathFinder::PassSetMapSquare(void(*function)(int i, int j, ECubeTypes newType, ECubeStatus newStatus))
{
	SetMapSquare = function;
}

void CPathFinder::PassDisplayUI(void(*function)(CPathFinder* thePathfinder, bool isWaypointEmpty))
{
	DisplayUI = function;
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
	std::reverse(mMap.begin(), mMap.end());
	mMap[mStart.second][mStart.first] = ECubeTypes::start;
	mMap[mEnd.second][mEnd.first] = ECubeTypes::end;
}

void CPathFinder::DisplayMap()
{
	std::cout << "Display start" << endl;
	vector<vector<int>>::iterator row;
	vector<int>::iterator col;
	for (row = mMap.begin(); row != mMap.end(); row++)
	{
		for (col = row->begin(); col != row->end(); col++)
		{
			std::cout << to_string(*col);
		}
		std::cout << endl;
	}
	std::cout << "Display end" << endl;
}

void CPathFinder::SolveAStar(bool live)
{
	bool found = false;
	deque <unique_ptr <coords>> openList, closedList;
	unique_ptr <coords> current(new coords), tmp(new coords), goal(new coords);

	// put the start into open list
	current->location = mStart;
	current->manhattanDist = CalcManDist(mStart);
	current->runningDist = 0;
	current->parent = new coords;
	current->parent = 0;
	openList.push_back(move(current));
	current.reset(new coords);

	// while !openList.empty
	while (!openList.empty() && !found)
	{
		// pick best option (first)
		current = move(openList.front());
		openList.pop_front();
#ifdef DEBUG
		std::cout << "Moved front of openList to current" << endl;
		std::cout << "current: x: " << current->location.first << ", y: " << current->location.second << " ";
		std::cout << "ManDist: " << current->manhattanDist << " runDist: " << current->runningDist << " ";
		std::cout << "Tile: " << mMap[current->location.second][current->location.first] << " ";
		std::cout << endl;
#endif // DEBUG

		// count node has visited
		++mNumOfNodesVisited;
		if(live)
			SetMapSquare(current->location.second, current->location.first, static_cast<ECubeTypes>(mMap[current->location.second][current->location.first]), ECubeStatus::visited);

		// is goal?
		if (current->location.first == mEnd.first && current->location.second == mEnd.second)
		{
			// goal found
#ifdef DEBUG
			std::cout << endl << "***End found Hard***" << endl;
#endif // DEBUG

			goal = move(current);
			found = true;
			break;
		}

		// check arround
		for (int i = 0; i < EDirrection::NumberOfDirections; ++i)
		{
			// reset tmp // start with a new tmp
			tmp.reset(new coords);

			// set tmp's similar variables
			tmp->location = current->location;
			tmp->parent = current.get();

			switch (i)
			{
			case EDirrection::North:
				++(tmp->location.second);
				break;
			case EDirrection::East:
				++(tmp->location.first);
				break;
			case EDirrection::South:
				--(tmp->location.second);
				break;
			case EDirrection::West:
				--(tmp->location.first);
				break;
			default:
				break;
			}

			// is valid?
			if (tmp->location.first < 0 || tmp->location.first >= mMapSize.first ||
				tmp->location.second < 0 || tmp->location.second >= mMapSize.second)
			{
#ifdef DEBUG
				std::cout << "Tile out of bounds" << endl;
#endif // DEBUG
				continue; // std::cout of bounds go to next itt
			}
#ifdef DEBUG
			std::cout << "tmp: x: " << tmp->location.first << ", y: " << tmp->location.second << ". Parent: " << tmp->parent << " ";
			std::cout << "Tile: " << mMap[tmp->location.second][tmp->location.first] << " ";
#endif // DEBUG

			// valid node so we see it
			++mNumOfNodesSeen;

			// is goal?
			if (tmp->location.first == mEnd.first && tmp->location.second == mEnd.second)
			{

#ifdef DEBUG
				std::cout << endl << "***End found***" << endl;
#endif // DEBUG
				if(live)
					SetMapSquare(tmp->location.second, tmp->location.first, ECubeTypes::end, ECubeStatus::visited);
				goal = move(tmp);
				found = true;
				break; // goal found
			}

			
			if (!mMap[tmp->location.second][tmp->location.first])
			{
				// tile on map is wall do not add
#ifdef DEBUG
				std::cout << "Tile is a wall" << endl;
#endif // DEBUG
				continue;
			}

			// calc running dist
			tmp->runningDist = CalcRunDist(tmp);
#ifdef DEBUG
			std::cout << "runDist: " << tmp->runningDist << " ";
#endif // DEBUG

			// calc manhattan dist
			tmp->manhattanDist = CalcManDist(tmp->location);
#ifdef DEBUG
			std::cout << "manDist: " << tmp->manhattanDist << " ";
#endif // DEBUG

			// push to openList
			// check closed list
			if (!Find(closedList, tmp->location))
			{
				// is in open list?
				if (Find(openList, tmp->location))
				{
					// yes - it is better?
					SwapFirstWithCheck(openList, tmp);
				}
				else
				{
					// seen
					if (live)
						SetMapSquare(tmp->location.second, tmp->location.first, static_cast<ECubeTypes>(mMap[tmp->location.second][tmp->location.first]), ECubeStatus::seen);
					// no add it
					openList.push_back(move(tmp));
				}
			}

#ifdef DEBUG // neaten the debug output
			std::cout << endl;
#endif // DEBUG
		}
		// sort openList
		std::sort(openList.begin(), openList.end(), CompareCoords);
		++mNumOfSorts;

		// push current to closedList
		closedList.push_back(move(current));
		current.reset(new coords);

		if (live)
		{
			// slow down the solver and show its steps
			while (mCurrentWaitTime > 0)
			{
				mpMyEngine->DrawScene();
				mCurrentWaitTime -= mpMyEngine->Timer();
				DisplayUI(this, !found);
			}
			mCurrentWaitTime = waitTimer;
		}

#ifdef DEBUG // neaten the debug output
		std::cout << endl;

#endif // DEBUG
	}
	if (!found)
	{
		std::cout << "Path not found" << endl;
	}
	else
	{
		// Get a list from end to start
		ReturnPath(goal);
		// flip the list
		reverse(mPath.begin(), mPath.end());
		// print to xOutput.txt
		WriteResult(live);
	}
	// output info to xStats.txt
#ifdef DEBUG // neaten the debug output
	std::cout << "Number of sorts: " << mNumOfSorts <<  endl;
	std::cout << "openList: " << endl;
	DisplayList(openList);
	std::cout << "closedList: " << endl;
	DisplayList(closedList);
#endif // DEBUG
}

int CPathFinder::CalcManDist(pair<int, int> Loc)
{
	return abs(mEnd.first - Loc.first) + abs(mEnd.second - Loc.second);
}

int CPathFinder::CalcRunDist(unique_ptr <coords>& givenPoint)
{
	return givenPoint->parent->runningDist + mMap[givenPoint->location.second][givenPoint->location.first];
}

bool CPathFinder::Find(deque<unique_ptr<coords>>& myList, pair<int, int> Loc)
{
	for (auto it = myList.begin(); it != myList.end(); ++it)
	{
		if ((*it)->location.first == Loc.first && (*it)->location.second == Loc.second)
			return true;
	}
	return false;
}

void CPathFinder::SwapFirstWithCheck(deque<unique_ptr<coords>>& myList, unique_ptr <coords>& givenPoint)
{
	for (auto it = myList.begin(); it != myList.end(); ++it)
	{
		if ((*it)->location.first == givenPoint->location.first && (*it)->location.second == givenPoint->location.second &&
			CompareCoords(givenPoint, *it))
		{
			givenPoint.swap(*it);
			return;
		}
	}
}

void CPathFinder::DisplayList(deque<unique_ptr<coords>>& myList)
{
	for (auto it = myList.begin(); it != myList.end(); ++it)
	{
		std::cout << "x: " << (*it)->location.first << ", " << (*it)->location.second
			<<", manDist: " << (*it)->manhattanDist << " runDist: " << (*it)->runningDist << endl;
	}
}

void CPathFinder::ReturnPath(unique_ptr <coords>& givenPoint)
{
	coords* current;

	// put the first path into mPath
	mPath.push_back(givenPoint->location);
	current = givenPoint->parent;

	// loop till parent = 0
	while (current->parent != 0)
	{
		mPath.push_back(current->location);
		current = current->parent;
	}
}

void CPathFinder::WriteResult(bool live)
{
	ofstream myOutput, myStats;
	myOutput.open("maps/" + mFileName + "Output.txt", ios::trunc);

	for (vector<pair<int, int>>::iterator it = mPath.begin(); it != mPath.end(); ++it)
	{
		myOutput << (*it).first << " " << (*it).second << endl;
		// if live display path
		if (live)
			SetMapSquare((*it).second, (*it).first, static_cast<ECubeTypes>(mMap[(*it).second][(*it).first]), ECubeStatus::path);
	}
	myOutput.close();

	myStats.open("maps/" + mFileName + "Stats.txt", ios::trunc);
	myStats << "Number of sorts: " << mNumOfSorts << endl;
	myStats << "Nodes visited: " << mNumOfNodesVisited << endl;
	myStats << "Nodes seen: " << mNumOfNodesSeen << endl;
	myStats.close();
}