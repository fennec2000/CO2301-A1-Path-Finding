// CO2301 Path-Finding.cpp: A program using the TL-Engine
// Stuart Hayes		20363714

#include <TL-Engine.h>	// TL-Engine include file and namespace
#include "PathFinder.h" // path finder class
#include "Vec3.h"		// Vec3
#include "Matrix4x4.h"	// 4x4 matrics class
#include <filesystem>	// filesystem
using namespace tle;

enum cubeTypes { wall, clear, wood, water, start, end, hidden, numOfCubeTypes };
enum cubeStatus { unknown, seen, visited, numOfStatus };

// declarations
void LookAt(Vec3 targetPosition, IModel* myModel);
vector<string> GetFiles();
pair<float, float> Bezeir(vector<pair<int, int>> &waypoints, int currentPoint, int splits, int currentSplit, pair<int, int> mapStart);

void main()
{
	// Create a 3D engine (using TLX engine here) and open a window for it
	I3DEngine* pMyEngine = New3DEngine(kTLX);
	pMyEngine->StartWindowed();

	vector<string> ListOfMaps = GetFiles();

	// Pathfinder
	CPathFinder* pCMyPathFinder = new CPathFinder(ListOfMaps[0]);

	// Add default folder for meshes and other media
	pMyEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");
	pMyEngine->AddMediaFolder("media");

	// Varables
	float frameTimer = 0.0f;
	const float TIME_BETWEEN_STEPS = 1.00f;
	float timeLeftForNextStep = TIME_BETWEEN_STEPS;
	bool autoStep = false;
	bool singleStep = false;
	int currentPoint = -2;
	vector<pair<int, int>> waypoints = pCMyPathFinder->GetPath();
	bool displayedFoundPath = false;
	bool guardMove = false;
	const float EPS = 0.1f;
	int currentMap = 0;
	const int numOfBezierSections = 10;
	int currentBezierSection = 1;
	pair<float, float> BezierWaypoint(-2.0f, -2.0f);
	bool haveBezierWaypoint = false;

	// keybindings
	EKeyCode buttonClose = Key_Escape;	// quit key
	EKeyCode autoStepButton = Key_A;	// auto step key
	EKeyCode singleStepButton = Key_Space;	// single step key
	EKeyCode hideUIButton = Key_F1;		// Hide ui key
	EKeyCode hideMapUIButton = Key_F2;	// Hide ui key
	EKeyCode nextMapButton = Key_Right;	// Next map button
	EKeyCode prevMapButton = Key_Left;	// Previous map button
	EKeyCode loadMapButton = Key_Return;	// Load the current map

	/**** Set up your scene here ****/
	// font
	const int textSize = 24;
	IFont* myFont = pMyEngine->LoadFont("Consolas", textSize);
	bool ShowUI = true;
	bool ShowMapUI = true;
	int mapTestPos = pMyEngine->GetWidth() % pMyEngine->GetHeight() * 3 / 4 + pMyEngine->GetHeight();
	const int numOfUI = 9;
	string UI_Info[numOfUI] = { "F1: Hide UI", "F2: Hide Map File", "A: Auto step", "Space: Single step", "Enter: Load Map", "Left Arrow: Previous", "map file", "Right Arrow: Next map", "file" };

	// Meshs
	IMesh* floorMesh = pMyEngine->LoadMesh("Floor.x");
	IMesh* cubeMesh = pMyEngine->LoadMesh("Cube.x");
	IMesh* mobMesh = pMyEngine->LoadMesh("sierra.x");

	// Models
	IModel* floor = floorMesh->CreateModel(0.0f, -10.0f, 0.0f);

	// cubes
	string cubeSkins[cubeStatus::numOfStatus][cubeTypes::numOfCubeTypes] = {
		{ "wall.jpg", "clear.jpg", "wood.jpg", "water.jpg", "start.jpg", "end.jpg" },
		{ "na", "clear_seen.jpg", "wood_seen.jpg", "water_seen.jpg", "start_seen.jpg", "end_seen.jpg" },
		{ "na", "clear_visited.jpg", "wood_visited.jpg", "water_visited.jpg", "start_visited.jpg", "end_visited.jpg" }, };
	float cubeYOffset[] = { 0.0f, -5.0f, -4.5f, -5.5f };
	float cubeSize = 10.0f;

	// multi vector of cubes
	pair<int, int> mapSize = pCMyPathFinder->GetMapSize();
	pair<int, int> mapStart = pCMyPathFinder->GetMapStart();
	pair<int, int> mapEnd = pCMyPathFinder->GetMapEnd();
	vector<vector<int>> map = pCMyPathFinder->GetMap();
	vector <vector <IModel*>> cubes;

	for (int i = 0; i < mapSize.second; ++i)
	{
		vector <IModel*> tmpCubes;

		for (int j = 0; j < mapSize.first; ++j)
		{
			IModel* tmpCubeModel;

			switch (map[i][j])
			{
			case cubeTypes::wall:
				tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, cubeYOffset[cubeTypes::wall], i * cubeSize);
				tmpCubeModel->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::wall]);
				break;
			case cubeTypes::clear:
				tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, cubeYOffset[cubeTypes::clear], i * cubeSize);
				tmpCubeModel->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::clear]);
				break;
			case cubeTypes::wood:
				tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, cubeYOffset[cubeTypes::wood], i * cubeSize);
				tmpCubeModel->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::wood]);
				break;
			case cubeTypes::water:
				tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, cubeYOffset[cubeTypes::water], i * cubeSize);
				tmpCubeModel->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::water]);
				break;
			default:
#ifdef DEBUG
				cout << "Invalid cube type" << endl;
#endif // DEBUG

				break;
			}

			tmpCubes.push_back(tmpCubeModel);
		}
		cubes.push_back(tmpCubes);
	}

	// set the start
	cubes[mapStart.second][mapStart.first]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::start]);
	map[mapStart.second][mapStart.first] = cubeTypes::start;

	// set the end
	cubes[mapEnd.second][mapEnd.first]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::end]);
	map[mapEnd.second][mapEnd.first] = cubeTypes::end;

	// mob variables
	float mobSpeed = 1.0f;
	float halfMobHieght = 1.742465f / 2.0f;
	pair<float, float> mobPos;

	// mob
	IModel* mob = mobMesh->CreateModel();
	mob->Scale(cubeSize);
	mob->SetPosition(mapStart.first * cubeSize, 0.0f, mapStart.second * cubeSize);

	// Camera
	const int MY_CAMERA_SPEED = 10;

	// myCamera
	ICamera* myCamera = pMyEngine->CreateCamera(kManual);
	myCamera->SetPosition(cubeSize * 4.5f, cubeSize * 10.0f, cubeSize * 4.5f);
	myCamera->RotateX(90.0f);
	myCamera->SetMovementSpeed(2.0f * MY_CAMERA_SPEED);
	myCamera->SetRotationSpeed(MY_CAMERA_SPEED);

	// The main game loop, repeat until engine is stopped
	while (pMyEngine->IsRunning())
	{
		// Draw the scene
		pMyEngine->DrawScene();

		// Get frame time
		frameTimer = pMyEngine->Timer();
		if (autoStep)
			timeLeftForNextStep -= frameTimer;

		if ((0.0f > timeLeftForNextStep || singleStep) && !displayedFoundPath)
		{
			singleStep = false;
			timeLeftForNextStep = TIME_BETWEEN_STEPS;
			++currentPoint;
			pair<int, int> currentWaypath;

			if (currentPoint == -1) // starting point
				currentWaypath = mapStart;
			else
				currentWaypath = waypoints[currentPoint];

			cubes[currentWaypath.second][currentWaypath.first]->SetSkin(cubeSkins[cubeStatus::visited][map[currentWaypath.second][currentWaypath.first] % cubeTypes::numOfCubeTypes]);
			map[currentWaypath.second][currentWaypath.first] += cubeTypes::numOfCubeTypes * cubeStatus::visited;

			pair<int, int> tmp;
			for (int i = 0; i < dirrection::NumberOfDirections; ++i)
			{
				tmp = currentWaypath;
				switch (i)
				{
				case dirrection::North:
					++(tmp.second);
					break;
				case dirrection::East:
					++(tmp.first);
					break;
				case dirrection::South:
					--(tmp.second);
					break;
				case dirrection::West:
					--(tmp.first);
					break;
				default:
					break;
				}
				// is valid?
				if (tmp.first < 0 || tmp.first >= mapSize.first ||
					tmp.second < 0 || tmp.second >= mapSize.second)
				{
#ifdef DEBUG
					std::cout << "Tile out of bounds" << endl;
#endif // DEBUG
					continue; // std::cout of bounds go to next itt
				}

				// if wall ignore
				if (map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes == cubeTypes::wall)
					continue;

				if (tmp == mapEnd) // found the goal
				{
					displayedFoundPath = true;
					guardMove = true;
					cubes[tmp.second][tmp.first]->SetSkin(cubeSkins[cubeStatus::visited][map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes]);
					map[tmp.second][tmp.first] += cubeTypes::numOfCubeTypes * cubeStatus::visited;
					currentPoint = 0;
				}

				// if not visited set visited
				if (map[tmp.second][tmp.first] / cubeTypes::numOfCubeTypes < 1)
				{
					cubes[tmp.second][tmp.first]->SetSkin(cubeSkins[cubeStatus::seen][map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes]);
					map[tmp.second][tmp.first] += cubeTypes::numOfCubeTypes;
				}
			}
		}

		if ((0.0f > timeLeftForNextStep || singleStep) && guardMove)
		{
			mobPos.first = mob->GetX();
			mobPos.second = mob->GetZ();

			// at goal?
			if (mobPos.first < waypoints[waypoints.size() - 1].first * cubeSize + EPS && mobPos.first > waypoints[waypoints.size() - 1].first * cubeSize - EPS &&
				mobPos.second < waypoints[waypoints.size() - 1].second * cubeSize + EPS && mobPos.second > waypoints[waypoints.size() - 1].second * cubeSize - EPS)
			{
				guardMove = false;
			}

			// last waypoint
			if (currentPoint >= waypoints.size() - 1)
			{
				LookAt(Vec3(waypoints[waypoints.size() - 1].first * cubeSize, halfMobHieght, waypoints[waypoints.size() - 1].second * cubeSize), mob);
			}
			else
			{
				if (std::floorf(numOfBezierSections * 0.6f) <= currentBezierSection)
				{
					++currentPoint;
					currentBezierSection = std::ceilf(numOfBezierSections * 0.3f);
					haveBezierWaypoint = false;
				}

				if (!haveBezierWaypoint)
				{
					BezierWaypoint = Bezeir(waypoints, currentPoint, numOfBezierSections, currentBezierSection, mapStart);
					cout << "wp: " << waypoints[currentPoint].first << ", " << waypoints[currentPoint].second << " step: " << currentBezierSection / static_cast<float>(numOfBezierSections) << ", Bez: " << BezierWaypoint.first << ", " << BezierWaypoint.second << endl;
					haveBezierWaypoint = true;
				}


				if (mobPos.first < BezierWaypoint.first * cubeSize + EPS && mobPos.first > BezierWaypoint.first * cubeSize - EPS &&
					mobPos.second < BezierWaypoint.second * cubeSize + EPS && mobPos.second > BezierWaypoint.second * cubeSize - EPS)
				{
					++currentBezierSection;
					haveBezierWaypoint = false;


				}

				LookAt(Vec3(BezierWaypoint.first * cubeSize, halfMobHieght, BezierWaypoint.second * cubeSize), mob);
			}
			mob->Scale(cubeSize);
			mob->MoveLocalZ(mobSpeed * frameTimer);
		}

		// UI
		if (ShowUI)
		{
			for (int i = 0; i < numOfUI; ++i)
			{
				myFont->Draw(UI_Info[i], 0, textSize * i);
			}
		}
		if (ShowMapUI && ShowUI)
		{
			myFont->Draw("Current map:", mapTestPos, 0, kBlack, kCentre);
			myFont->Draw(ListOfMaps[currentMap], mapTestPos, textSize, kBlack, kCentre);
			if (waypoints.empty())
				myFont->Draw("No Path Found", mapTestPos, textSize * 2, kBlack, kCentre);
		}

		// keybindings
		if (pMyEngine->KeyHit(autoStepButton) && !waypoints.empty())
		{
			autoStep = !autoStep;
		}
		if (pMyEngine->KeyHit(singleStepButton) && !waypoints.empty())
		{
			singleStep = true;
		}

		if (pMyEngine->KeyHit(nextMapButton))
		{
			++currentMap;
			if (ListOfMaps.size() <= currentMap)
				currentMap = 0;
		}
		if (pMyEngine->KeyHit(prevMapButton))
		{
			--currentMap;
			if (0 > currentMap)
				currentMap = ListOfMaps.size() - 1;
		}
		if (pMyEngine->KeyHit(loadMapButton))
		{
			pCMyPathFinder->SetMap(ListOfMaps[currentMap]);

			// Stop the auto solve
			autoStep = false;

			// Reset Data
			currentPoint = -2;
			waypoints = pCMyPathFinder->GetPath();
			displayedFoundPath = false;
			guardMove = false;
			currentBezierSection = 1;
			haveBezierWaypoint = false;
			mapSize = pCMyPathFinder->GetMapSize();
			mapStart = pCMyPathFinder->GetMapStart();
			mapEnd = pCMyPathFinder->GetMapEnd();
			map = pCMyPathFinder->GetMap();

			if (mapSize.second * 9 > mapSize.first * 16)
				myCamera->SetPosition(cubeSize * (mapSize.first * 0.5f - 0.5f), cubeSize * mapSize.second, cubeSize * (mapSize.second * 0.5f - 0.5f));
			else
				myCamera->SetPosition(cubeSize * (mapSize.first * 0.5f - 0.5f), cubeSize * mapSize.first, cubeSize * (mapSize.second * 0.5f - 0.5f));

			for (int i = 0; i < cubes.size(); ++i)
			{
				for (int j = 0; j < cubes[i].size(); ++j)
				{
					cubes[i][j]->MoveY(-cubeSize * 2);
				}
			}

			for (int i = 0; i < mapSize.second; ++i)
			{
				for (int j = 0; j < mapSize.first; ++j)
				{
					while (cubes.size() <= i)
					{
						vector <IModel*> tmpCubes;
						cubes.push_back(tmpCubes);
					}

					while (cubes[i].size() <= j)
					{
						IModel* tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, -cubeSize * 2, i * cubeSize);
						cubes[i].push_back(tmpCubeModel);
					}

					switch (map[i][j])
					{
					case cubeTypes::wall:
						cubes[i][j]->SetPosition(j * cubeSize, cubeYOffset[cubeTypes::wall], i * cubeSize);
						cubes[i][j]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::wall]);
						break;
					case cubeTypes::clear:
						cubes[i][j]->SetPosition(j * cubeSize, cubeYOffset[cubeTypes::clear], i * cubeSize);
						cubes[i][j]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::clear]);
						break;
					case cubeTypes::wood:
						cubes[i][j]->SetPosition(j * cubeSize, cubeYOffset[cubeTypes::wood], i * cubeSize);
						cubes[i][j]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::wood]);
						break;
					case cubeTypes::water:
						cubes[i][j]->SetPosition(j * cubeSize, cubeYOffset[cubeTypes::water], i * cubeSize);
						cubes[i][j]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::water]);
						break;
					default:
#ifdef DEBUG
						cout << "Invalid cube type" << endl;
#endif // DEBUG

						break;
					}
				}
			}

			// set the start
			cubes[mapStart.second][mapStart.first]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::start]);
			map[mapStart.second][mapStart.first] = cubeTypes::start;

			// set the end
			cubes[mapEnd.second][mapEnd.first]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::end]);
			map[mapEnd.second][mapEnd.first] = cubeTypes::end;

			// Reset mob
			mob->SetPosition(mapStart.first * cubeSize, 0.0f, mapStart.second * cubeSize);
		}

		// UI
		if (pMyEngine->KeyHit(hideUIButton))
		{
			ShowUI = !ShowUI;
		}
		if (pMyEngine->KeyHit(hideMapUIButton))
		{
			ShowMapUI = !ShowMapUI;
		}

		// close
		if (pMyEngine->KeyHit(buttonClose))
		{
			pMyEngine->Stop();
		}
	}

	// Delete the 3D engine now we are finished with it
	delete pCMyPathFinder;
	pMyEngine->Delete();
}

void LookAt(Vec3 targetPosition, IModel* myModel)
{
	Vec3 myPosition(myModel->GetX(), myModel->GetY(), myModel->GetZ());
	// Calculate matrix axes for guard
	// Get facing (z) vector from positions
	Vec3 vecZ = Normalise(Subtract(targetPosition, myPosition));
	// Use cross products to get other axes
	// Must normalise axes
	// Use the y axis as a base
	Vec3 vecX = Normalise(Cross(kYAxis, vecZ));
	Vec3 vecY = Normalise(Cross(vecZ, vecX));

	// Build matrix from axes + position
	// Matrix constructor using four CVector3 variables
	// - one for each row/column
	// (see matrix header)
	// Build matrix by row (axes + position)
	Matrix4x4 myMat;
	myMat.MakeIdentity();
	myMat.SetRow(0, vecX);
	myMat.SetRow(1, vecY);
	myMat.SetRow(2, vecZ);
	myMat.SetRow(3, myPosition);

	// Set position of guard using matrix
	myModel->SetMatrix(&myMat.e00);

}

// check to see if 'test' ends with 'ending'
bool EndsWith(string test, string ending)
{
	if (test.length() >= ending.length())
	{
		return (0 == test.compare(test.length() - ending.length(), ending.length(), ending));
	}
	return false;
}

vector<string> GetFiles()
{
	vector<string> MapsFound;
	string last;


	for (auto & p : std::experimental::filesystem::directory_iterator("maps")) // long name but only used once
	{
		stringstream ss;
		ss << p;

		// substr uses 5 to skip maps\\ and 15 to skip the affex

		if (EndsWith(ss.str(), "Coords.txt")) // coords found
		{
			last = ss.str().substr(5, ss.str().length() - 15); // Get prefix
		}

		else if (EndsWith(ss.str(), "Map.txt")) // map found
		{
			if (last.compare(ss.str().substr(5, ss.str().length() - 15))) // if prefix's match
			{
				// valid map
				MapsFound.push_back(last);
			}
		}
	}

	return MapsFound;
}

float BezeirFormula(float t, float p1x, float p2x, float p3x, float p4x)
{
	return (1-t) * (1-t) * (1-t) * p1x + 3 * t * (1-t) * (1-t) * p2x + 3 * t * t * (1-t) * p3x + t * t * t * p4x;
}

pair<float, float> Bezeir(vector<pair<int, int>> &waypoints, int currentPoint, int splits, int currentSplit, pair<int, int> mapStart)
{
	pair<float, float> ans;

	if (0 == currentPoint)
	{
		ans.second = BezeirFormula(currentSplit / static_cast<float>(splits), mapStart.second, mapStart.second, waypoints[currentPoint].second, waypoints[currentPoint + 1].second);
		ans.first = BezeirFormula(currentSplit / static_cast<float>(splits), mapStart.first, mapStart.first, waypoints[currentPoint].first, waypoints[currentPoint + 1].first);
	}
	else if (1 == currentPoint)
	{
		ans.second = BezeirFormula(currentSplit / static_cast<float>(splits), mapStart.second, waypoints[currentPoint - 1].second, waypoints[currentPoint].second, waypoints[currentPoint + 1].second);
		ans.first = BezeirFormula(currentSplit / static_cast<float>(splits), mapStart.first, waypoints[currentPoint - 1].first, waypoints[currentPoint].first, waypoints[currentPoint + 1].first);
	}
	else if (waypoints.size() - 1 == currentPoint)
	{
		ans.second = BezeirFormula(currentSplit / static_cast<float>(splits), waypoints[currentPoint - 2].second, waypoints[currentPoint - 1].second, waypoints[currentPoint].second, waypoints[currentPoint].second);
		ans.first = BezeirFormula(currentSplit / static_cast<float>(splits), waypoints[currentPoint - 2].first, waypoints[currentPoint - 1].first, waypoints[currentPoint].first, waypoints[currentPoint].first);
	}
	else
	{
		ans.second = BezeirFormula(currentSplit / static_cast<float>(splits), waypoints[currentPoint - 2].second, waypoints[currentPoint - 1].second, waypoints[currentPoint].second, waypoints[currentPoint + 1].second);
		ans.first = BezeirFormula(currentSplit / static_cast<float>(splits), waypoints[currentPoint - 2].first, waypoints[currentPoint - 1].first, waypoints[currentPoint].first, waypoints[currentPoint + 1].first);
	}
	return ans;
}