// CO2301 Path-Finding.cpp: A program using the TL-Engine
// Stuart Hayes		20363714

#include <TL-Engine.h>	// TL-Engine include file and namespace
#include "PathFinder.h" // path finder class
#include "Vec3.h"		// Vec3
#include "Matrix4x4.h"	// 4x4 matrics class
#include <filesystem>	// filesystem
using namespace tle;

// declarations
void LookAt(Vec3 targetPosition, IModel* myModel);
vector<string> GetFiles();
pair<float, float> Bezeir(vector<pair<int, int>> &waypoints, int currentPoint, int splits, int currentSplit, pair<int, int> mapStart);

// Global
vector <vector <IModel*>> gCubes;
I3DEngine* gpMyEngine;

// cubes
const string gCUBE_SKINS[cubeStatus::numOfStatus][cubeTypes::numOfCubeTypes] = {
	{ "wall.jpg", "clear.jpg", "wood.jpg", "water.jpg", "start.jpg", "end.jpg" },
	{ "na", "clear_seen.jpg", "wood_seen.jpg", "water_seen.jpg", "start_seen.jpg", "end_seen.jpg" },
	{ "na", "clear_visited.jpg", "wood_visited.jpg", "water_visited.jpg", "start_visited.jpg", "end_visited.jpg" },
	{ "na", "clear_path.jpg", "wood_path.jpg", "water_path.jpg", "start_path.jpg", "end_path.jpg" }, };
const float gCUBE_Y_OFFSET[] = { 0.0f, -5.0f, -4.5f, -5.5f };
const float gCUBE_SIZE = 10.0f;

void SetMapSquare(int i, int j, cubeTypes newType, cubeStatus newStatus)
{
	if (newStatus < cubeTypes::numOfCubeTypes)
	{
		if (cubeTypes::start == newType || cubeTypes::end == newType)
			gCubes[i][j]->SetPosition(j * gCUBE_SIZE, gCUBE_Y_OFFSET[cubeTypes::clear], i * gCUBE_SIZE);
		else
			gCubes[i][j]->SetPosition(j * gCUBE_SIZE, gCUBE_Y_OFFSET[newType], i * gCUBE_SIZE);

		gCubes[i][j]->SetSkin(gCUBE_SKINS[newStatus][newType]);
	}
	else
	{
#ifdef DEBUG
		cout << "Invalid cube type" << endl;
#endif // DEBUG
	}
}

void main()
{
	// Create a 3D engine (using TLX engine here) and open a window for it
	gpMyEngine = New3DEngine(kTLX);
	gpMyEngine->StartWindowed();

	vector<string> ListOfMaps = GetFiles();

	// Pathfinder
	CPathFinder* pCMyPathFinder = new CPathFinder(gpMyEngine, ListOfMaps[0]);
	pCMyPathFinder->PassFunc(SetMapSquare);
	pCMyPathFinder->SolveAStar();

	// Add default folder for meshes and other media
	gpMyEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");
	gpMyEngine->AddMediaFolder("media");

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
	const int NUM_OF_BEZIER_SECTIONS = 10;
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
	EKeyCode liveMapVersion = Key_L;	// Live key

	/**** Set up your scene here ****/
	// font
	const int textSize = 24;
	IFont* myFont = gpMyEngine->LoadFont("Consolas", textSize);
	bool ShowUI = true;
	bool ShowMapUI = true;
	int mapTestPos = gpMyEngine->GetWidth() % gpMyEngine->GetHeight() * 3 / 4 + gpMyEngine->GetHeight();
	const int numOfUI = 10;
	string UI_Info[numOfUI] = { "F1: Hide UI", "F2: Hide Map File", "A: Auto step", "Space: Single step", "L: Live", "Enter: Load Map", "Left Arrow: Previous", "map file", "Right Arrow: Next map", "file" };

	// Meshs
	IMesh* floorMesh = gpMyEngine->LoadMesh("Floor.x");
	IMesh* cubeMesh = gpMyEngine->LoadMesh("Cube.x");
	IMesh* mobMesh = gpMyEngine->LoadMesh("sierra.x");

	// Models
	IModel* floor = floorMesh->CreateModel(0.0f, -10.0f, 0.0f);
	// multi vector of cubes
	pair<int, int> mapSize = pCMyPathFinder->GetMapSize();
	pair<int, int> mapStart = pCMyPathFinder->GetMapStart();
	pair<int, int> mapEnd = pCMyPathFinder->GetMapEnd();
	vector<vector<int>> map = pCMyPathFinder->GetMap();

	for (int i = 0; i < mapSize.second; ++i)
	{
		vector <IModel*> tmpCubes;

		for (int j = 0; j < mapSize.first; ++j)
		{
			IModel* tmpCubeModel;


			if (cubeTypes::start == map[i][j] || cubeTypes::end == map[i][j])
				tmpCubeModel = cubeMesh->CreateModel(j * gCUBE_SIZE, gCUBE_Y_OFFSET[cubeTypes::clear], i * gCUBE_SIZE);
			else
				tmpCubeModel = cubeMesh->CreateModel(j * gCUBE_SIZE, gCUBE_Y_OFFSET[map[i][j]], i * gCUBE_SIZE);

			tmpCubeModel->SetSkin(gCUBE_SKINS[cubeStatus::unknown][map[i][j]]);
			tmpCubes.push_back(tmpCubeModel);
		}
		gCubes.push_back(tmpCubes);
	}

	// set the start
	SetMapSquare(mapStart.second, mapStart.first, cubeTypes::start, cubeStatus::unknown);

	// set the end
	SetMapSquare(mapEnd.second, mapEnd.first, cubeTypes::end, cubeStatus::unknown);

	// mob variables
	float mobSpeed = 1.0f;
	float halfMobHieght = 1.742465f / 2.0f;
	pair<float, float> mobPos;

	// mob
	IModel* mob = mobMesh->CreateModel();
	mob->Scale(gCUBE_SIZE); // mob scale base off its surroundings, a cube
	mob->SetPosition(mapStart.first * gCUBE_SIZE, 0.0f, mapStart.second * gCUBE_SIZE);

	// Camera
	const int MY_CAMERA_SPEED = 10;

	// myCamera
	ICamera* myCamera = gpMyEngine->CreateCamera(kManual);
	myCamera->SetPosition(gCUBE_SIZE * 4.5f, 100.0f, gCUBE_SIZE * 4.5f);
	myCamera->RotateX(90.0f);
	myCamera->SetMovementSpeed(2.0f * MY_CAMERA_SPEED);
	myCamera->SetRotationSpeed(MY_CAMERA_SPEED);

	// hide mouse
	//gpMyEngine->StartMouseCapture();

	// The main game loop, repeat until engine is stopped
	while (gpMyEngine->IsRunning())
	{
		// Draw the scene
		gpMyEngine->DrawScene();

		// Get frame time
		frameTimer = gpMyEngine->Timer();
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

			gCubes[currentWaypath.second][currentWaypath.first]->SetSkin(gCUBE_SKINS[cubeStatus::path][map[currentWaypath.second][currentWaypath.first] % cubeTypes::numOfCubeTypes]);
			map[currentWaypath.second][currentWaypath.first] += cubeTypes::numOfCubeTypes * cubeStatus::path;

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
					gCubes[tmp.second][tmp.first]->SetSkin(gCUBE_SKINS[cubeStatus::path][map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes]);
					map[tmp.second][tmp.first] += cubeTypes::numOfCubeTypes * cubeStatus::path;
					currentPoint = 0;
				}

				// if not visited set visited
				if (map[tmp.second][tmp.first] / cubeTypes::numOfCubeTypes < 1)
				{
					gCubes[tmp.second][tmp.first]->SetSkin(gCUBE_SKINS[cubeStatus::seen][map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes]);
					map[tmp.second][tmp.first] += cubeTypes::numOfCubeTypes;
				}
			}
		}

		if ((0.0f > timeLeftForNextStep || singleStep) && guardMove)
		{
			mobPos.first = mob->GetX();
			mobPos.second = mob->GetZ();

			if (mobPos.first > mapSize.first * gCUBE_SIZE || mobPos.first < -gCUBE_SIZE || mobPos.second > mapSize.second * gCUBE_SIZE || mobPos.second < -gCUBE_SIZE)
				mob->SetPosition(mapStart.first * gCUBE_SIZE, 0.0f, mapStart.second * gCUBE_SIZE);

			// at goal?
			if (mobPos.first < waypoints[waypoints.size() - 1].first * gCUBE_SIZE + EPS && mobPos.first > waypoints[waypoints.size() - 1].first * gCUBE_SIZE - EPS &&
				mobPos.second < waypoints[waypoints.size() - 1].second * gCUBE_SIZE + EPS && mobPos.second > waypoints[waypoints.size() - 1].second * gCUBE_SIZE - EPS)
			{
				guardMove = false;
			}

			// last waypoint
			if (currentPoint >= waypoints.size() - 1)
			{
				LookAt(Vec3(waypoints[waypoints.size() - 1].first * gCUBE_SIZE, halfMobHieght, waypoints[waypoints.size() - 1].second * gCUBE_SIZE), mob);
			}
			else
			{
				if (std::floorf(NUM_OF_BEZIER_SECTIONS * 0.6f) <= currentBezierSection)
				{
					++currentPoint;
					currentBezierSection = std::ceilf(NUM_OF_BEZIER_SECTIONS * 0.3f);
					haveBezierWaypoint = false;
				}

				if (!haveBezierWaypoint)
				{
					BezierWaypoint = Bezeir(waypoints, currentPoint, NUM_OF_BEZIER_SECTIONS, currentBezierSection, mapStart);
					cout << "wp: " << waypoints[currentPoint].first << ", " << waypoints[currentPoint].second << " step: " << currentBezierSection / static_cast<float>(NUM_OF_BEZIER_SECTIONS) << ", Bez: " << BezierWaypoint.first << ", " << BezierWaypoint.second << endl;
					haveBezierWaypoint = true;
				}


				if (mobPos.first < BezierWaypoint.first * gCUBE_SIZE + EPS && mobPos.first > BezierWaypoint.first * gCUBE_SIZE - EPS &&
					mobPos.second < BezierWaypoint.second * gCUBE_SIZE + EPS && mobPos.second > BezierWaypoint.second * gCUBE_SIZE - EPS)
				{
					++currentBezierSection;
					haveBezierWaypoint = false;


				}

				LookAt(Vec3(BezierWaypoint.first * gCUBE_SIZE, halfMobHieght, BezierWaypoint.second * gCUBE_SIZE), mob);
			}
			mob->Scale(gCUBE_SIZE);
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
		if (gpMyEngine->KeyHit(liveMapVersion))
		{
			pCMyPathFinder->SetMap(ListOfMaps[currentMap]);
			pCMyPathFinder->SolveAStar(true);

			// Reset Data
			waypoints = pCMyPathFinder->GetPath();
			mapSize = pCMyPathFinder->GetMapSize();
			mapStart = pCMyPathFinder->GetMapStart();
			mapEnd = pCMyPathFinder->GetMapEnd();
			map = pCMyPathFinder->GetMap();

			if (!waypoints.empty())
			{
				displayedFoundPath = true;
				guardMove = true;
				autoStep = true;
				currentPoint = 0;
			}

		}
			
		if (gpMyEngine->KeyHit(autoStepButton) && !waypoints.empty())
		{
			autoStep = !autoStep;
		}
		if (gpMyEngine->KeyHit(singleStepButton) && !waypoints.empty())
		{
			singleStep = true;
		}

		if (gpMyEngine->KeyHit(nextMapButton))
		{
			++currentMap;
			if (ListOfMaps.size() <= currentMap)
				currentMap = 0;
		}
		if (gpMyEngine->KeyHit(prevMapButton))
		{
			--currentMap;
			if (0 > currentMap)
				currentMap = ListOfMaps.size() - 1;
		}
		if (gpMyEngine->KeyHit(loadMapButton))
		{
			pCMyPathFinder->SetMap(ListOfMaps[currentMap]);
			pCMyPathFinder->SolveAStar();

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

			// set camera based off map
			if (mapSize.second * 9 > mapSize.first * 16)
				myCamera->SetPosition(gCUBE_SIZE * (mapSize.first * 0.5f - 0.5f), gCUBE_SIZE * mapSize.second, gCUBE_SIZE * (mapSize.second * 0.5f - 0.5f));
			else
				myCamera->SetPosition(gCUBE_SIZE * (mapSize.first * 0.5f - 0.5f), gCUBE_SIZE * mapSize.first, gCUBE_SIZE * (mapSize.second * 0.5f - 0.5f));

			for (int i = 0; i < gCubes.size(); ++i)
			{
				for (int j = 0; j < gCubes[i].size(); ++j)
				{
					gCubes[i][j]->MoveY(-gCUBE_SIZE * 2);
				}
			}

			for (int i = 0; i < mapSize.second; ++i)
			{
				for (int j = 0; j < mapSize.first; ++j)
				{
					while (gCubes.size() <= i)
					{
						vector <IModel*> tmpCubes;
						gCubes.push_back(tmpCubes);
					}

					while (gCubes[i].size() <= j)
					{
						IModel* tmpCubeModel = cubeMesh->CreateModel(j * gCUBE_SIZE, -gCUBE_SIZE * 2, i * gCUBE_SIZE);
						gCubes[i].push_back(tmpCubeModel);
					}

					SetMapSquare(i, j, static_cast<cubeTypes>(map[i][j]), cubeStatus::unknown);
				}
			}

			// set the start
			SetMapSquare(mapStart.second, mapStart.first, cubeTypes::start, cubeStatus::unknown);

			// set the end
			SetMapSquare(mapEnd.second, mapEnd.first, cubeTypes::end, cubeStatus::unknown);

			// Reset mob
			mob->SetPosition(mapStart.first * gCUBE_SIZE, 0.0f, mapStart.second * gCUBE_SIZE);
		}

		// UI
		if (gpMyEngine->KeyHit(hideUIButton))
		{
			ShowUI = !ShowUI;
		}
		if (gpMyEngine->KeyHit(hideMapUIButton))
		{
			ShowMapUI = !ShowMapUI;
		}

		// close
		if (gpMyEngine->KeyHit(buttonClose))
		{
			gpMyEngine->Stop();
		}
	}

	// Delete the 3D engine now we are finished with it
	delete pCMyPathFinder;
	gpMyEngine->Delete();
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
	return (1 - t) * (1 - t) * (1 - t) * p1x + 3 * t * (1 - t) * (1 - t) * p2x + 3 * t * t * (1 - t) * p3x + t * t * t * p4x;
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