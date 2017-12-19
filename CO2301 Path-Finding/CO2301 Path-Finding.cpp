// CO2301 Path-Finding.cpp: A program using the TL-Engine

#include <TL-Engine.h>	// TL-Engine include file and namespace
#include "PathFinder.h" // path finder class
using namespace tle;

enum cubeTypes { wall, clear, wood, water, start, end, numOfCubeTypes };
enum cubeStatus { unknown, seen, visited, numOfStatus };

void main()
{
	// Create a 3D engine (using TLX engine here) and open a window for it
	I3DEngine* pMyEngine = New3DEngine(kTLX);
	pMyEngine->StartWindowed();

	// Pathfinder
	CPathFinder* pCMyPathFinder = new CPathFinder("m");

	// Add default folder for meshes and other media
	pMyEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");
	pMyEngine->AddMediaFolder("media");

	// Varables
	float frameTimer = 0.0f;
	const float TIME_BETWEEN_STEPS = 1.00f;
	float timeLeftForNextStep = TIME_BETWEEN_STEPS;
	bool autoStep = false;
	int currentPoint = -2;
	vector<pair<int, int>> waypoints = pCMyPathFinder->GetPath();
	bool displayedFoundPath = false;

	// keybindings
	EKeyCode buttonClose = Key_Escape;	// quit key
	EKeyCode autoStepButton = Key_A;	// auto step key
	EKeyCode singleStepButton = Key_Space;	// single step key

	/**** Set up your scene here ****/

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

	// set the skin of the start
	cubes[mapStart.second][mapStart.first]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::start]);
	map[mapStart.second][mapStart.first] = cubeTypes::start;

	// set the skin of the end
	cubes[mapEnd.second][mapEnd.first]->SetSkin(cubeSkins[cubeStatus::unknown][cubeTypes::end]);
	map[mapEnd.second][mapEnd.first] = cubeTypes::end;

	// mob
	IModel* mob = mobMesh->CreateModel();
	mob->Scale(cubeSize);
	mob->SetPosition(mapStart.first * cubeSize, 0.0f, mapStart.second * cubeSize);


	// Camera
	const int MY_CAMERA_SPEED = 10;

	// myCamera
	ICamera* myCamera = pMyEngine->CreateCamera(kManual);
	myCamera->SetPosition(cubeSize * 4.5f, 100.0f, cubeSize * 4.5f);
	myCamera->RotateX(90.0f);
	myCamera->SetMovementSpeed(2.0f * MY_CAMERA_SPEED);
	myCamera->SetRotationSpeed(MY_CAMERA_SPEED);

	// hide mouse
	//pMyEngine->StartMouseCapture();

	// The main game loop, repeat until engine is stopped
	while (pMyEngine->IsRunning())
	{
		// Draw the scene
		pMyEngine->DrawScene();

		// Get frame time
		frameTimer = pMyEngine->Timer();
		if (autoStep)
			timeLeftForNextStep -= frameTimer;

		if (timeLeftForNextStep < 0.0f && !displayedFoundPath)
		{
			timeLeftForNextStep = TIME_BETWEEN_STEPS;
			++currentPoint;
			pair<int, int> currentWaypath;

			if (currentPoint ==-1) // starting point
				currentWaypath = mapStart;
			else
				currentWaypath = waypoints[currentPoint];

			cubes[currentWaypath.second][currentWaypath.first]->SetSkin(cubeSkins[cubeStatus::visited][map[currentWaypath.second][currentWaypath.first] % cubeTypes::numOfCubeTypes]);
			map[currentWaypath.second][currentWaypath.first] += cubeTypes::numOfCubeTypes * 2;

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

				// if wall ignore
				if (map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes == cubeTypes::wall)
					continue;

				if (tmp == mapEnd) // found the goal
				{
					displayedFoundPath = true;
					cubes[tmp.second][tmp.first]->SetSkin(cubeSkins[cubeStatus::visited][map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes]);
					map[tmp.second][tmp.first] += cubeTypes::numOfCubeTypes * 2;
				}

				// if not visited set visited
				if (map[tmp.second][tmp.first] / cubeTypes::numOfCubeTypes < 1)
				{
					cubes[tmp.second][tmp.first]->SetSkin(cubeSkins[cubeStatus::seen][map[tmp.second][tmp.first] % cubeTypes::numOfCubeTypes]);
					map[tmp.second][tmp.first] += cubeTypes::numOfCubeTypes;
				}
			}
		}

		/**** Update your scene each frame here ****/
		if (pMyEngine->KeyHit(autoStepButton))
		{
			autoStep = !autoStep;
		}
		if (pMyEngine->KeyHit(singleStepButton))
		{
			
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
