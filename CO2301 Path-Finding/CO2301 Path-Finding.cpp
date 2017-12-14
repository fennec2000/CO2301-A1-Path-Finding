// CO2301 Path-Finding.cpp: A program using the TL-Engine

#include <TL-Engine.h>	// TL-Engine include file and namespace
#include "PathFinder.h" // path finder class
using namespace tle;

enum cubeTypes { wall, clear, wood, water, start, end, numOfCubeTypes };

void main()
{
	// Create a 3D engine (using TLX engine here) and open a window for it
	I3DEngine* pMyEngine = New3DEngine(kTLX);
	pMyEngine->StartWindowed();

	// Add default folder for meshes and other media
	pMyEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");
	pMyEngine->AddMediaFolder("media");

	// Varables
	// cubes
	string cubeSkins[] = { "wall.jpg", "clear.jpg", "wood.jpg", "water.jpg", "start.jpg", "end.jpg" };
	float cubeYOffset[] = { 0.0f, -5.0f, -4.5f, -5.5f };
	float cubeSize = 10.0f;

	// keybindings
	EKeyCode buttonClose = Key_Escape; // quit key

	// Camera
	const int MY_CAMERA_SPEED = 10;

	/**** Set up your scene here ****/
	// Engine
	CPathFinder* pCMyPathFinder = new CPathFinder("m");

	// Meshs
	IMesh* floorMesh = pMyEngine->LoadMesh("Floor.x");
	IMesh* cubeMesh = pMyEngine->LoadMesh("Cube.x");
	IMesh* mobMesh = pMyEngine->LoadMesh("sierra.x");

	// Models
	IModel* floor = floorMesh->CreateModel(0.0f, -10.0f, 0.0f);

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
				tmpCubeModel->SetSkin(cubeSkins[cubeTypes::wall]);
				break;
			case cubeTypes::clear:
				tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, cubeYOffset[cubeTypes::clear], i * cubeSize);
				tmpCubeModel->SetSkin(cubeSkins[cubeTypes::clear]);
				break;
			case cubeTypes::wood:
				tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, cubeYOffset[cubeTypes::wood], i * cubeSize);
				tmpCubeModel->SetSkin(cubeSkins[cubeTypes::wood]);
				break;
			case cubeTypes::water:
				tmpCubeModel = cubeMesh->CreateModel(j * cubeSize, cubeYOffset[cubeTypes::water], i * cubeSize);
				tmpCubeModel->SetSkin(cubeSkins[cubeTypes::water]);
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
	cubes[mapStart.second][mapStart.first]->SetSkin(cubeSkins[cubeTypes::start]);

	// set the skin of the end
	cubes[mapEnd.second][mapEnd.first]->SetSkin(cubeSkins[cubeTypes::end]);

	// Mob
	IModel* mob = mobMesh->CreateModel();
	mob->Scale(cubeSize);
	mob->SetPosition(mapStart.first * cubeSize, 0.0f, mapStart.second * cubeSize);

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

		/**** Update your scene each frame here ****/

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
