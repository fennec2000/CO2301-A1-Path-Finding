// CO2301 Path-Finding.cpp: A program using the TL-Engine

#include <TL-Engine.h>	// TL-Engine include file and namespace
#include "PathFinder.h" // path finder class
using namespace tle;

void main()
{
	// Create a 3D engine (using TLX engine here) and open a window for it
	I3DEngine* pMyEngine = New3DEngine(kTLX);
	pMyEngine->StartWindowed();

	// Add default folder for meshes and other media
	pMyEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");

	// Varables

	// keybindings
	EKeyCode buttonClose = Key_Escape; // quit key

	// camera
	const int MY_CAMERA_SPEED = 10;

	/**** Set up your scene here ****/
	CPathFinder* pCMyPathFinder = new CPathFinder("m");

	// meshs
	IMesh* floorMesh = pMyEngine->LoadMesh("Floor.x");
	IMesh* cubeMesh = pMyEngine->LoadMesh("Cube.x");

	// Models
	IModel* floor = floorMesh->CreateModel();

	// multi vector of cubes
	pair<int, int> mapSize = pCMyPathFinder->GetMapSize();
	vector <vector <IModel*>> cubes;
	vector <IModel*> tmpCubes;

	float cubeYOffset = -4.9f;
	float cubeSize = 10.0f;

	for (int i = 0; i < mapSize.second; ++i)
	{
		for (int j = 0; j < mapSize.first; ++j)
		{
			tmpCubes.push_back(cubeMesh->CreateModel(j * cubeSize, cubeYOffset, i * cubeSize));
		}
		cubes.push_back(tmpCubes);
	}

	// myCamera
	ICamera* myCamera = pMyEngine->CreateCamera(kFPS);
	myCamera->SetMovementSpeed(2.0f * MY_CAMERA_SPEED);
	myCamera->SetRotationSpeed(MY_CAMERA_SPEED);

	// hide mouse
	pMyEngine->StartMouseCapture();

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
