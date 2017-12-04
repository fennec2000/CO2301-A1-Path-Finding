// CO2301 Path-Finding.cpp: A program using the TL-Engine

#include <TL-Engine.h>	// TL-Engine include file and namespace
using namespace tle;

void main()
{
	// Create a 3D engine (using TLX engine here) and open a window for it
	I3DEngine* pMyEngine = New3DEngine(kTLX);
	pMyEngine->StartWindowed();

	// Add default folder for meshes and other media
	pMyEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");

	/**** Set up your scene here ****/


	// The main game loop, repeat until engine is stopped
	while (pMyEngine->IsRunning())
	{
		// Draw the scene
		pMyEngine->DrawScene();

		/**** Update your scene each frame here ****/

	}

	// Delete the 3D engine now we are finished with it
	pMyEngine->Delete();
}
