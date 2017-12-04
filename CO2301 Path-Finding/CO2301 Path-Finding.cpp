// CO2301 Path-Finding.cpp: A program using the TL-Engine

#include <TL-Engine.h>	// TL-Engine include file and namespace
#include <vector> // vectors
using namespace tle;

void main()
{
    // Create a 3D engine (using TLX engine here) and open a window for it
    I3DEngine* myEngine = New3DEngine(kTLX);
    myEngine->StartWindowed();

    // Add default folder for meshes and other media
    myEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");

    /**** Set up your scene here ****/
    vector<vector<int>> map; // the current map


    // The main game loop, repeat until engine is stopped
    while (myEngine->IsRunning())
    {
        // Draw the scene
        myEngine->DrawScene();

        /**** Update your scene each frame here ****/

    }

    // Delete the 3D engine now we are finished with it
    myEngine->Delete();
}
