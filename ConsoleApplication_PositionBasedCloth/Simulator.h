#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "BaraffDynamics.h"
#include "JanBenderDynamics.h"
#include "Render\Scene.h"
#include "Render\EventManager.h"
#include "Render\Camera.h"
#include "Util\Clock.h"

class Simulator
{
public:

	explicit Simulator():
		viewer(new FOVControl(glm::vec3(0.0f, 0.0f, -20.0f), glm::vec3(0.0f, 1.0f, 0.0f), 90.0f)),
		clock(new Clock())
	{} 

	// WARNING: should be called explicitly
	void init();

	void run();

private:

	SurfaceMeshObject * clothPiece;
	SurfaceMeshObject * rigidBody;
	BaraffDynamics * clothDynamics;
	OtaduyContact * contactHandler;
	
	JanBenderDynamics * jbDynamics;

	EventManager * eventManager;
	FOVControl * viewer;
	Clock * clock;

	GLuint loopCount;
	Scene::Index contactSceneIndex;

	void updateData();

	void pauseEventHandle(bool const * const keyMask);

};

#endif