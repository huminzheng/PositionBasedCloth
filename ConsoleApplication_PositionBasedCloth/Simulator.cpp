#include "Simulator.h"
#include "Render\Screen.h"
#include "Render\Scene.h"
#include "Render\ResourceManager.h"
#include "Util\Config.h"
#include "Render\EventManager.h"

void Simulator::run()
{
	while (!Screen::closed())
	{
		{
			EventManager::active(eventManager);
			Screen::pullEvents();
			eventManager->handleEvents();
			if (!clock->paused())
			{
				updateData();
			}
		}
		Scene::renderScene();
		Screen::swapBuffers();
	}
}

void Simulator::init()
{
	Screen::initEnv();
	loopCount = 0u;

	// ---------- Setup and compile our shaders -------------------
	ResourceManager::LoadShader("model_loading", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\model_loading.frag");
	ResourceManager::LoadShader("background_cube", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\background_cube.frag");
	ResourceManager::LoadShader("cloth_piece", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece.frag");
	ResourceManager::LoadShader("rigid_body", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\rigid_body.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\rigid_body.frag");
	ResourceManager::LoadShader("cloth_piece_normal", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.frag", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\cloth_piece_debug.gs");
	ResourceManager::LoadShader("bounding_box", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.frag", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\bounding_box.gs");
	ResourceManager::LoadShader("contact_point", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\contact_point.vs", "E:\\Microsoft Visual Studio 2015\\Workspace\\ConsoleApplication_BaraffBridson\\ConsoleApplication_BaraffBridson\\contact_point.frag");
	//ResourceManager::LoadShader("model_loading", ".\\model_loading.vs", ".\\model_loading.frag");
	//ResourceManager::LoadShader("background_cube", ".\\background_cube.vs", ".\\background_cube.frag");
	
	// ----------- load cube map ----------------
	std::vector<const GLchar*> faces;
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\top.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\bottom.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	ResourceManager::LoadCubeMap("background_texture", faces);

	// ----------- Load cloth model ----------------
	Model ourModel((GLchar *)Config::modelPath.c_str(), 
		(aiPostProcessSteps)(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices));
	
	clothPiece = new SurfaceMeshObject(3);
	clothPiece->import(ourModel.getMeshes()[0]);
	//clothPiece->useVTexCoord2DAsVPlanarCoord3f();
	//clothPiece->addPositionsProperty();
	
	// initial planar coordinates
	//clothDynamics = new BaraffDynamics(clothPiece);

	jbDynamics = new JanBenderDynamics(clothPiece);

	auto env = new SceneEnv(ResourceManager::GetShader("background_cube"),
		ResourceManager::GetCubeMap("background_texture"), viewer->getCamera());
	Scene::add_component(env);
	auto clo = new SceneClothPiece(ResourceManager::GetShader("cloth_piece"),
		ResourceManager::GetShader("cloth_piece_normal"),
		clothPiece, viewer->getCamera());
	Scene::add_component(clo);
	//clothPieceBoxSceneIndex = Scene::add_component(new SceneAABBox(ResourceManager::GetShader("bounding_box"),
	//	viewer->getCamera()));

	// ----------- Load rigid model ----------------
	Model rigidBodyModel((GLchar *)Config::spherePath.c_str(),
		(aiPostProcessSteps)(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices));
	rigidBody = new SurfaceMeshObject(3);
	rigidBody->import(rigidBodyModel.getMeshes()[0]);
	//rigidBody->addPositionsProperty();

	auto sph = new SceneRigidBody(ResourceManager::GetShader("rigid_body"),
		rigidBody, viewer->getCamera());
	Scene::add_component(sph);

	contactSceneIndex = Scene::add_component(new SceneContact(
		ResourceManager::GetShader("bounding_box"), ResourceManager::GetShader("contact_point"),
		viewer->getCamera()));

	Scene::load();

	// ------------ register event handlers ------------
	eventManager = new EventManager(Screen::window);
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->viewer->keyboard_press(keyMask); });
	eventManager->registerMouseEventHandler([this](GLfloat xpos, GLfloat ypos, GLfloat xlast, GLfloat ylast) -> void {this->viewer->move_mouse(xpos, ypos, xlast, ylast); });
	eventManager->registerScrollEventHandler([this](GLfloat scrollX, GLfloat scrollY) -> void {this->viewer->scroll_mouse(scrollX, scrollY); });
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->pauseEventHandle(keyMask); });

}

void Simulator::updateData()
{
	
	jbDynamics->userSet();
	jbDynamics->stepforward(0.03f);

	GLfloat * buffer = nullptr;
	GLuint size = 0;
	jbDynamics->exportCollisionVertices(buffer, size);

	//contactHandler = new OtaduyContact(clothPiece, rigidBody);
	//contactHandler->pointTriangleDetection(0.1f);
	//contactHandler->edgeEdgeDetection(0.1f);
	(dynamic_cast<SceneContact *>(Scene::get_component(contactSceneIndex)))
		->setContacts(buffer, size);

}

void Simulator::pauseEventHandle(bool const * const keyMask)
{
	if (keyMask[GLFW_KEY_Z])
	{
		clock->pause();
	}
	if (keyMask[GLFW_KEY_X])
	{
		clock->resume();
	}
}
