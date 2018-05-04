#include "Simulator.h"
#include "Render\Screen.h"
#include "Render\Scene.h"
#include "Render\ResourceManager.h"
#include "Util\Config.h"
#include "Render\EventManager.h"

//#define USE_RIGIDBODY

void Simulator::run()
{
	while (!Screen::closed())
	{
		Clock::Instance()->Tick(1.0f);
		{
			EventManager::active(eventManager);
			Screen::pullEvents();
			eventManager->handleEvents();
			if (!Clock::Instance()->paused())
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

	config = new Config(".\\Config.json");

	// ---------- Setup and compile our shaders -------------------
	ResourceManager::LoadShader("model_loading", ".\\GLSL\\model_loading.vs", ".\\GLSL\\model_loading.frag");
	ResourceManager::LoadShader("background_cube", ".\\GLSL\\background_cube.vs", ".\\GLSL\\background_cube.frag");
	ResourceManager::LoadShader("cloth_piece", ".\\GLSL\\cloth_piece.vs", ".\\GLSL\\cloth_piece.frag");
	ResourceManager::LoadShader("rigid_body", ".\\GLSL\\rigid_body.vs", ".\\GLSL\\rigid_body.frag");
	ResourceManager::LoadShader("cloth_piece_normal", ".\\GLSL\\cloth_piece_debug.vs", ".\\GLSL\\cloth_piece_debug.frag", ".\\GLSL\\cloth_piece_debug.gs");
	ResourceManager::LoadShader("bounding_box", ".\\GLSL\\bounding_box.vs", ".\\GLSL\\bounding_box.frag", ".\\GLSL\\bounding_box.gs");
	ResourceManager::LoadShader("contact_point", ".\\GLSL\\contact_point.vs", ".\\GLSL\\contact_point.frag");
	ResourceManager::LoadShader("cloth", ".\\GLSL\\cloth_vs.glsl", ".\\GLSL\\cloth_frag.glsl");
	//ResourceManager::LoadShader("model_loading", ".\\model_loading.vs", ".\\model_loading.frag");
	//ResourceManager::LoadShader("background_cube", ".\\background_cube.vs", ".\\background_cube.frag");
	
	// ----------- load cube map ----------------
	//std::vector<const GLchar*> faces;
	//faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	//faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	//faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\top.jpg");
	//faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\bottom.jpg");
	//faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	//faces.push_back("E:\\Computer Graphics\\Materials\\CubeMaps\\background01\\side.jpg");
	//ResourceManager::LoadCubeMap("background_texture", faces);

	// ----------- Load cloth model ----------------
	Model ourModel((GLchar *)config->modelPath.c_str(), 
		aiPostProcessSteps(aiProcess_Triangulate | /*aiProcess_FlipUVs | */aiProcess_JoinIdenticalVertices | aiProcess_RemoveComponent),
		aiComponent(aiComponent_NORMALS));

	std::array<float, 16> matdata = readData<float, 16>((GLchar *)config->matrixPath.c_str());
	
	clothPiece = new SurfaceMeshObject(3);
	clothPiece->import(ourModel.getMeshes()[0], matdata);
	clothPiece->refreshNormals();

	//auto env = new SceneEnv(ResourceManager::GetShader("background_cube"),
	//	ResourceManager::GetCubeMap("background_texture"), viewer->getCamera());
	//Scene::add_component(env);

	auto clo = new SceneClothPiece(ResourceManager::GetShader("cloth"),
		ResourceManager::GetShader("cloth_piece_normal"),
		clothPiece, viewer->getCamera());
	Scene::add_component(clo);
	//clothPieceBoxSceneIndex = Scene::add_component(new SceneAABBox(ResourceManager::GetShader("bounding_box"),
	//	viewer->getCamera()));

	// ----------- Load rigid model ----------------
#ifdef USE_RIGIDBODY
	Model rigidBodyModel(
		(GLchar *)config->rigidBodyPath.c_str(),
		aiPostProcessSteps(aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices), 
		aiComponent(0x00));
	rigidBody = new SurfaceMeshObject(3);
	rigidBody->import(rigidBodyModel.getMeshes()[0]);
	rigidBody->refreshNormals();


	auto sph = new SceneRigidBody(ResourceManager::GetShader("rigid_body"),
		rigidBody, viewer->getCamera());
	Scene::add_component(sph);

	contactSceneIndex = Scene::add_component(new SceneContact(
		ResourceManager::GetShader("bounding_box"), ResourceManager::GetShader("contact_point"),
		viewer->getCamera()));
#endif


	// ----------- Load dynamics ----------------
	jbDynamics = new JanBenderDynamics(clothPiece, config->parameters);
#ifdef USE_RIGIDBODY
	jbDynamics->addRigidBody(rigidBody);
#endif	

	// ----------- Load scene ----------------
	Scene::load();

	// ------------ register event handlers ------------
	eventManager = new EventManager(Screen::window);
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->viewer->keyboard_press(keyMask); });
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {
		if (keyMask[GLFW_KEY_C])
		{
			Clock::Instance()->resume();
			Clock::Instance()->PushFrameCounter(new FrameCounter(40, []() { Clock::Instance()->pause(); }));
		}
	});
	eventManager->registerMouseEventHandler([this](GLfloat xpos, GLfloat ypos, GLfloat xlast, GLfloat ylast) -> void {this->viewer->move_mouse(xpos, ypos, xlast, ylast); });
	eventManager->registerScrollEventHandler([this](GLfloat scrollX, GLfloat scrollY) -> void {this->viewer->scroll_mouse(scrollX, scrollY); });
	eventManager->registerKeyboardEventHandler([this](bool const * const keyMask) -> void {this->pauseEventHandle(keyMask); });

}

void Simulator::updateData()
{
	
	jbDynamics->userSet();
	jbDynamics->stepforward(0.1f);

	GLfloat * buffer = nullptr;
	GLuint * typeBuffer = nullptr;
	GLuint size = 0;
	jbDynamics->exportCollisionVertices(buffer, typeBuffer, size);

	//contactHandler = new OtaduyContact(clothPiece, rigidBody);
	//contactHandler->pointTriangleDetection(0.1f);
	//contactHandler->edgeEdgeDetection(0.1f);
#ifdef USE_RIGIDBODY
	(dynamic_cast<SceneContact *>(Scene::get_component(contactSceneIndex)))
		->setContacts(buffer, typeBuffer, size);
#endif
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
