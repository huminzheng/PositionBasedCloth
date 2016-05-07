#ifndef SCENE_H
#define SCENE_H

#include "Shader.h"
#include "texture.h"
#include "Camera.h"
#include "..\Model\SurfaceMeshObject.h"
#include "..\ContactDetection.h"

#include <vector>

class SceneComponent;

class Scene
{
public:
	typedef int Index;

	static Index add_component(SceneComponent * com);

	static void erase_component(Index index);
	
	static SceneComponent * get_component(Index index)
	{
		assert(index >= 0 && index <= render_list.size());
		return render_list[index];
	}

	static void load();

	static void renderScene()
	{
		update();
		draw();
	}

private:
	explicit Scene() {}

	static std::vector<SceneComponent *> render_list;

	static void draw();

	static void update();

};

class SceneComponent
{
public:
	virtual void draw() const = 0;

	virtual void load() = 0;

	virtual void update() = 0;

	virtual ~SceneComponent() {}

};


class SceneEnv : public SceneComponent
{
public:

	SceneEnv(Shader * shader, CubeMap const * cubeMap, Camera * camera) :
		SceneComponent(), cubeMap(cubeMap), shader(shader), camera(camera)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneEnv() {}

protected:

	CubeMap const * cubeMap;
	Shader  * shader;
	Camera * camera;

	glm::mat4 projection;
	glm::mat4 view;
	GLuint vao, vbo;

	GLfloat const cubeVertices[108] = {
		// Positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f,  1.0f
	};
};

class SceneClothPiece : public SceneComponent
{
public:

	SceneClothPiece(Shader * clothPieceShader, Shader * debugShader, SurfaceMeshObject const * const clothPiece, Camera * camera) :
		SceneComponent(), clothPiece(clothPiece), camera(camera),
		clothPieceShader(clothPieceShader), debugShader(debugShader)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneClothPiece() {}

protected:
	SurfaceMeshObject const * const clothPiece;

	Camera * camera;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 model;
	glm::vec3 viewPos;

	/* cloth piece surface */
	Shader  * clothPieceShader = nullptr;
	Shader  * debugShader = nullptr;
	
	GLfloat * meshVB = nullptr;
	GLfloat * meshVNormalB = nullptr;
	GLuint * meshEB = nullptr;
	GLuint meshVBcnt = 0, meshEBcnt = 0;

	GLfloat * conditionBuffer = nullptr;
	GLuint conditionCnt = 0;

	GLuint meshVAO, meshVBO, meshVNormalBO, meshEBO, conditionVBO;

	/* cloth piece normal */
	GLboolean drawNormal = false;
	GLuint debugVAO, debugVBO, debugNormalVBO;
	GLfloat * fBarycentreBuffer = nullptr;
	GLfloat * fNormalBuffer = nullptr;
	GLuint fSize;

	//GLfloat * tempptr;
	//GLuint tempuint;
	//stepforward->exportBendConditionData(tempptr, tempuint);



};

class SceneRigidBody: public SceneComponent
{
public:

	SceneRigidBody(Shader * shader, SurfaceMeshObject const * const rigidBody, Camera * camera) :
		SceneComponent(), rigidBody(rigidBody), camera(camera), shader(shader)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneRigidBody() {}

protected:
	SurfaceMeshObject const * const rigidBody;

	Camera * camera;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 model;
	glm::vec3 viewPos;

	/* surface */
	Shader  * shader = nullptr;

	GLfloat * meshVB = nullptr;
	GLfloat * meshVNormalB = nullptr;
	GLuint * meshEB = nullptr;
	GLuint meshVBcnt = 0, meshEBcnt = 0;

	GLuint meshVAO, meshVBO, meshVNormalBO, meshEBO, conditionVBO;

};

class OtaduyContact;

class SceneContact: public SceneComponent
{
public:

	SceneContact(Shader * boxShader, Shader * pointShader, Camera * camera) :
		SceneComponent(), camera(camera),
		boxShader(boxShader), pointShader(pointShader)
	{}

	virtual void draw() const;

	virtual void load();

	virtual void update();

	virtual ~SceneContact() {}

	void setContacts(GLfloat * points, GLuint * types, GLuint pointCount)
	{
		this->pointVerticesBuffer = points;
		this->pointTypesBuffer = types;
		this->pointVerticesCount = pointCount;
	}

private:

	void * contacts = nullptr;

	Shader * boxShader;
	Shader * pointShader;

	Camera * camera;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 model;

	GLboolean drawTrees = false;
	
	GLuint treeVAO0, treeVBO0;
	GLuint treeVAO1, treeVBO1;

	GLfloat * boxTreeVerticesBuffer0;
	GLuint treeVerticesCount0;
	GLfloat * boxTreeVerticesBuffer1;
	GLuint treeVerticesCount1;

	GLuint pointVAO, pointVBO, pointTypesVBO;
	GLuint edgeVAO, edgeVBO;

	GLfloat * pointVerticesBuffer;
	GLuint * pointTypesBuffer;
	GLuint pointVerticesCount = 0;

	GLfloat * edgeVerticesBuffer;
	GLuint edgeVerticesCount = 0;

};

#endif
