#ifndef JANBENDER_DYNAMICS_H
#define JANBENDER_DYNAMICS_H

#include "Util\BasicTypes.h"
#include "Model\SurfaceMeshObject.h"
#include "Constraints.h"

#include <Eigen/Dense>

//class SimulationModel;


class JanBenderDynamics
{
public:
	JanBenderDynamics(SurfaceMeshObject * clothPiece) :
		m_clothPiece(clothPiece)
	{
		initial(1.0f);
		addPermanentConstraints();
	}

	~JanBenderDynamics() {}

	void addRigidBody(SurfaceMeshObject * const surfaceMeshObject);

	void userSet();

	void stepforward(float timeStep);

	void exportCollisionVertices(GLfloat *& buffer, GLuint *& type, GLuint & size);
	
private:
	SurfaceMeshObject * const m_clothPiece;
	std::list<SurfaceMeshObject *> m_rigidBodies;

	//std::vector<SurfaceMeshObject *> m_rigidBodies;

	std::list<Constraint *> m_permanentConstraints;
	std::list<Constraint *> m_temporaryConstraints;

	unsigned int m_iterCount = 8;

	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_lastPositions;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_currentPositions;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_predictPositions;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_vertexVelocities;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_planarCoordinates;

	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_vertexEigenNormals;
	SurfaceMesh3f::Property_map<Faceidx, Eigen::Vector3f> m_faceEigenNormals;

	SurfaceMesh3f::Property_map<Veridx, float> m_vertexInversedMasses;
	SurfaceMesh3f::Property_map<Veridx, float> m_vertexMasses;
		
	Eigen::Vector3f f_ext = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
	
	void initial(float density);

	void addPermanentConstraints();

	void freeForward(float timeStep);

	void genCollConstraints();

	void projectConstraints(int iterCount);

	void updateStates(float timeStep);

	void velocityUpdate();

	void writeBack();
};


#endif
