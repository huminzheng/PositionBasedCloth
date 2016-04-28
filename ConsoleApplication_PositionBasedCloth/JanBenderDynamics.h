#ifndef JANBENDER_DYNAMICS_H
#define JANBENDER_DYNAMICS_H

#include "Util\BasicTypes.h"
#include "Model\SurfaceMeshObject.h"

#include <Eigen/Dense>

//class SimulationModel;

class Constraint
{
public:
	unsigned int const m_numberOfBodies;
	/** indices of the linked bodies */
	//unsigned int *m_bodies;
	float m_stiff = 1.0f;

	Constraint(const unsigned int numberOfBodies) :
		m_numberOfBodies(numberOfBodies)
	{
		//m_bodies = new unsigned int[numberOfBodies];
	}

	virtual ~Constraint() 
	{
		//delete[] m_bodies; 
	}

	virtual int &getTypeId() const = 0;
	virtual void setStiff(float stiff)
	{
		m_stiff = stiff;
	}

	virtual bool updateConstraint() { return true; };
	virtual bool solvePositionConstraint() { return true; };
	virtual bool solveVelocityConstraint() { return true; };

};

//class BallJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 4> m_jointInfo;
//
//	BallJoint() : Constraint(2) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class BallOnLineJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 10> m_jointInfo;
//
//	BallOnLineJoint() : Constraint(2) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class HingeJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 12> m_jointInfo;
//
//	HingeJoint() : Constraint(2) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class UniversalJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 8> m_jointInfo;
//
//	UniversalJoint() : Constraint(2) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class SliderJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//
//	SliderJoint() : Constraint(2) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class TargetPositionMotorSliderJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetPosition;
//
//	TargetPositionMotorSliderJoint() : Constraint(2) { m_targetPosition = 0.0f; }
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	float getTargetPosition() const { return m_targetPosition; }
//	void setTargetPosition(const float val) { m_targetPosition = val; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class TargetVelocityMotorSliderJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetVelocity;
//
//	TargetVelocityMotorSliderJoint() : Constraint(2) { m_targetVelocity = 0.0f; }
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	float getTargetVelocity() const { return m_targetVelocity; }
//	void setTargetVelocity(const float val) { m_targetVelocity = val; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//	virtual bool solveVelocityConstraint(SimulationModel &model);
//};
//
//class TargetAngleMotorHingeJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetAngle;
//	TargetAngleMotorHingeJoint() : Constraint(2) { m_targetAngle = 0.0f; }
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	float getTargetAngle() const { return m_targetAngle; }
//	void setTargetAngle(const float val)
//	{
//		const float pi = (float)MATH_PI;
//		m_targetAngle = std::max(val, -pi);
//		m_targetAngle = std::min(m_targetAngle, pi);
//	}
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class TargetVelocityMotorHingeJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetAngularVelocity;
//	TargetVelocityMotorHingeJoint() : Constraint(2) { m_targetAngularVelocity = 0.0f; }
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	float getTargetAngularVelocity() const { return m_targetAngularVelocity; }
//	void setTargetAngularVelocity(const float val) { m_targetAngularVelocity = val; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//	virtual bool solveVelocityConstraint(SimulationModel &model);
//};
//
//class RigidBodyParticleBallJoint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix<float, 3, 2> m_jointInfo;
//
//	RigidBodyParticleBallJoint() : Constraint(2) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex, const unsigned int particleIndex);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

class DistanceConstraint : public Constraint
{
public:
	static int TYPE_ID;

	float m_restLength;

	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & m_geodesicMap;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & m_posMap;
	SurfaceMesh3f::Property_map<Veridx, float> & m_invMassMap;
	Veridx m_v1, m_v2;

	DistanceConstraint(SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & geodesicMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & posMap,
		SurfaceMesh3f::Property_map<Veridx, float> & invMassMap,
		Veridx v1, Veridx v2) :
		Constraint(2), m_geodesicMap(geodesicMap), m_posMap(posMap), m_invMassMap(invMassMap),
		m_v1(v1), m_v2(v2)
	{
		initConstraint();
	}
	
	virtual int &getTypeId() const { return TYPE_ID; }

	virtual bool initConstraint();
	virtual bool solvePositionConstraint();
};

//class DihedralConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	float m_restAngle;
//
//	DihedralConstraint() : Constraint(4) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

//class IsometricBendingConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix4f m_Q;
//
//	IsometricBendingConstraint() : Constraint(4) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SurfaceMesh3f &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

//class FEMTriangleConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	float m_area;
//	Eigen::Matrix2f m_invRestMat;
//
//	FEMTriangleConstraint() : Constraint(3) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class StrainTriangleConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix2f m_invRestMat;
//
//	StrainTriangleConstraint() : Constraint(3) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class VolumeConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	float m_restVolume;
//
//	VolumeConstraint() : Constraint(4) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class FEMTetConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	float m_volume;
//	Eigen::Matrix3f m_invRestMat;
//
//	FEMTetConstraint() : Constraint(4) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class StrainTetConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Matrix3f m_invRestMat;
//
//	StrainTetConstraint() : Constraint(4) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class ShapeMatchingConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//	Eigen::Vector3f m_restCm;
//	Eigen::Matrix3f m_invRestMat;
//	float *m_w;
//	Eigen::Vector3f *m_x0;
//	Eigen::Vector3f *m_x;
//	Eigen::Vector3f *m_corr;
//	unsigned int *m_numClusters;
//
//	ShapeMatchingConstraint(const unsigned int numberOfParticles) : Constraint(numberOfParticles)
//	{
//		m_x = new Eigen::Vector3f[numberOfParticles];
//		m_x0 = new Eigen::Vector3f[numberOfParticles];
//		m_corr = new Eigen::Vector3f[numberOfParticles];
//		m_w = new float[numberOfParticles];
//		m_numClusters = new unsigned int[numberOfParticles];
//	}
//	virtual ~ShapeMatchingConstraint()
//	{
//		delete[] m_x;
//		delete[] m_x0;
//		delete[] m_corr;
//		delete[] m_w;
//		delete[] m_numClusters;
//	}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particleIndices[], const unsigned int numClusters[]);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

//class VertexTriangleDistanceConstraint : public Constraint
//{
//public:
//	static int TYPE_ID;
//
//	enum { POINTSIDE, TRIANGLESIDE, TWOSIDE };
//
//	unsigned int vobj;
//	Veridx vidx;
//	
//	unsigned int fobj;
//	Faceidx fidx;
//
//	VertexTriangleDistanceConstraint() : Constraint(4) {}
//	virtual int &getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

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

	void userSet();

	void stepforward(float timeStep);
	
private:
	SurfaceMeshObject * const m_clothPiece;

	std::list<Constraint *> m_constraints;
	unsigned int m_iterCount = 8;

	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_lastPositions;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_currentPositions;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_predictPositions;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_vertexVelocities;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> m_planarCoordinates;

	SurfaceMesh3f::Property_map<Veridx, Vec3f> m_vertexNormals;
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> m_faceNormals;

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
