#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "Util\BasicTypes.h"
#include "Model\SurfaceMeshObject.h"

#include <Eigen/Dense>

enum ConstraintType
{
	BallJoint_Type,
	BallOnLineJoint_Type,
	HingeJoint_Type,
	UniversalJoint_Type,
	RigidBodyParticleBallJoint_Type,
	DistanceConstraint_Type,
	DihedralConstraint_Type,
	IsometricBendingConstraint_Type,
	FEMTriangleConstraint_Type,
	StrainTriangleConstraint_Type,
	VolumeConstraint_Type,
	FEMTetConstraint_Type,
	StrainTetConstraint_Type,
	ShapeMatchingConstraint_Type,
	TargetAngleMotorHingeJoint_Type,
	TargetVelocityMotorHingeJoint_Type,
	SliderJoint_Type,
	TargetPositionMotorSliderJoint_Type,
	TargetVelocityMotorSliderJoint_Type,
	EdgeEdgeDistanceConstraint_Type,
	VertexFaceDistanceConstraint_Type,
	VertexFaceSidedDistanceConstraint_Type,
	VertexFaceDirectedDistanceConstraint_Type
};

class Constraint
{
public:
	unsigned int const m_numberOfBodies;
	float m_stiff = 1.0f;

	Constraint(const unsigned int numberOfBodies) :
		m_numberOfBodies(numberOfBodies)
	{
	}

	virtual int const getTypeId() const = 0;
	virtual void setStiff(float stiff)
	{
		m_stiff = stiff;
	}

	virtual bool updateConstraint() { return true; };
	virtual bool solvePositionConstraint(float factor = 1.0f) { return true; };
	virtual bool solveVelocityConstraint() { return true; };

};

//class BallJoint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 4> m_jointInfo;
//
//	BallJoint() : Constraint(2) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class BallOnLineJoint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 10> m_jointInfo;
//
//	BallOnLineJoint() : Constraint(2) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class HingeJoint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 12> m_jointInfo;
//
//	HingeJoint() : Constraint(2) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class UniversalJoint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 8> m_jointInfo;
//
//	UniversalJoint() : Constraint(2) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class SliderJoint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//
//	SliderJoint() : Constraint(2) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class TargetPositionMotorSliderJoint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetPosition;
//
//	TargetPositionMotorSliderJoint() : Constraint(2) { m_targetPosition = 0.0f; }
//	virtual int const getTypeId() const { return TYPE_ID; }
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
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetVelocity;
//
//	TargetVelocityMotorSliderJoint() : Constraint(2) { m_targetVelocity = 0.0f; }
//	virtual int const getTypeId() const { return TYPE_ID; }
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
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetAngle;
//	TargetAngleMotorHingeJoint() : Constraint(2) { m_targetAngle = 0.0f; }
//	virtual int const getTypeId() const { return TYPE_ID; }
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
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 14> m_jointInfo;
//	float m_targetAngularVelocity;
//	TargetVelocityMotorHingeJoint() : Constraint(2) { m_targetAngularVelocity = 0.0f; }
//	virtual int const getTypeId() const { return TYPE_ID; }
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
//	static int const TYPE_ID;
//	Eigen::Matrix<float, 3, 2> m_jointInfo;
//
//	RigidBodyParticleBallJoint() : Constraint(2) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	bool initConstraint(SimulationModel &model, const unsigned int rbIndex, const unsigned int particleIndex);
//	virtual bool updateConstraint(SimulationModel &model);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

class DistanceConstraint : public Constraint
{
public:
	enum { TYPE_ID = ConstraintType::DistanceConstraint_Type
	};

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

	virtual int const getTypeId() const { return TYPE_ID; }

	virtual bool initConstraint();
	virtual bool solvePositionConstraint(float factor = 1.0f) override;
};

//class DihedralConstraint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	float m_restAngle;
//
//	DihedralConstraint() : Constraint(4) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

class IsometricBendingConstraint : public Constraint
{
public:
	enum { TYPE_ID = ConstraintType::IsometricBendingConstraint_Type
	};

	Eigen::Matrix4f m_Q;

	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & m_posMap;
	SurfaceMesh3f::Property_map<Veridx, float> & m_invMassMap;
	Veridx m_v1, m_v2, m_v3, m_v4;

	IsometricBendingConstraint(SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & posMap,
		SurfaceMesh3f::Property_map<Veridx, float> & invMassMap,
		Veridx v1, Veridx v2, Veridx v3, Veridx v4, 
		float stiff) :
		Constraint(4), m_posMap(posMap), m_invMassMap(invMassMap),
		m_v1(v1), m_v2(v2), m_v3(v3), m_v4(v4)
	{
		initConstraint();
		m_stiff = stiff;
	}

	virtual int const getTypeId() const { return TYPE_ID; }

	virtual bool initConstraint();
	virtual bool solvePositionConstraint(float factor = 1.0f) override;
};

class FEMTriangleConstraint : public Constraint
{
public:
	enum { TYPE_ID = ConstraintType::FEMTriangleConstraint_Type
	};

	float m_area;
	Eigen::Matrix2f m_invRestMat;

	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & m_posMap;
	SurfaceMesh3f::Property_map<Veridx, float> & m_invMassMap;
	Veridx m_v1, m_v2, m_v3;
	
	float m_stiff_xx, m_stiff_yy, m_stiff_xy;
	float m_possion_ration_xy, m_possion_ration_yx;

	FEMTriangleConstraint(SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & posMap,
		SurfaceMesh3f::Property_map<Veridx, float> & invMassMap,
		Veridx v1, Veridx v2, Veridx v3,
		float stiff_xx, float stiff_yy, float stiff_xy,
		float possion_ration_xy, float possion_ration_yx) :
		Constraint(3), m_posMap(posMap), m_invMassMap(invMassMap),
		m_v1(v1), m_v2(v2), m_v3(v3),
		m_stiff_xx(stiff_xx), m_stiff_yy(stiff_yy), m_stiff_xy(stiff_xy),
		m_possion_ration_xy(possion_ration_xy), m_possion_ration_yx(possion_ration_yx)
	{
		initConstraint();
		m_stiff = 0.9f;
	}

	virtual int const getTypeId() const { return TYPE_ID; }

	virtual bool initConstraint();
	virtual bool solvePositionConstraint(float factor = 1.0f) override;
};

//class StrainTriangleConstraint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix2f m_invRestMat;
//
//	StrainTriangleConstraint() : Constraint(3) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class VolumeConstraint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	float m_restVolume;
//
//	VolumeConstraint() : Constraint(4) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class FEMTetConstraint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	float m_volume;
//	Eigen::Matrix3f m_invRestMat;
//
//	FEMTetConstraint() : Constraint(4) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class StrainTetConstraint : public Constraint
//{
//public:
//	static int const TYPE_ID;
//	Eigen::Matrix3f m_invRestMat;
//
//	StrainTetConstraint() : Constraint(4) {}
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//		const unsigned int particle3, const unsigned int particle4);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};
//
//class ShapeMatchingConstraint : public Constraint
//{
//public:
//	static int const TYPE_ID;
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
//	virtual int const getTypeId() const { return TYPE_ID; }
//
//	virtual bool initConstraint(SimulationModel &model, const unsigned int particleIndices[], const unsigned int numClusters[]);
//	virtual bool solvePositionConstraint(SimulationModel &model);
//};

class VertexFaceDistanceConstraint : public Constraint
{
public:
	enum { TYPE_ID = ConstraintType::VertexFaceDistanceConstraint_Type
	};

	float m_distance;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_vertexPosMap;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_vertexVelocityMap;
	SurfaceMesh3f::Property_map<Veridx, float> const m_vertexInvMassMap;
	Veridx m_v;
	bool m_vertexMove;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_facePosMap;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_faceVelocityMap;
	SurfaceMesh3f::Property_map<Veridx, float> const m_faceInvMassMap;
	Veridx m_fv1, m_fv2, m_fv3;
	bool m_faceMove;

	VertexFaceDistanceConstraint(SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const vertexPosMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const vertexVelocityMap,
		SurfaceMesh3f::Property_map<Veridx, float> const vertexInvMassMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const facePosMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const faceVelocityMap,
		SurfaceMesh3f::Property_map<Veridx, float> const faceInvMassMap,
		Veridx v, Veridx fv1, Veridx fv2, Veridx fv3,
		bool vertexMove, bool faceMove, float distance) :
		Constraint(4), 
		m_vertexPosMap(vertexPosMap), m_vertexVelocityMap(vertexVelocityMap), m_vertexInvMassMap(vertexInvMassMap),
		m_facePosMap(facePosMap), m_faceVelocityMap(faceVelocityMap), m_faceInvMassMap(faceInvMassMap),
		m_v(v), m_fv1(fv1), m_fv2(fv2), m_fv3(fv3),
		m_vertexMove(vertexMove), m_faceMove(faceMove), m_distance(distance)
	{}

	virtual int const getTypeId() const { return TYPE_ID; }

	virtual bool initConstraint();
	virtual bool solvePositionConstraint(float factor = 1.0f) override;
	virtual bool solveVelocityConstraint() override;
};

class VertexFaceSidedDistanceConstraint : public Constraint
{
public:
	enum { TYPE_ID = ConstraintType::VertexFaceSidedDistanceConstraint_Type
	};

	float m_distance, m_time;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_vertexPosMap;
	//SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & m_vertexNextPosMap;
	SurfaceMesh3f::Property_map<Veridx, float> const m_vertexInvMassMap;
	Veridx m_v;
	bool m_vertexMove;

	// BUG error with const reference of property map
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_facePosMap;
	//SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & m_faceNextPosMap;
	SurfaceMesh3f::Property_map<Veridx, float> const m_faceInvMassMap;
	Veridx m_fv1, m_fv2, m_fv3;
	bool m_faceMove;
	bool m_atRightSide;

	VertexFaceSidedDistanceConstraint(SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const vertexPosMap,
		//SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & vertexNextPosMap,
		SurfaceMesh3f::Property_map<Veridx, float> const vertexInvMassMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const facePosMap,
		//SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> & faceNextPosMap,
		SurfaceMesh3f::Property_map<Veridx, float> const faceInvMassMap,
		Veridx v, Veridx fv1, Veridx fv2, Veridx fv3,
		bool vertexMove, bool faceMove, bool forward,
		float distance, float time) :
		Constraint(4), 
		m_vertexPosMap(vertexPosMap), /*m_vertexNextPosMap(vertexNextPosMap), */m_vertexInvMassMap(vertexInvMassMap),
		m_facePosMap(facePosMap), /*m_faceNextPosMap(faceNextPosMap), */m_faceInvMassMap(faceInvMassMap),
		m_v(v), m_fv1(fv1), m_fv2(fv2), m_fv3(fv3),
		m_vertexMove(vertexMove), m_faceMove(faceMove), m_atRightSide(forward),
		m_distance(distance), m_time(time)
	{}

	virtual int const getTypeId() const { return TYPE_ID; }

	virtual bool initConstraint();
	virtual bool solvePositionConstraint(float factor = 1.0f) override;
};

class VertexFaceDirectedDistanceConstraint : public Constraint
{
public:
	enum { TYPE_ID = ConstraintType::VertexFaceDirectedDistanceConstraint_Type
	};

	float m_distance, m_time;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_vertexPosMap;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_vertexVelocityMap;
	SurfaceMesh3f::Property_map<Veridx, float> const m_vertexInvMassMap;
	Veridx m_v;

	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_facePosMap;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const m_faceVelocityMap;
	SurfaceMesh3f::Property_map<Faceidx, Eigen::Vector3f> const m_faceNormalMap;
	SurfaceMesh3f::Property_map<Veridx, float> const m_faceInvMassMap;
	Faceidx m_f;
	Veridx m_fv1, m_fv2, m_fv3;

	VertexFaceDirectedDistanceConstraint(
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const vertexPosMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const vertexVelocityMap,
		SurfaceMesh3f::Property_map<Veridx, float> const vertexInvMassMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const facePosMap,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const faceVelocityMap,
		SurfaceMesh3f::Property_map<Faceidx, Eigen::Vector3f> const faceNormalMap,
		SurfaceMesh3f::Property_map<Veridx, float> const faceInvMassMap,
		Veridx v, Faceidx f, Veridx fv1, Veridx fv2, Veridx fv3,
		float distance) :
		Constraint(4),
		m_vertexPosMap(vertexPosMap), m_vertexVelocityMap(vertexVelocityMap), m_vertexInvMassMap(vertexInvMassMap),
		m_facePosMap(facePosMap), m_faceVelocityMap(faceVelocityMap), m_faceNormalMap(faceNormalMap), m_faceInvMassMap(faceInvMassMap),
		m_v(v), m_f(f), m_fv1(fv1), m_fv2(fv2), m_fv3(fv3),
		m_distance(distance)
	{}

	virtual int const getTypeId() const { return TYPE_ID; }

	virtual bool initConstraint();
	virtual bool solvePositionConstraint(float factor = 1.0f) override;
	virtual bool solveVelocityConstraint() override;
};

class DirectedPositionBasedDynamics
{
public:
	static bool solve_TrianglePointSidedDistanceConstraint(
		const Eigen::Vector3f &p, float invMass,
		const Eigen::Vector3f &p0, float invMass0,
		const Eigen::Vector3f &p1, float invMass1,
		const Eigen::Vector3f &p2, float invMass2,
		const float restDist,
		const float compressionStiffness,
		const float stretchStiffness,
		const bool atRightSide,
		Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2);

	static bool solve_TrianglePointDirectedDistanceConstraint(
		const Eigen::Vector3f &p, float invMass,
		const Eigen::Vector3f &p0, float invMass0,
		const Eigen::Vector3f &p1, float invMass1,
		const Eigen::Vector3f &p2, float invMass2,
		const float restDist,
		const float compressionStiffness,
		const float stretchStiffness,
		const Eigen::Vector3f & normal,
		Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2);

};

#endif