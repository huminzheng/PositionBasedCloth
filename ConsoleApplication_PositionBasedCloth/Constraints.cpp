#include "Constraints.h"
#include "Util\BasicOperations.h"
#include "Util\Geometry.h"

#include "PositionBasedDynamics\PositionBasedDynamics.h"

//int BallJoint::TYPE_ID = 1;
//int BallOnLineJoint::TYPE_ID = 2;
//int HingeJoint::TYPE_ID = 3;
//int UniversalJoint::TYPE_ID = 4;
//int RigidBodyParticleBallJoint::TYPE_ID = 5;
int DistanceConstraint::TYPE_ID = 6;
//int DihedralConstraint::TYPE_ID = 7;
int IsometricBendingConstraint::TYPE_ID = 8;
int FEMTriangleConstraint::TYPE_ID = 9;
//int StrainTriangleConstraint::TYPE_ID = 10;
//int VolumeConstraint::TYPE_ID = 11;
//int FEMTetConstraint::TYPE_ID = 12;
//int StrainTetConstraint::TYPE_ID = 13;
//int ShapeMatchingConstraint::TYPE_ID = 14;
//int TargetAngleMotorHingeJoint::TYPE_ID = 15;
//int TargetVelocityMotorHingeJoint::TYPE_ID = 16;
//int SliderJoint::TYPE_ID = 17;
//int TargetPositionMotorSliderJoint::TYPE_ID = 18;
//int TargetVelocityMotorSliderJoint::TYPE_ID = 19;
//int EdgeEdgeDistanceConstraint::TYPE_ID = 20;
int VertexFaceDistanceConstraint::TYPE_ID = 21;
int VertexFaceSidedDistanceConstraint::TYPE_ID = 22;
int VertexFaceDirectedDistanceConstraint::TYPE_ID = 23;

////////////////////////////////////////////////////////////////////////////
//// BallJoint
////////////////////////////////////////////////////////////////////////////
//bool BallJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_BallJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos,
//		m_jointInfo);
//}
//
//bool BallJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_BallJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool BallJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_BallJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// BallOnLineJoint
////////////////////////////////////////////////////////////////////////////
//bool BallOnLineJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_BallOnLineJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos, dir,
//		m_jointInfo);
//}
//
//bool BallOnLineJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_BallOnLineJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool BallOnLineJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_BallOnLineJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// HingeJoint
////////////////////////////////////////////////////////////////////////////
//bool HingeJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_HingeJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos, axis,
//		m_jointInfo);
//}
//
//bool HingeJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_HingeJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool HingeJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_HingeJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// UniversalJoint
////////////////////////////////////////////////////////////////////////////
//bool UniversalJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_UniversalJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos,
//		axis1,
//		axis2,
//		m_jointInfo);
//}
//
//bool UniversalJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_UniversalJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool UniversalJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_UniversalJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// SliderJoint
////////////////////////////////////////////////////////////////////////////
//bool SliderJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_SliderJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos, axis,
//		m_jointInfo);
//}
//
//bool SliderJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_SliderJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool SliderJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_SliderJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// TargetPositionMotorSliderJoint
////////////////////////////////////////////////////////////////////////////
//bool TargetPositionMotorSliderJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_TargetPositionMotorSliderJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos, axis,
//		m_jointInfo);
//}
//
//bool TargetPositionMotorSliderJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_TargetPositionMotorSliderJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool TargetPositionMotorSliderJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_TargetPositionMotorSliderJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_targetPosition,
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//
//
////////////////////////////////////////////////////////////////////////////
//// TargetVelocityMotorSliderJoint
////////////////////////////////////////////////////////////////////////////
//bool TargetVelocityMotorSliderJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_TargetVelocityMotorSliderJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos, axis,
//		m_jointInfo);
//}
//
//bool TargetVelocityMotorSliderJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_TargetVelocityMotorSliderJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool TargetVelocityMotorSliderJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_TargetVelocityMotorSliderJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//
//bool TargetVelocityMotorSliderJoint::solveVelocityConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_v1, corr_v2;
//	Eigen::Vector3f corr_omega1, corr_omega2;
//	const bool res = PositionBasedRigidBodyDynamics::velocitySolve_TargetVelocityMotorSliderJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getVelocity(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getAngularVelocity(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getVelocity(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getAngularVelocity(),
//		m_targetVelocity,
//		m_jointInfo,
//		corr_v1,
//		corr_omega1,
//		corr_v2,
//		corr_omega2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getVelocity() += corr_v1;
//			rb1.getAngularVelocity() += corr_omega1;
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getVelocity() += corr_v2;
//			rb2.getAngularVelocity() += corr_omega2;
//		}
//	}
//	return res;
//}
//
////////////////////////////////////////////////////////////////////////////
//// TargetAngleMotorHingeJoint
////////////////////////////////////////////////////////////////////////////
//bool TargetAngleMotorHingeJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_TargetAngleMotorHingeJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos, axis,
//		m_jointInfo);
//}
//
//bool TargetAngleMotorHingeJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_TargetAngleMotorHingeJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool TargetAngleMotorHingeJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_TargetAngleMotorHingeJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_targetAngle,
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
////////////////////////////////////////////////////////////////////////////
//// TargetVelocityMotorHingeJoint
////////////////////////////////////////////////////////////////////////////
//bool TargetVelocityMotorHingeJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
//{
//	m_bodies[0] = rbIndex1;
//	m_bodies[1] = rbIndex2;
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::init_TargetVelocityMotorHingeJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		pos, axis,
//		m_jointInfo);
//}
//
//bool TargetVelocityMotorHingeJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//	return PositionBasedRigidBodyDynamics::update_TargetVelocityMotorHingeJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		rb2.getPosition(),
//		rb2.getRotation(),
//		m_jointInfo);
//}
//
//bool TargetVelocityMotorHingeJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1, corr_q2;
//	const bool res = PositionBasedRigidBodyDynamics::solve_TargetVelocityMotorHingeJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getRotation(),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2,
//		corr_q2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getPosition() += corr_x2;
//			rb2.getRotation().coeffs() += corr_q2.coeffs();
//			rb2.getRotation().normalize();
//			rb2.rotationUpdated();
//		}
//	}
//	return res;
//}
//
//bool TargetVelocityMotorHingeJoint::solveVelocityConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	RigidBody &rb2 = *rb[m_bodies[1]];
//
//	Eigen::Vector3f corr_v1, corr_v2;
//	Eigen::Vector3f corr_omega1, corr_omega2;
//	const bool res = PositionBasedRigidBodyDynamics::velocitySolve_TargetVelocityMotorHingeJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getVelocity(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getAngularVelocity(),
//		rb2.getInvMass(),
//		rb2.getPosition(),
//		rb2.getVelocity(),
//		rb2.getInertiaTensorInverseW(),
//		rb2.getAngularVelocity(),
//		m_targetAngularVelocity,
//		m_jointInfo,
//		corr_v1,
//		corr_omega1,
//		corr_v2,
//		corr_omega2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getVelocity() += corr_v1;
//			rb1.getAngularVelocity() += corr_omega1;
//		}
//		if (rb2.getMass() != 0.0f)
//		{
//			rb2.getVelocity() += corr_v2;
//			rb2.getAngularVelocity() += corr_omega2;
//		}
//	}
//	return res;
//}
//
////////////////////////////////////////////////////////////////////////////
//// RigidBodyParticleBallJoint
////////////////////////////////////////////////////////////////////////////
//bool RigidBodyParticleBallJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex, const unsigned int particleIndex)
//{
//	m_bodies[0] = rbIndex;
//	m_bodies[1] = particleIndex;
//	SimulationModel::RigidBodyVector &rbs = model.getRigidBodies();
//	ParticleData &pd = model.getParticles();
//	RigidBody &rb = *rbs[m_bodies[0]];
//	return PositionBasedRigidBodyDynamics::init_RigidBodyParticleBallJoint(
//		rb.getPosition(),
//		rb.getRotation(),
//		pd.getPosition(particleIndex),
//		m_jointInfo);
//}
//
//bool RigidBodyParticleBallJoint::updateConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	ParticleData &pd = model.getParticles();
//	RigidBody &rb1 = *rb[m_bodies[0]];
//	return PositionBasedRigidBodyDynamics::update_RigidBodyParticleBallJoint(
//		rb1.getPosition(),
//		rb1.getRotation(),
//		pd.getPosition(m_bodies[1]),
//		m_jointInfo);
//}
//
//bool RigidBodyParticleBallJoint::solvePositionConstraint(SimulationModel &model)
//{
//	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
//	ParticleData &pd = model.getParticles();
//
//	RigidBody &rb1 = *rb[m_bodies[0]];
//
//	Eigen::Vector3f corr_x1, corr_x2;
//	Eigen::Quaternionf corr_q1;
//	const bool res = PositionBasedRigidBodyDynamics::solve_RigidBodyParticleBallJoint(
//		rb1.getInvMass(),
//		rb1.getPosition(),
//		rb1.getInertiaTensorInverseW(),
//		rb1.getRotation(),
//		pd.getInvMass(m_bodies[1]),
//		pd.getPosition(m_bodies[1]),
//		m_jointInfo,
//		corr_x1,
//		corr_q1,
//		corr_x2);
//
//	if (res)
//	{
//		if (rb1.getMass() != 0.0f)
//		{
//			rb1.getPosition() += corr_x1;
//			rb1.getRotation().coeffs() += corr_q1.coeffs();
//			rb1.getRotation().normalize();
//			rb1.rotationUpdated();
//		}
//		if (pd.getMass(m_bodies[1]) != 0.0f)
//		{
//			pd.getPosition(m_bodies[1]) += corr_x2;
//		}
//	}
//	return res;
//}

//////////////////////////////////////////////////////////////////////////
// DistanceConstraint
//////////////////////////////////////////////////////////////////////////
bool DistanceConstraint::initConstraint()
{
	m_restLength = (m_geodesicMap[m_v1] - m_geodesicMap[m_v2]).norm();

	return true;
}

bool DistanceConstraint::solvePositionConstraint()
{
	Eigen::Vector3f & x1 = m_posMap[m_v1];
	Eigen::Vector3f & x2 = m_posMap[m_v2];
	//std::cout << x1 << std::endl;
	//std::cout << x2 << std::endl;

	float const invMass1 = m_invMassMap[m_v1];
	float const invMass2 = m_invMassMap[m_v2];
	//std::cout << invMass1 << std::endl;
	//std::cout << invMass2 << std::endl;

	Eigen::Vector3f corr1, corr2;
	const bool res = PBD::PositionBasedDynamics::solve_DistanceConstraint(
		x1, invMass1, x2, invMass2,
		m_restLength, m_stiff, m_stiff, corr1, corr2);

	//if (corr1.squaredNorm() > 1e-4 || corr2.squaredNorm() > 1e-4)
	//{
	//	std::cout << "correction too large between " << m_v1 << " " << m_v2 << std::endl;
	//}

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
	}
	return res;
}


////////////////////////////////////////////////////////////////////////////
//// DihedralConstraint
////////////////////////////////////////////////////////////////////////////
//
//bool DihedralConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//	const unsigned int particle3, const unsigned int particle4)
//{
//	m_bodies[0] = particle1;
//	m_bodies[1] = particle2;
//	m_bodies[2] = particle3;
//	m_bodies[3] = particle4;
//	ParticleData &pd = model.getParticles();
//
//	const Eigen::Vector3f &p0 = pd.getPosition0(particle1);
//	const Eigen::Vector3f &p1 = pd.getPosition0(particle2);
//	const Eigen::Vector3f &p2 = pd.getPosition0(particle3);
//	const Eigen::Vector3f &p3 = pd.getPosition0(particle4);
//
//	Eigen::Vector3f e = p3 - p2;
//	float  elen = e.norm();
//	if (elen < 1e-6f)
//		return false;
//
//	float invElen = 1.0f / elen;
//
//	Eigen::Vector3f n1 = (p2 - p0).cross(p3 - p0); n1 /= n1.squaredNorm();
//	Eigen::Vector3f n2 = (p3 - p1).cross(p2 - p1); n2 /= n2.squaredNorm();
//
//	n1.normalize();
//	n2.normalize();
//	float dot = n1.dot(n2);
//
//	if (dot < -1.0f) dot = -1.0f;
//	if (dot > 1.0f) dot = 1.0f;
//
//	m_restAngle = acosf(dot);
//
//	return true;
//}
//
//bool DihedralConstraint::solvePositionConstraint(SimulationModel &model)
//{
//	ParticleData &pd = model.getParticles();
//
//	const unsigned i1 = m_bodies[0];
//	const unsigned i2 = m_bodies[1];
//	const unsigned i3 = m_bodies[2];
//	const unsigned i4 = m_bodies[3];
//
//	Eigen::Vector3f &x1 = pd.getPosition(i1);
//	Eigen::Vector3f &x2 = pd.getPosition(i2);
//	Eigen::Vector3f &x3 = pd.getPosition(i3);
//	Eigen::Vector3f &x4 = pd.getPosition(i4);
//
//	const float invMass1 = pd.getInvMass(i1);
//	const float invMass2 = pd.getInvMass(i2);
//	const float invMass3 = pd.getInvMass(i3);
//	const float invMass4 = pd.getInvMass(i4);
//
//	Eigen::Vector3f corr1, corr2, corr3, corr4;
//	const bool res = PositionBasedDynamics::solve_DihedralConstraint(
//		x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4,
//		m_restAngle,
//		model.getClothBendingStiffness(),
//		corr1, corr2, corr3, corr4);
//
//	if (res)
//	{
//		if (invMass1 != 0.0f)
//			x1 += corr1;
//		if (invMass2 != 0.0f)
//			x2 += corr2;
//		if (invMass3 != 0.0f)
//			x3 += corr3;
//		if (invMass4 != 0.0f)
//			x4 += corr4;
//	}
//	return res;
//}


//////////////////////////////////////////////////////////////////////////
// IsometricBendingConstraint
//////////////////////////////////////////////////////////////////////////
bool IsometricBendingConstraint::initConstraint()
{
	Eigen::Vector3f & x1 = m_posMap[m_v1];
	Eigen::Vector3f & x2 = m_posMap[m_v2];
	Eigen::Vector3f & x3 = m_posMap[m_v3];
	Eigen::Vector3f & x4 = m_posMap[m_v4];

	return PBD::PositionBasedDynamics::init_IsometricBendingConstraint(x1, x2, x3, x4, m_Q);
}

bool IsometricBendingConstraint::solvePositionConstraint()
{
	Eigen::Vector3f & x1 = m_posMap[m_v1];
	Eigen::Vector3f & x2 = m_posMap[m_v2];
	Eigen::Vector3f & x3 = m_posMap[m_v3];
	Eigen::Vector3f & x4 = m_posMap[m_v4];

	//if ((x1 - x2).squaredNorm() <= DISTANCE_OVERLAP_THRESHOLD ||
	//	(x2 - x3).squaredNorm() <= DISTANCE_OVERLAP_THRESHOLD ||
	//	(x3 - x1).squaredNorm() <= DISTANCE_OVERLAP_THRESHOLD ||
	//	(x3 - x4).squaredNorm() <= DISTANCE_OVERLAP_THRESHOLD ||
	//	(x4 - x2).squaredNorm() <= DISTANCE_OVERLAP_THRESHOLD)
	//{
	//	return false;
	//}

	float const invMass1 = m_invMassMap[m_v1];
	float const invMass2 = m_invMassMap[m_v2];
	float const invMass3 = m_invMassMap[m_v3];
	float const invMass4 = m_invMassMap[m_v4];

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	const bool res = PBD::PositionBasedDynamics::solve_IsometricBendingConstraint(
		x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4,
		m_Q, m_stiff,
		corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
		if (invMass4 != 0.0f)
			x4 += corr4;
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// FEMTriangleConstraint
//////////////////////////////////////////////////////////////////////////
bool FEMTriangleConstraint::initConstraint()
{
	Eigen::Vector3f &x1 = m_posMap[m_v1];
	Eigen::Vector3f &x2 = m_posMap[m_v2];
	Eigen::Vector3f &x3 = m_posMap[m_v3];

	return PBD::PositionBasedDynamics::init_FEMTriangleConstraint(x1, x2, x3, m_area, m_invRestMat);
}

bool FEMTriangleConstraint::solvePositionConstraint()
{
	Eigen::Vector3f &x1 = m_posMap[m_v1];
	Eigen::Vector3f &x2 = m_posMap[m_v2];
	Eigen::Vector3f &x3 = m_posMap[m_v3];

	float const invMass1 = m_invMassMap[m_v1];
	float const invMass2 = m_invMassMap[m_v2];
	float const invMass3 = m_invMassMap[m_v3];

	Eigen::Vector3f corr1, corr2, corr3;
	const bool res = PBD::PositionBasedDynamics::solve_FEMTriangleConstraint(
		x1, invMass1,
		x2, invMass2,
		x3, invMass3,
		m_area,
		m_invRestMat,
		m_stiff_xx, m_stiff_yy, m_stiff_xy, 
		m_possion_ration_xy, m_possion_ration_yx,
		corr1, corr2, corr3);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
	}
	return res;
}


////////////////////////////////////////////////////////////////////////////
//// StrainTriangleConstraint
////////////////////////////////////////////////////////////////////////////
//bool StrainTriangleConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//	const unsigned int particle3)
//{
//	m_bodies[0] = particle1;
//	m_bodies[1] = particle2;
//	m_bodies[2] = particle3;
//
//	ParticleData &pd = model.getParticles();
//
//	Eigen::Vector3f &x1 = pd.getPosition0(particle1);
//	Eigen::Vector3f &x2 = pd.getPosition0(particle2);
//	Eigen::Vector3f &x3 = pd.getPosition0(particle3);
//
//	// Bring triangles to xy plane
//	const Eigen::Vector3f y1(x1[0], x1[2], 0.0);
//	const Eigen::Vector3f y2(x2[0], x2[2], 0.0);
//	const Eigen::Vector3f y3(x3[0], x3[2], 0.0);
//
//	return PositionBasedDynamics::init_StrainTriangleConstraint(y1, y2, y3, m_invRestMat);
//}
//
//bool StrainTriangleConstraint::solvePositionConstraint(SimulationModel &model)
//{
//	ParticleData &pd = model.getParticles();
//
//	const unsigned i1 = m_bodies[0];
//	const unsigned i2 = m_bodies[1];
//	const unsigned i3 = m_bodies[2];
//
//	Eigen::Vector3f &x1 = pd.getPosition(i1);
//	Eigen::Vector3f &x2 = pd.getPosition(i2);
//	Eigen::Vector3f &x3 = pd.getPosition(i3);
//
//	const float invMass1 = pd.getInvMass(i1);
//	const float invMass2 = pd.getInvMass(i2);
//	const float invMass3 = pd.getInvMass(i3);
//
//	Eigen::Vector3f corr1, corr2, corr3;
//	const bool res = PositionBasedDynamics::solve_StrainTriangleConstraint(
//		x1, invMass1,
//		x2, invMass2,
//		x3, invMass3,
//		m_invRestMat,
//		model.getClothXXStiffness(),
//		model.getClothYYStiffness(),
//		model.getClothXYStiffness(),
//		model.getClothNormalizeStretch(),
//		model.getClothNormalizeShear(),
//		corr1, corr2, corr3);
//
//	if (res)
//	{
//		if (invMass1 != 0.0f)
//			x1 += corr1;
//		if (invMass2 != 0.0f)
//			x2 += corr2;
//		if (invMass3 != 0.0f)
//			x3 += corr3;
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// VolumeConstraint
////////////////////////////////////////////////////////////////////////////
//
//bool VolumeConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//	const unsigned int particle3, const unsigned int particle4)
//{
//	m_bodies[0] = particle1;
//	m_bodies[1] = particle2;
//	m_bodies[2] = particle3;
//	m_bodies[3] = particle4;
//	ParticleData &pd = model.getParticles();
//
//	const Eigen::Vector3f &p0 = pd.getPosition0(particle1);
//	const Eigen::Vector3f &p1 = pd.getPosition0(particle2);
//	const Eigen::Vector3f &p2 = pd.getPosition0(particle3);
//	const Eigen::Vector3f &p3 = pd.getPosition0(particle4);
//
//	m_restVolume = fabs((1.0f / 6.0f) * (p3 - p0).dot((p2 - p0).cross(p1 - p0)));
//
//	return true;
//}
//
//bool VolumeConstraint::solvePositionConstraint(SimulationModel &model)
//{
//	ParticleData &pd = model.getParticles();
//
//	const unsigned i1 = m_bodies[0];
//	const unsigned i2 = m_bodies[1];
//	const unsigned i3 = m_bodies[2];
//	const unsigned i4 = m_bodies[3];
//
//	Eigen::Vector3f &x1 = pd.getPosition(i1);
//	Eigen::Vector3f &x2 = pd.getPosition(i2);
//	Eigen::Vector3f &x3 = pd.getPosition(i3);
//	Eigen::Vector3f &x4 = pd.getPosition(i4);
//
//	const float invMass1 = pd.getInvMass(i1);
//	const float invMass2 = pd.getInvMass(i2);
//	const float invMass3 = pd.getInvMass(i3);
//	const float invMass4 = pd.getInvMass(i4);
//
//	Eigen::Vector3f corr1, corr2, corr3, corr4;
//	const bool res = PositionBasedDynamics::solve_VolumeConstraint(x1, invMass1,
//		x2, invMass2,
//		x3, invMass3,
//		x4, invMass4,
//		m_restVolume,
//		model.getSolidStiffness(),
//		model.getSolidStiffness(),
//		corr1, corr2, corr3, corr4);
//
//	if (res)
//	{
//		if (invMass1 != 0.0f)
//			x1 += corr1;
//		if (invMass2 != 0.0f)
//			x2 += corr2;
//		if (invMass3 != 0.0f)
//			x3 += corr3;
//		if (invMass4 != 0.0f)
//			x4 += corr4;
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// FEMTetConstraint
////////////////////////////////////////////////////////////////////////////
//bool FEMTetConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//	const unsigned int particle3, const unsigned int particle4)
//{
//	m_bodies[0] = particle1;
//	m_bodies[1] = particle2;
//	m_bodies[2] = particle3;
//	m_bodies[3] = particle4;
//
//	ParticleData &pd = model.getParticles();
//
//	Eigen::Vector3f &x1 = pd.getPosition0(particle1);
//	Eigen::Vector3f &x2 = pd.getPosition0(particle2);
//	Eigen::Vector3f &x3 = pd.getPosition0(particle3);
//	Eigen::Vector3f &x4 = pd.getPosition0(particle4);
//
//	return PositionBasedDynamics::init_FEMTetraConstraint(x1, x2, x3, x4, m_volume, m_invRestMat);
//}
//
//bool FEMTetConstraint::solvePositionConstraint(SimulationModel &model)
//{
//	ParticleData &pd = model.getParticles();
//
//	const unsigned i1 = m_bodies[0];
//	const unsigned i2 = m_bodies[1];
//	const unsigned i3 = m_bodies[2];
//	const unsigned i4 = m_bodies[3];
//
//	Eigen::Vector3f &x1 = pd.getPosition(i1);
//	Eigen::Vector3f &x2 = pd.getPosition(i2);
//	Eigen::Vector3f &x3 = pd.getPosition(i3);
//	Eigen::Vector3f &x4 = pd.getPosition(i4);
//
//	const float invMass1 = pd.getInvMass(i1);
//	const float invMass2 = pd.getInvMass(i2);
//	const float invMass3 = pd.getInvMass(i3);
//	const float invMass4 = pd.getInvMass(i4);
//
//	float currentVolume = -(1.0f / 6.0f) * (x4 - x1).dot((x3 - x1).cross(x2 - x1));
//	bool handleInversion = false;
//	if (currentVolume / m_volume < 0.2)		// Only 20% of initial volume left
//		handleInversion = true;
//
//
//	Eigen::Vector3f corr1, corr2, corr3, corr4;
//	const bool res = PositionBasedDynamics::solve_FEMTetraConstraint(
//		x1, invMass1,
//		x2, invMass2,
//		x3, invMass3,
//		x4, invMass4,
//		m_volume,
//		m_invRestMat,
//		model.getSolidStiffness(),
//		model.getSolidPoissonRatio(), handleInversion,
//		corr1, corr2, corr3, corr4);
//
//	if (res)
//	{
//		if (invMass1 != 0.0f)
//			x1 += corr1;
//		if (invMass2 != 0.0f)
//			x2 += corr2;
//		if (invMass3 != 0.0f)
//			x3 += corr3;
//		if (invMass4 != 0.0f)
//			x4 += corr4;
//	}
//	return res;
//}
//
//
////////////////////////////////////////////////////////////////////////////
//// StrainTetConstraint
////////////////////////////////////////////////////////////////////////////
//bool StrainTetConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
//	const unsigned int particle3, const unsigned int particle4)
//{
//	m_bodies[0] = particle1;
//	m_bodies[1] = particle2;
//	m_bodies[2] = particle3;
//	m_bodies[3] = particle4;
//
//	ParticleData &pd = model.getParticles();
//
//	Eigen::Vector3f &x1 = pd.getPosition0(particle1);
//	Eigen::Vector3f &x2 = pd.getPosition0(particle2);
//	Eigen::Vector3f &x3 = pd.getPosition0(particle3);
//	Eigen::Vector3f &x4 = pd.getPosition0(particle4);
//
//	return PositionBasedDynamics::init_StrainTetraConstraint(x1, x2, x3, x4, m_invRestMat);
//}
//
//bool StrainTetConstraint::solvePositionConstraint(SimulationModel &model)
//{
//	ParticleData &pd = model.getParticles();
//
//	const unsigned i1 = m_bodies[0];
//	const unsigned i2 = m_bodies[1];
//	const unsigned i3 = m_bodies[2];
//	const unsigned i4 = m_bodies[3];
//
//	Eigen::Vector3f &x1 = pd.getPosition(i1);
//	Eigen::Vector3f &x2 = pd.getPosition(i2);
//	Eigen::Vector3f &x3 = pd.getPosition(i3);
//	Eigen::Vector3f &x4 = pd.getPosition(i4);
//
//	const float invMass1 = pd.getInvMass(i1);
//	const float invMass2 = pd.getInvMass(i2);
//	const float invMass3 = pd.getInvMass(i3);
//	const float invMass4 = pd.getInvMass(i4);
//
//	Eigen::Vector3f stiffness(model.getSolidStiffness(), model.getSolidStiffness(), model.getSolidStiffness());
//
//	Eigen::Vector3f corr1, corr2, corr3, corr4;
//	const bool res = PositionBasedDynamics::solve_StrainTetraConstraint(
//		x1, invMass1,
//		x2, invMass2,
//		x3, invMass3,
//		x4, invMass4,
//		m_invRestMat,
//		stiffness,
//		stiffness,
//		model.getSolidNormalizeStretch(),
//		model.getSolidNormalizeShear(),
//		corr1, corr2, corr3, corr4);
//
//	if (res)
//	{
//		if (invMass1 != 0.0f)
//			x1 += corr1;
//		if (invMass2 != 0.0f)
//			x2 += corr2;
//		if (invMass3 != 0.0f)
//			x3 += corr3;
//		if (invMass4 != 0.0f)
//			x4 += corr4;
//	}
//	return res;
//}
//
////////////////////////////////////////////////////////////////////////////
//// ShapeMatchingConstraint
////////////////////////////////////////////////////////////////////////////
//bool ShapeMatchingConstraint::initConstraint(SimulationModel &model,
//	const unsigned int particleIndices[], const unsigned int numClusters[])
//{
//	ParticleData &pd = model.getParticles();
//	for (unsigned int i = 0; i < m_numberOfBodies; i++)
//	{
//		m_bodies[i] = particleIndices[i];
//		m_x0[i] = pd.getPosition0(m_bodies[i]);
//		m_w[i] = pd.getInvMass(m_bodies[i]);
//		m_numClusters[i] = numClusters[i];
//	}
//
//	const bool res = PositionBasedDynamics::init_ShapeMatchingConstraint(m_x0, m_w, m_numberOfBodies, m_restCm, m_invRestMat);
//	return res;
//}
//
//bool ShapeMatchingConstraint::solvePositionConstraint(SimulationModel &model)
//{
//	ParticleData &pd = model.getParticles();
//	for (unsigned int i = 0; i < m_numberOfBodies; i++)
//	{
//		m_x[i] = pd.getPosition(m_bodies[i]);
//	}
//
//	const bool res = PositionBasedDynamics::solve_ShapeMatchingConstraint(
//		m_x0, m_x, m_w, m_numberOfBodies,
//		m_restCm, m_invRestMat,
//		model.getSolidStiffness(), false,
//		m_corr);
//
//	if (res)
//	{
//		for (unsigned int i = 0; i < m_numberOfBodies; i++)
//		{
//			// Important: Divide position correction by the number of clusters 
//			// which contain the vertex. 
//			if (m_w[i] != 0.0f)
//				pd.getPosition(m_bodies[i]) += (1.0f / m_numClusters[i]) * m_corr[i];
//		}
//	}
//	return res;
//}

//////////////////////////////////////////////////////////////////////////
// VertexFaceDistanceConstraint
//////////////////////////////////////////////////////////////////////////
bool VertexFaceDistanceConstraint::initConstraint()
{
	return true;
}

bool VertexFaceDistanceConstraint::solvePositionConstraint()
{
	Eigen::Vector3f & p = m_vertexPosMap[m_v];
	Eigen::Vector3f & p0 = m_facePosMap[m_fv1];
	Eigen::Vector3f & p1 = m_facePosMap[m_fv2];
	Eigen::Vector3f & p2 = m_facePosMap[m_fv3];

	float const invMass1 = m_vertexMove ? m_vertexInvMassMap[m_v] : 0.0f;
	float const invMass2 = m_faceMove ? m_faceInvMassMap[m_fv1] : 0.0f;
	float const invMass3 = m_faceMove ? m_faceInvMassMap[m_fv2] : 0.0f;
	float const invMass4 = m_faceMove ? m_faceInvMassMap[m_fv3] : 0.0f;

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	bool res = PBD::PositionBasedDynamics::solve_TrianglePointDistanceConstraint(
		p, invMass1, p0, invMass2, p1, invMass3, p2, invMass4,
		m_distance, m_stiff, 0.0f, corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			p += corr1;
		if (invMass2 != 0.0f)
			p0 += corr2;
		if (invMass3 != 0.0f)
			p1 += corr3;
		if (invMass4 != 0.0f)
			p2 += corr4;
		//std::cout << m_stiff << std::endl;
		//std::cout << corr1 << std::endl;
		//std::cout << corr2 << std::endl;
		//std::cout << corr3 << std::endl;
		//std::cout << corr4 << std::endl;
	}

	return true;
}

bool VertexFaceDistanceConstraint::solveVelocityConstraint()
{
	m_vertexVelocityMap[m_v] = Eigen::Vector3f::Zero();
	m_faceVelocityMap[m_fv1] = Eigen::Vector3f::Zero();
	m_faceVelocityMap[m_fv2] = Eigen::Vector3f::Zero();
	m_faceVelocityMap[m_fv3] = Eigen::Vector3f::Zero();
	return true;
}

//////////////////////////////////////////////////////////////////////////
// VertexFaceSidedDistanceConstraint
//////////////////////////////////////////////////////////////////////////
bool VertexFaceSidedDistanceConstraint::initConstraint()
{
	return true;
}

bool VertexFaceSidedDistanceConstraint::solvePositionConstraint()
{
	Eigen::Vector3f & p = m_vertexPosMap[m_v];
	Eigen::Vector3f & p0 = m_facePosMap[m_fv1];
	Eigen::Vector3f & p1 = m_facePosMap[m_fv2];
	Eigen::Vector3f & p2 = m_facePosMap[m_fv3];

	float const invMass1 = m_vertexInvMassMap[m_v];
	float const invMass2 = m_faceInvMassMap[m_fv1];
	float const invMass3 = m_faceInvMassMap[m_fv2];
	float const invMass4 = m_faceInvMassMap[m_fv3];

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	bool res = DirectedPositionBasedDynamics::solve_TrianglePointSidedDistanceConstraint(
		p, invMass1, p0, invMass2, p1, invMass3, p2, invMass4,
		m_distance, m_stiff, 0.0f, m_atRightSide, corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			p += corr1;
		if (invMass2 != 0.0f)
			p0 += corr2;
		if (invMass3 != 0.0f)
			p1 += corr3;
		if (invMass4 != 0.0f)
			p2 += corr4;
		//std::cout << m_stiff << std::endl;
		//std::cout << corr1 << std::endl;
		//std::cout << corr2 << std::endl;
		//std::cout << corr3 << std::endl;
		//std::cout << corr4 << std::endl;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
// VertexFaceDirectedDistanceConstraint
//////////////////////////////////////////////////////////////////////////
bool VertexFaceDirectedDistanceConstraint::initConstraint()
{
	return true;
}

bool VertexFaceDirectedDistanceConstraint::solvePositionConstraint()
{
	Eigen::Vector3f & p = m_vertexPosMap[m_v];
	
	Eigen::Vector3f & p0 = m_facePosMap[m_fv1];
	Eigen::Vector3f & p1 = m_facePosMap[m_fv2];
	Eigen::Vector3f & p2 = m_facePosMap[m_fv3];

	Eigen::Vector3f & n = m_faceNormalMap[m_f];

	float const invMass1 = m_vertexInvMassMap[m_v];
	float const invMass2 = m_faceInvMassMap[m_fv1];
	float const invMass3 = m_faceInvMassMap[m_fv2];
	float const invMass4 = m_faceInvMassMap[m_fv3];

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	bool res = DirectedPositionBasedDynamics::solve_TrianglePointDirectedDistanceConstraint(
		p, invMass1, p0, invMass2, p1, invMass3, p2, invMass4,
		m_distance, m_stiff, 0.0f, n, corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			p += corr1;
		if (invMass2 != 0.0f)
			p0 += corr2;
		if (invMass3 != 0.0f)
			p1 += corr3;
		if (invMass4 != 0.0f)
			p2 += corr4;
	}

	return true;
}

bool VertexFaceDirectedDistanceConstraint::solveVelocityConstraint()
{
	float damp = 0.9f;
	Eigen::Vector3f normal = m_faceNormalMap[m_f];
	Eigen::Matrix3f dof = (Eigen::Matrix3f::Identity() - normal * normal.transpose()) * damp;
	m_vertexVelocityMap[m_v] = dof * m_vertexVelocityMap[m_v];
	m_faceVelocityMap[m_fv1] = dof * m_faceVelocityMap[m_fv1];
	m_faceVelocityMap[m_fv2] = dof * m_faceVelocityMap[m_fv2];
	m_faceVelocityMap[m_fv3] = dof * m_faceVelocityMap[m_fv3];
	return true;
}

/* solve triangle point distance constraint,
 * with forward/backward face of triangle makes difference,
 * constraint function is 
 * C(q, p1, p2, p3) = (q - p1) * n_directed_hat - h 
 */
bool DirectedPositionBasedDynamics::solve_TrianglePointSidedDistanceConstraint(
	const Eigen::Vector3f & p, float invMass, 
	const Eigen::Vector3f & p0, float invMass0, 
	const Eigen::Vector3f & p1, float invMass1, 
	const Eigen::Vector3f & p2, float invMass2, 
	const float restDist, 
	const float compressionStiffness, 
	const float stretchStiffness, 
	const bool atRightSide, 
	Eigen::Vector3f & corr, Eigen::Vector3f & corr0, Eigen::Vector3f & corr1, Eigen::Vector3f & corr2)
{
	// find barycentric coordinates of closest point on triangle

	float b0 = 1.0f / 3.0f;		// for singular case
	float b1 = b0;
	float b2 = b0;

	Eigen::Vector3f d1 = p1 - p0;
	Eigen::Vector3f d2 = p2 - p0;
	Eigen::Vector3f pp0 = p - p0;
	float a = d1.dot(d1);
	float b = d2.dot(d1);
	float c = pp0.dot(d1);
	float d = b;
	float e = d2.dot(d2);
	float f = pp0.dot(d2);
	float det = a*e - b*d;

	if (det != 0.0f) {
		float s = (c*e - b*f) / det;
		float t = (a*f - c*d) / det;
		b0 = 1.0f - s - t;		// inside triangle
		b1 = s;
		b2 = t;
		if (b0 < 0.0f) {		// on edge 1-2
			Eigen::Vector3f d = p2 - p1;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p1) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 1
			if (t > 1.0f) t = 1.0f;	// on point 2
			b0 = 0.0f;
			b1 = (1.0f - t);
			b2 = t;
		}
		else if (b1 < 0.0f) {	// on edge 2-0
			Eigen::Vector3f d = p0 - p2;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p2) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 2
			if (t > 1.0f) t = 1.0f; // on point 0
			b1 = 0.0f;
			b2 = (1.0f - t);
			b0 = t;
		}
		else if (b2 < 0.0f) {	// on edge 0-1
			Eigen::Vector3f d = p1 - p0;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p0) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 0
			if (t > 1.0f) t = 1.0f;	// on point 1
			b2 = 0.0f;
			b0 = (1.0f - t);
			b1 = t;
		}
	}
	Eigen::Vector3f q = p0 * b0 + p1 * b1 + p2 * b2;
	Eigen::Vector3f n = (p - q) * ((atRightSide) ? 1.0f : -1.0f);
	float dist = n.norm() * ((atRightSide) ? 1.0f : -1.0f);
	n.normalize();
	float C = dist - restDist;
	Eigen::Vector3f grad = n;
	Eigen::Vector3f grad0 = -n * b0;
	Eigen::Vector3f grad1 = -n * b1;
	Eigen::Vector3f grad2 = -n * b2;

	float s = invMass + invMass0 * b0*b0 + invMass1 * b1*b1 + invMass2 * b2*b2;
	if (s == 0.0f)
		return false;

	s = C / s;
	if (C < 0.0f)
		s *= compressionStiffness;
	else
		s *= stretchStiffness;

	if (s == 0.0f)
		return false;

	corr = -s * invMass * grad;
	corr0 = -s * invMass0 * grad0;
	corr1 = -s * invMass1 * grad1;
	corr2 = -s * invMass2 * grad2;
	return true;
}

/* solve triangle point distance constraint,
* with forward/backward face of triangle makes difference,
* constraint function is
* C(q, p1, p2, p3) = (q - p1) * n_directed_hat - h
*/
bool DirectedPositionBasedDynamics::solve_TrianglePointDirectedDistanceConstraint(
	const Eigen::Vector3f & p, float invMass,
	const Eigen::Vector3f & p0, float invMass0,
	const Eigen::Vector3f & p1, float invMass1,
	const Eigen::Vector3f & p2, float invMass2,
	const float restDist,
	const float compressionStiffness,
	const float stretchStiffness,
	const Eigen::Vector3f & normal,
	Eigen::Vector3f & corr, Eigen::Vector3f & corr0, Eigen::Vector3f & corr1, Eigen::Vector3f & corr2)
{
	// find barycentric coordinates of closest point on triangle

	float b0 = 1.0f / 3.0f;		// for singular case
	float b1 = b0;
	float b2 = b0;

	Eigen::Vector3f d1 = p1 - p0;
	Eigen::Vector3f d2 = p2 - p0;
	Eigen::Vector3f pp0 = p - p0;
	float a = d1.dot(d1);
	float b = d2.dot(d1);
	float c = pp0.dot(d1);
	float d = b;
	float e = d2.dot(d2);
	float f = pp0.dot(d2);
	float det = a*e - b*d;

	if (det != 0.0f) {
		float s = (c*e - b*f) / det;
		float t = (a*f - c*d) / det;
		b0 = 1.0f - s - t;		// inside triangle
		b1 = s;
		b2 = t;
		if (b0 < 0.0f) {		// on edge 1-2
			Eigen::Vector3f d = p2 - p1;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p1) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 1
			if (t > 1.0f) t = 1.0f;	// on point 2
			b0 = 0.0f;
			b1 = (1.0f - t);
			b2 = t;
		}
		else if (b1 < 0.0f) {	// on edge 2-0
			Eigen::Vector3f d = p0 - p2;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p2) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 2
			if (t > 1.0f) t = 1.0f; // on point 0
			b1 = 0.0f;
			b2 = (1.0f - t);
			b0 = t;
		}
		else if (b2 < 0.0f) {	// on edge 0-1
			Eigen::Vector3f d = p1 - p0;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p0) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 0
			if (t > 1.0f) t = 1.0f;	// on point 1
			b2 = 0.0f;
			b0 = (1.0f - t);
			b1 = t;
		}
	}
	Eigen::Vector3f q = p0 * b0 + p1 * b1 + p2 * b2;
	bool atRightSide = ((p - q).transpose() * normal > 0);
	Eigen::Vector3f n = (p - q) * ((atRightSide) ? 1.0f : -1.0f);
	float dist = n.norm() * ((atRightSide) ? 1.0f : -1.0f);
	n.normalize();
	float C = dist - restDist;
	Eigen::Vector3f grad = n;
	Eigen::Vector3f grad0 = -n * b0;
	Eigen::Vector3f grad1 = -n * b1;
	Eigen::Vector3f grad2 = -n * b2;

	float s = invMass + invMass0 * b0*b0 + invMass1 * b1*b1 + invMass2 * b2*b2;
	if (s == 0.0f)
		return false;

	s = C / s;
	if (C < 0.0f)
		s *= compressionStiffness;
	else
		s *= stretchStiffness;

	if (s == 0.0f)
		return false;

	corr = -s * invMass * grad;
	corr0 = -s * invMass0 * grad0;
	corr1 = -s * invMass1 * grad1;
	corr2 = -s * invMass2 * grad2;
	return true;
}
