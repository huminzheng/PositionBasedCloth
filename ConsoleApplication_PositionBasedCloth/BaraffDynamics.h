#ifndef BARAFF_DYNAMICS_H
#define BARAFF_DYNAMICS_H

#include "Model\SurfaceMeshObject.h"
#include "BaraffPhysics.h"
#include "BaraffMPCGSolver.h"


class BaraffDynamics
{

public:
	BaraffDynamics(SurfaceMeshObject * model) : 
		model(model), physics(new BaraffPhysics(model))
	{
		initial();
	}

	void stepforward(float time_step);

	void writeBack();

	SurfaceMeshObject* getClothPiece()
	{
		return model;
	}

	void exportShearConditionData(GLfloat* & dataBuffer, GLuint & dataSize);
	void exportBendConditionData(GLfloat* & dataBuffer, GLuint & dataSize);

private:
	SurfaceMeshObject* model;
	BaraffPhysics* physics;
	Eigen::VectorXf last_root;
	//BaraffMPCGSolver solver;

	void initial();

};

#endif
