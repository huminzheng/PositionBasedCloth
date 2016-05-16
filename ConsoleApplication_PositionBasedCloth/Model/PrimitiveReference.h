#ifndef PRIMITIVE_REFERENCE_H
#define PRIMITIVE_REFERENCE_H

#include "SurfaceMeshObject.h"
#include "Types.h"

#include <Eigen/Dense>

class Face3fRef
{
public:
	SurfaceMesh3f const & mesh;
	Faceidx const faceidx;
	/* NOTE use direct Property_map (which itself is a reference) 
	 * to avoid the strange bug that the PrimitiveRef's (in AABBTree)
	 * maps vanish
	 */
	SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap;
	SurfaceMesh3f::Property_map<Veridx, float> const invMass;

	Face3fRef(SurfaceMesh3f const & mesh, Faceidx const faceidx,
		SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap,
		SurfaceMesh3f::Property_map<Veridx, float> const invMass) :
		mesh(mesh), faceidx(faceidx), posMap(posMap), invMass(invMass) {}

	Face3fRef(Face3fRef && other) = default;
	Face3fRef & operator=(Face3fRef && other) = default;
	Face3fRef(Face3fRef const & other) = default;
	Face3fRef & operator=(Face3fRef const & other) = delete;

	PointEigen3f point(Veridx const & vid) const
	{
		return posMap[vid];
	}

};

class Vertex3fRef
{
public:
	SurfaceMesh3f const & mesh;
	Veridx const veridx;
	SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap;
	SurfaceMesh3f::Property_map<Veridx, float> const invMass;

	Vertex3fRef(SurfaceMesh3f const & mesh, Veridx const veridx,
		SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap,
		SurfaceMesh3f::Property_map<Veridx, float> const invMass) :
		mesh(mesh), veridx(veridx), posMap(posMap), invMass(invMass) {}

	Vertex3fRef(Vertex3fRef && other) = default;
	Vertex3fRef & operator=(Vertex3fRef && other) = default;
	Vertex3fRef(Vertex3fRef const & other) = default;
	Vertex3fRef & operator=(Vertex3fRef const & other) = delete;

	// TODO change to range of PointEigen3fs
	PointEigen3f point(Veridx const & vid) const
	{
		return posMap[vid];
	}

};

class Edge3fRef
{
public:
	SurfaceMesh3f const & mesh;
	Edgeidx const edgeidx;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const posMap;
	SurfaceMesh3f::Property_map<Veridx, float> const invMass;

	Edge3fRef(SurfaceMesh3f const & mesh, Edgeidx const edgeidx,
		SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> const posMap,
		SurfaceMesh3f::Property_map<Veridx, float> const invMass) :
		mesh(mesh), edgeidx(edgeidx), posMap(posMap), invMass(invMass) {}

	Edge3fRef(Edge3fRef && other) = default;
	Edge3fRef & operator=(Edge3fRef && other) = default;
	Edge3fRef(Edge3fRef const & other) = default;
	Edge3fRef & operator=(Edge3fRef const & other) = delete;

	PointEigen3f point(Veridx const & vid) const
	{
		return posMap[vid];
	}
};

class Face3fContinuesRef
{
public:
	SurfaceMesh3f const & mesh;
	Faceidx const faceidx;
	SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap0;
	SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap1;
	SurfaceMesh3f::Property_map<Veridx, float> const invMass;

	Face3fContinuesRef(SurfaceMesh3f const & mesh, Faceidx const faceidx,
		SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap0,
		SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap1,
		SurfaceMesh3f::Property_map<Veridx, float> const invMass) :
		mesh(mesh), faceidx(faceidx), posMap0(posMap0), posMap1(posMap1), invMass(invMass) {}

	Face3fContinuesRef(Face3fContinuesRef && other) = default;
	Face3fContinuesRef & operator=(Face3fContinuesRef && other) = default;
	Face3fContinuesRef(Face3fContinuesRef const & other) = default;
	Face3fContinuesRef & operator=(Face3fContinuesRef const & other) = delete;

	PointEigen3f point0(Veridx const & vid) const
	{
		return posMap0[vid];
	}

	PointEigen3f point1(Veridx const & vid) const
	{
		return posMap1[vid];
	}

};

class Vertex3fContinuesRef
{
public:
	SurfaceMesh3f const & mesh;
	Veridx const veridx;
	SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap0;
	SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap1;
	SurfaceMesh3f::Property_map<Veridx, float> const invMass;

	Vertex3fContinuesRef(SurfaceMesh3f const & mesh, Veridx const veridx,
		SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap0,
		SurfaceMesh3f::Property_map<Veridx, PointEigen3f> const posMap1,
		SurfaceMesh3f::Property_map<Veridx, float> const invMass) :
		mesh(mesh), veridx(veridx), posMap0(posMap0), posMap1(posMap1), invMass(invMass) {}

	Vertex3fContinuesRef(Vertex3fContinuesRef && other) = default;
	Vertex3fContinuesRef & operator=(Vertex3fContinuesRef && other) = default;
	Vertex3fContinuesRef(Vertex3fContinuesRef const & other) = default;
	Vertex3fContinuesRef & operator=(Vertex3fContinuesRef const & other) = delete;

	PointEigen3f point0(Veridx const & vid) const
	{
		return posMap0[vid];
	}

	PointEigen3f point1(Veridx const & vid) const
	{
		return posMap1[vid];
	}

};

#endif