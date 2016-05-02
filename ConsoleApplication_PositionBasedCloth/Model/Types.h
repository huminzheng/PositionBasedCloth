#ifndef TYPES_H
#define TYPES_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL\Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <boost/foreach.hpp>

#include <Eigen\Core>

#include <map>

// kernel type
typedef CGAL::Simple_cartesian<float> Kernelf;
// primitive types
typedef Kernelf::FT FT;
typedef Kernelf::Point_3 Point3f;
typedef Kernelf::Vector_3 Vec3f;
typedef Kernelf::Triangle_3 Triangle3f;
typedef Kernelf::Segment_3 Segment3f;
typedef Kernelf::Plane_3 Plane3f;
// surface mesh type
typedef CGAL::Surface_mesh<Point3f> SurfaceMesh3f;
// primitive index types
typedef SurfaceMesh3f::Vertex_index Veridx;
typedef SurfaceMesh3f::Face_index Faceidx;
typedef SurfaceMesh3f::Edge_index Edgeidx;
typedef SurfaceMesh3f::Halfedge_index Halfedgeidx;
// primitive iterator types
typedef SurfaceMesh3f::Vertex_iterator Veriter;
typedef SurfaceMesh3f::Edge_iterator Edgeiter;
typedef SurfaceMesh3f::Face_iterator Faceiter;

/* ----------- Eigen based primitives ---------- */

typedef Eigen::Vector3f PointEigen3f;

struct TriangleEigen3f
{
	PointEigen3f vertex[3];
};

struct PlaneEigen3f
{
	PointEigen3f normal;
	float distance;

	PlaneEigen3f(TriangleEigen3f const & triangle)
	{
		normal = (triangle.vertex[0] - triangle.vertex[2])
			.cross(triangle.vertex[1] - triangle.vertex[2]).normalized();
		distance = -normal.transpose() * triangle.vertex[2];
	}

	PointEigen3f projection(PointEigen3f const & point)
	{
		return (point - (point.transpose() * normal + distance) * normal);
	}
};


#endif
