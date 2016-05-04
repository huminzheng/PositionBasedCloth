#ifndef DISTANCE_H
#define DISTANCE_H

#include "BasicTypes.h"
#include "..\Model\Types.h"
#include "Geometry.h"
#include "..\AABBTree\AABBTree.h"

#include <CGAL/squared_distance_3.h>
#include <Eigen/Dense>

#ifdef USE_HULL_POINT_TRIANGLE_DISTANCE
#include <CGAL\global_functions_spherical_kernel_3.h>
#include <CGAL/Polytope_distance_d.h>
#include <CGAL/Polytope_distance_d_traits_3.h>

#ifdef CGAL_USE_GMP
#include <CGAL/Gmpzf.h>
typedef CGAL::Gmpzf ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

typedef CGAL::Polytope_distance_d_traits_3<Kernelf, ET, double> Traits;
typedef CGAL::Polytope_distance_d<Traits>                 Polytope_distance;
#endif


/* ---------------- squared distance -------------------- */

template <typename Primitive, typename RefPrimitive>
float squared_distance(Primitive const & p, RefPrimitive const & rp) 
{
	return POSITIVE_MAX_FLOAT;
}

/* WARNING: just for approximation
 * return distance between point and triangle's support plane,
 * precise method may go to http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
 */
template <>
inline float squared_distance<Point3f, Triangle3f>(Point3f const & point, Triangle3f const & triangle)
{
#ifdef USE_HULL_POINT_TRIANGLE_DISTANCE
	Point3f po[1] = { point };
	Point3f tri[3] = { triangle.vertex(0), triangle.vertex(1), triangle.vertex(2) };
	Polytope_distance pd(po, po + 1, tri, tri + 3);
	assert(pd.is_valid());
	double sqdis = CGAL::to_double(pd.squared_distance_numerator()) /
		CGAL::to_double(pd.squared_distance_denominator());
	return float(sqdis);
#else
	Plane3f plane = triangle.supporting_plane();
	return squared_distance(point, plane);
#endif
	}

template <>
inline float squared_distance<Segment3f, Segment3f>(Segment3f const & segment, Segment3f const & refSegment)
{
	return CGAL::squared_distance(segment, refSegment);
}

template <>
inline float squared_distance<Point3f, Plane3f>(Point3f const & point, Plane3f const & plane)
{
	return CGAL::squared_distance(point, plane);
}

template <>
inline float squared_distance<PointEigen3f, PlaneEigen3f>(PointEigen3f const & point, PlaneEigen3f const & plane)
{
	return std::abs(point.transpose() * plane.normal + plane.distance);
}

template <>
inline float squared_distance<Point3f, Point3f>(Point3f const & point, Point3f const & refPoint)
{
	return CGAL::squared_distance(point, refPoint);
}

/* ---------------- intersection -------------------- */
template <typename Primitive, typename RefPrimitive, typename Foot>
bool intersection(Primitive const & p, RefPrimitive const & rp, float tolerance, Foot & result)
{
	return false;
}

template <>
bool intersection<Point3f, Triangle3f, Eigen::Vector3f>(
	Point3f const & point, Triangle3f const & triangle, float tolerance,
	Eigen::Vector3f & baryceterCoord);

template <>
bool intersection<PointEigen3f, TriangleEigen3f, Eigen::Vector3f>(
	PointEigen3f const & point, TriangleEigen3f const & triangle, float tolerance,
	Eigen::Vector3f & baryceterCoord);

template <>
bool intersection<Segment3f, Segment3f, Eigen::Vector2f>(
	Segment3f const & seg1, Segment3f const & seg2, float tolerance,
	Eigen::Vector2f & barycenterCoord);

#endif