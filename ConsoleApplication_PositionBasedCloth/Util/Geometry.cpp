#include "Geometry.h"
#include "BasicOperations.h"

/* ---------------- intersection -------------------- */

template <>
bool intersection<Point3f, Triangle3f, Eigen::Vector3f>(
	Point3f const & point, Triangle3f const & triangle, float tolerance,
	Eigen::Vector3f & baryceterCoord)
{
	Point3f const & v1 = triangle.vertex(0);
	Point3f const & v2 = triangle.vertex(1);
	Point3f const & v3 = triangle.vertex(2);
	//bool d12 = squared_distance(v1, v2) < DISTANCE_OVERLAP_SQUARED_THRESHOLD;
	//bool d23 = squared_distance(v2, v3) < DISTANCE_OVERLAP_SQUARED_THRESHOLD;
	//bool d31 = squared_distance(v3, v1) < DISTANCE_OVERLAP_SQUARED_THRESHOLD;
	//// point
	//if (d12 && d23 && d31)
	//{
	//	return (squared_distance(point, v1) < tolerance) ? true : false;
	//}
	//// segment
	//else if (d12 || d23 || d31)
	//{
	//	if (d12)
	//		return (squared_distance(point, new Segment3f(v2, v3)) < tolerance) ? true : false;
	//	else if (d23)
	//		return (squared_distance(point, new Segment3f(v3, v1)) < tolerance) ? true : false;
	//	else
	//		return (squared_distance(point, new Segment3f(v1, v2)) < tolerance) ? true : false;
	//}
	//// triangle
	//else
	{
		Plane3f plane = triangle.supporting_plane();
		float sqdis = squared_distance(point, plane);
		if (sqdis > tolerance)
			return false;

		Point3f foot = plane.projection(point);

		Eigen::Vector3f x13 = displacement(v1, v3);
		Eigen::Vector3f x23 = displacement(v2, v3);
		Eigen::Vector3f xf3 = displacement(foot, v3);
		float x13tx13 = x13.squaredNorm();
		float x13tx23 = x13.transpose() * x23;
		float x23tx23 = x23.squaredNorm();

		Eigen::Matrix3f A;
		A << x13tx13, x13tx23, 0.0f,
			x13tx23, x23tx23, 0.0f,
			1.0f, 1.0f, 1.0f;
		Eigen::Vector3f b;
		b << x13.transpose() * xf3, x23.transpose() * xf3, 1.0f;
		Eigen::Vector3f localCoord = A.householderQr().solve(b);
		if (localCoord[0] > -tolerance && localCoord[0] < 1.0f + tolerance &&
			localCoord[1] > -tolerance && localCoord[1] < 1.0f + tolerance &&
			localCoord[2] > -tolerance && localCoord[2] < 1.0f + tolerance)
		{
			baryceterCoord << localCoord[0], localCoord[1], localCoord[2];
			return true;
		}
		return false;
	}
}

template <>
bool intersection<PointEigen3f, TriangleEigen3f, Eigen::Vector3f>(
	PointEigen3f const & point, TriangleEigen3f const & triangle, float tolerance,
	Eigen::Vector3f & baryceterCoord)
{
	//std::cout << "PointEigen3f TriangleEigen3f intersection called" << std::endl;
	Eigen::Vector3f const & v1 = triangle.vertex[0];
	Eigen::Vector3f const & v2 = triangle.vertex[1];
	Eigen::Vector3f const & v3 = triangle.vertex[2];

	PlaneEigen3f plane(triangle);

	float sqdis = squared_distance(point, plane);
	if (sqdis > tolerance)
		return false;

	PointEigen3f foot = plane.projection(point);

	Eigen::Vector3f x13 = v1 - v3;
	Eigen::Vector3f x23 = v2 - v3;
	Eigen::Vector3f xf3 = foot - v3;
	float x13tx13 = x13.squaredNorm();
	float x13tx23 = x13.transpose() * x23;
	float x23tx23 = x23.squaredNorm();

	Eigen::Matrix3f A;
	A << x13tx13, x13tx23, 0.0f,
		x13tx23, x23tx23, 0.0f,
		1.0f, 1.0f, 1.0f;
	Eigen::Vector3f b;
	b << x13.transpose() * xf3, x23.transpose() * xf3, 1.0f;
	Eigen::Vector3f localCoord = A.householderQr().solve(b);
	if (localCoord[0] > -tolerance && localCoord[0] < 1.0f + tolerance &&
		localCoord[1] > -tolerance && localCoord[1] < 1.0f + tolerance &&
		localCoord[2] > -tolerance && localCoord[2] < 1.0f + tolerance)
	{
		baryceterCoord << localCoord[0], localCoord[1], localCoord[2];
		return true;
	}
	return false;
}

template <>
bool intersection<Segment3f, Segment3f, Eigen::Vector2f>(
	Segment3f const & seg1, Segment3f const & seg2, float tolerance,
	Eigen::Vector2f & barycenterCoord)
{
	Point3f v1 = seg1.start();
	Point3f v2 = seg1.end();
	Point3f v3 = seg2.start();
	Point3f v4 = seg2.end();
	Eigen::Vector3f x21 = displacement(v2, v1);
	Eigen::Vector3f x31 = displacement(v3, v1);
	Eigen::Vector3f x43 = displacement(v4, v3);

	// parallel
	if (x21.cross(x43).squaredNorm() < DISTANCE_OVERLAP_SQUARED_THRESHOLD)
	{
		/* cloest points on two segments x1->x2, x3->x4 should overlap,
		* select the point as the length weighted middle point of x1.5 and x3.5
		*/
		float sqLen1 = seg1.squared_length();
		float sqLen2 = seg2.squared_length();
		Point3f seg1middle = interpolate(v1, v2, 0.5f);
		Point3f seg2middle = interpolate(v3, v4, 0.5f);
		Point3f foot = interpolate(seg1middle, seg2middle, sqLen1 / (sqLen1 + sqLen2));
		barycenterCoord << (std::sqrt)(squared_distance(v1, foot) / sqLen1),
			(std::sqrt)(squared_distance(v3, foot) / sqLen2);
		return (AABBoxOf<Point3f, Segment3f>(seg1).intersection(
			AABBoxOf<Point3f, Segment3f>(seg2)))
			? true : false;
	}
	else
	{
		float x21tx21 = x21.squaredNorm();
		float x21tx43 = x21.transpose() * x43;
		float x43tx43 = x43.squaredNorm();

		Eigen::Matrix2f A;
		A << x21tx21, -x21tx43,
			-x21tx43, x43tx43;
		Eigen::Vector2f b;
		b << x21.transpose() * x31, -x43.transpose() * x31;
		Eigen::Vector2f x = A.householderQr().solve(b);
		if (x[0] < 0.0f || x[0] > 1.0f || x[1] < 0.0f || x[1] > 1.0f)
			return false;
		barycenterCoord << x[0], x[1];
		return true;
	}
}

//#define DEBUG_COPLANE
/* -------------- coplanar --------------- */
bool coplane(Eigen::Vector3f const & x1, Eigen::Vector3f const & v1, 
	Eigen::Vector3f const & x2, Eigen::Vector3f const & v2,
	Eigen::Vector3f const & x3, Eigen::Vector3f const & v3, 
	Eigen::Vector3f const & x4, Eigen::Vector3f const & v4, 
	float & time)
{
	Eigen::Vector3f x21 = x1 - x2;
	Eigen::Vector3f x31 = x1 - x3;
	Eigen::Vector3f x41 = x1 - x4;
	Eigen::Vector3f v21 = v1 - v2;
	Eigen::Vector3f v31 = v1 - v3;
	Eigen::Vector3f v41 = v1 - v4;

#ifdef DEBUG_COPLANE
	std::cout << x1 << std::endl << x2 << std::endl << x3 << std::endl << x4 << std::endl;
	std::cout << "x21" << std::endl << x21 << std::endl;
	std::cout << "x31" << std::endl << x31 << std::endl;
	std::cout << "x41" << std::endl << x41 << std::endl;
#endif
	float a0 = x21.cross(x31).transpose() * x41;
	float a1 = float(v21.cross(x31).transpose() * x41) + (x21.cross(v31).transpose() * x41) + (x21.cross(x31).transpose() * v41);
	float a2 = float(v21.cross(v31).transpose() * x41) + (v21.cross(x31).transpose() * v41) + (x21.cross(v31).transpose() * v41);
	float a3 = v21.cross(v31).transpose() * v41;
	
#ifdef DEBUG_COPLANE
	std::cout << " a0 " << a0 << " a1 " << a1 << " a2 " << a2 << " a3 " << a3 << std::endl;
#endif

	//// check the signal of start and end time 
	//if (a0 * (a3 + a2 + a1 + a0) > DISTANCE_OVERLAP_THRESHOLD)
	//	return false;

	for (int _i = 0; _i < 10; _i++)
	{
		float f_deriv = (3 * a3 * time + 2 * a2) * time + a1;
		if (f_deriv < DISTANCE_OVERLAP_THRESHOLD && f_deriv > -DISTANCE_OVERLAP_THRESHOLD)
			return false;
		float f = ((a3 * time + a2) * time + a1) * time + a0;
		time = time - f / f_deriv;
		if (f < DISTANCE_OVERLAP_THRESHOLD && f > -DISTANCE_OVERLAP_THRESHOLD)
			return true;
	}
	return false;
}
