#include "Geometry.h"


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