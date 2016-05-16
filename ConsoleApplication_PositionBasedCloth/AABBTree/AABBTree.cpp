#include "AABBTree.h"
#include "..\Util\Geometry.h"

//template<> template<>
//std::list<std::pair<typename AABBTree<Triangle3f, Point3f>::Index, Eigen::Vector3f> > *
//AABBTree<Triangle3f, Point3f>::contactDetection<Point3f, Eigen::Vector3f>
//(Point3f const & point, float tolerance)
//{
//	typedef AABBTree<Triangle3f, Point3f>::Index Idx;
//	typedef std::pair<Idx, Eigen::Vector3f> Pair;
//	std::list< Pair* > * result = new std::list<Pair* >();
//	Idx idx = 0;
//	for (auto iter = tree->begin(); iter != tree->end(); ++iter, ++idx)
//	{
//		auto pairPtr = (*iter);
//		AABBox<Point3f> * box = pairPtr->first;
//		Triangle3f const * tri = pairPtr->second;
//		float sqdis = 0.0f;
//		// should near the bounding box
//		if (box->squared_distance(point) >= tolerance)
//			continue;
//		//std::cout << "box " << std::endl
//		//	<< box->minCor() << std::endl << box->maxCor() << std::endl;
//		Eigen::Vector3f coord;
//		if (!intersection(point, *tri, tolerance, coord))
//			continue;
//		result->push_back(new Pair(idx, coord));
//	}
//	return result;
//}

template<> template<>
std::list<std::pair<typename Face3fTree::Index, Eigen::Vector3f> > *
Face3fTree::contactDetection<Eigen::Vector3f, Eigen::Vector3f>
(Eigen::Vector3f const & point, float tolerance)
{
	typedef std::pair<Face3fTree::Index, Eigen::Vector3f> Pair;
	std::list<Pair> * result = new std::list<Pair>();
	Face3fTree::Index idx = 0;
	for (auto iter = tree.begin(); iter != tree.end(); ++iter, ++idx)
	{
		AABBox<PointEigen3f> const & box = iter->first;
		Face3fRef const & faceref = iter->second;
		float sqdis = 0.0f;
		{
			// should near the bounding box
			if (box.squared_distance(point) >= tolerance)
				continue;
		}
		//std::cout << "box " << std::endl
		//	<< box->minCor() << std::endl << box->maxCor() << std::endl;
		Eigen::Vector3f coord;
		TriangleEigen3f triangle;
		{
			int _i = 0;
			auto const & mesh = faceref.mesh;
			//std::cout << "face " << faceref.faceidx << std::endl;
			for (auto vid : mesh.vertices_around_face(mesh.halfedge(faceref.faceidx)))
			{
				triangle.vertex[_i] = faceref.point(vid);
				_i++;
			}
		}
		{
			if (!intersection(point, triangle, tolerance, coord))
				continue;
		}
		result->push_back(Pair(idx, coord));
	}
	return result;
}

template<> template<>
std::list<std::pair<typename Face3fTree::Index, Eigen::Vector3f> > *
Face3fTree::contactDetection<Vertex3fRef, Eigen::Vector3f>
(Vertex3fRef const & point, float tolerance)
{
	typedef std::pair<Face3fTree::Index, Eigen::Vector3f> Pair;
	std::list<Pair> * result = new std::list<Pair>();
	Face3fTree::Index idx = 0;
	for (auto iter = tree.begin(); iter != tree.end(); ++iter, ++idx)
	{
		AABBox<PointEigen3f> const & box = iter->first;
		Face3fRef const & faceref = iter->second;
		float sqdis = 0.0f;
		// should near the bounding box
		if (box.squared_distance(point.point(point.veridx)) >= tolerance)
			continue;
		//std::cout << "box " << std::endl
		//	<< box->minCor() << std::endl << box->maxCor() << std::endl;
		Eigen::Vector3f coord;
		TriangleEigen3f triangle;

		int _i = 0;
		auto const & mesh = faceref.mesh;
		//std::cout << "face " << faceref.faceidx << std::endl;
		for (auto vid : mesh.vertices_around_face(mesh.halfedge(faceref.faceidx)))
		{
			//std::cout << "vertex id " << vid << std::endl;
			//std::cout << "\tpos " << faceref.point(vid) << std::endl;
			triangle.vertex[_i] = faceref.point(vid);
			_i++;
		}

		if (!intersection(point.point(point.veridx), triangle, tolerance, coord))
			continue;

		result->push_back(Pair(idx, coord));
	}
	return result;
}

template<> template<>
std::list<std::pair<typename Face3fConTree::Index, ContinuousCollideResult> > *
Face3fConTree::contactDetection<Vertex3fContinuesRef, ContinuousCollideResult>
(Vertex3fContinuesRef const & point, float tolerance)
{
	typedef std::pair<Face3fTree::Index, ContinuousCollideResult> Pair;
	std::list<Pair> * result = new std::list<Pair>();
	Face3fTree::Index idx = 0;
	for (auto iter = tree.begin(); iter != tree.end(); ++iter, ++idx)
	{
		AABBox<PointEigen3f> const & box = iter->first;
		Face3fContinuesRef const & faceref = iter->second;
		float sqdis = 0.0f;
		// should near the bounding box
		if (box.squared_distance(AABBoxOf<PointEigen3f, Vertex3fContinuesRef>(point)) >= tolerance)
			continue;
		//std::cout << "box " << std::endl
		//	<< box->minCor() << std::endl << box->maxCor() << std::endl;
		Veridx vids[3];
		int _i = 0;
		for (auto vid : faceref.mesh.vertices_around_face(faceref.mesh.halfedge(faceref.faceidx)))
		{
			vids[_i] = vid;
			_i++;
		}
		if (vids[0] == point.veridx || vids[1] == point.veridx || vids[2] == point.veridx)
			continue;
		Eigen::Vector3f x1 = faceref.point0(vids[0]);
		Eigen::Vector3f x2 = faceref.point0(vids[1]);
		Eigen::Vector3f x3 = faceref.point0(vids[2]);
		Eigen::Vector3f x4 = point.point0(point.veridx);
		Eigen::Vector3f v1 = faceref.point1(vids[0]) - x1;
		Eigen::Vector3f v2 = faceref.point1(vids[1]) - x2;
		Eigen::Vector3f v3 = faceref.point1(vids[2]) - x3;
		Eigen::Vector3f v4 = point.point1(point.veridx) - x4;
		Eigen::Vector3f v_relative = (v1 + v2 + v3) / 2 - v4;
		float v_relative_norm = v_relative.norm();
		if (v_relative_norm <= DISTANCE_OVERLAP_THRESHOLD)
			continue;

		float coplaneTime = 0.5f;
		bool coplaned = coplane(x1, v1, x2, v2, x3, v3, x4, v4, coplaneTime);
		//bool intersected = false;

		if (!coplaned || (coplaneTime < 0.0f || coplaneTime > 1.0f /*+ tolerance / v_relative_norm*/))
			continue;
		//if (coplaned && coplaneTime >= 0.0f && coplaneTime <= 1.0f)
		//	intersected = true;

		TriangleEigen3f triangle;
		triangle.vertex[0] = x1 + coplaneTime * v1;
		triangle.vertex[1] = x2 + coplaneTime * v2;
		triangle.vertex[2] = x3 + coplaneTime * v3;
		
		Eigen::Vector3f coord;
		if (!intersection<PointEigen3f, TriangleEigen3f, Eigen::Vector3f>(x4 + coplaneTime * v4, triangle, tolerance, coord))
			continue;
		
		ContinuousCollideResult res;
		res.state = /*intersected ? */ContinuousCollideResult::INTERSECTION/* : ContinuousCollideResult::CLOSE*/;
		res.time = coplaneTime;
		res.coord << coord;
		result->push_back(Pair(idx, std::move(res)));
	}
	return result;
}

//template<> template<>
//std::list<std::pair<typename AABBTree<Segment3f, Point3f>::Index, Eigen::Vector2f> * > *
//AABBTree<Segment3f, Point3f>::contactDetection<Segment3f, Eigen::Vector2f>
//(Segment3f const & segment, float tolerance)
//{
//	typedef AABBTree<Segment3f, Point3f>::Index Idx;
//	typedef std::pair<Idx, Eigen::Vector2f> Pair;
//	std::list<Pair *> * result = new std::list<Pair *>();
//	Idx idx = 0;
//	for (auto iter = tree->begin(); iter != tree->end(); ++iter, ++idx)
//	{
//		auto pairPtr = (*iter);
//		AABBox<Point3f> * box = pairPtr->first;
//		Segment3f const * seg = pairPtr->second;
//		float sqdis = 0.0f;
//		// should near the bounding box
//		if (!box->intersection(AABBoxOf<Point3f, Segment3f>(segment)))
//			continue;
//		//std::cout << "box " << std::endl
//		//	<< box->minCor() << std::endl << box->maxCor() << std::endl;
//
//		//sqdis = squared_distance(*seg, segment);
//		//if (sqdis > tolerance)
//		//	continue;
//		Eigen::Vector2f coord;
//		if (!intersection(*seg, segment, tolerance, coord))
//			continue;
//		result->push_back(new Pair(idx, coord));
//	}
//	return result;
//}
