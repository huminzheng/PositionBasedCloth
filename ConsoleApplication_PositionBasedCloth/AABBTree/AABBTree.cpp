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
		// should near the bounding box
		if (box.squared_distance(point) >= tolerance)
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
			triangle.vertex[_i] = faceref.posMap[vid];
			_i++;
		}
		if (!intersection(point, triangle, tolerance, coord))
			continue;
		result->push_back(Pair(idx, coord));
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
