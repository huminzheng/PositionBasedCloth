#ifndef AABBTREE_H
#define AABBTREE_H

#include "AABBox.h"
#include "../Util/BasicTypes.h"
#include "../Util/BasicOperations.h"
#include "../Model/Types.h"

#include <boost\geometry.hpp>
#include <boost\geometry\index\rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost\foreach.hpp>

#include <algorithm>
#include <list>
#include <vector>

template <typename PrimitiveRef, typename PointType>
class AABBTree
{
public:
	typedef std::pair<AABBox<PointType>, PrimitiveRef> NodeType;
	typedef int Index;

	template <typename IterType, typename IterToNode>
	AABBTree(IterType & begin, IterType const & end, 
		IterToNode & toNode, int reserveSize = -1)
	{
		if (reserveSize > 0)
			tree.reserve(reserveSize);
		for (; begin != end; ++begin)
		{
			NodeType node = toNode(begin);
			tree.push_back(std::move(node));
		}
	}

	std::vector<NodeType const &> const & getBoxes() const
	{
		return tree;
	}

	size_t size() const
	{
		return tree.size();
	}

	NodeType const & at(Index idx) const
	{
		return tree[idx];
	}

	void exportAABBoxPositions(GLfloat * & verticesBuffer, GLuint & pointSize) const
	{
		pointSize = tree.size() * 2;
		verticesBuffer = new GLfloat[pointSize * 3];
		int pivot = 0;
		for (auto iter = tree.begin(); iter != tree.end(); ++iter)
		{
			AABBox<Point3f> const & box = (*iter).first;
			Point3f const p = box.minCor();
			verticesBuffer[pivot++] = p.x();
			verticesBuffer[pivot++] = p.y();
			verticesBuffer[pivot++] = p.z();
			Point3f const q = box.maxCor();
			verticesBuffer[pivot++] = q.x();
			verticesBuffer[pivot++] = q.y();
			verticesBuffer[pivot++] = q.z();
		}
	}

	template <typename Obj, typename Foot>
	std::list<std::pair<Index, Foot> > *
	contactDetection(Obj const & obj, float tolerance);
	//{
	//	//std::cout << "WARNING: default contact detection calling" << std::endl;
	//	return new std::list<Index>();
	//}
	
private:
	std::vector<NodeType> tree;

};

class Face3fRef;

typedef AABBTree<Face3fRef, PointEigen3f> Face3fTree;

/* --------------- conversions implementations --------------- */

class FaceIter2Triangle3fAABBoxPair
{
	typedef SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> VPropertyMap;
	
	VPropertyMap const & m_propertyMap;
	SurfaceMesh3f const & m_mesh;

public:
	FaceIter2Triangle3fAABBoxPair(SurfaceMesh3f const & mesh, VPropertyMap const & propertyMap) :
		m_mesh(mesh), m_propertyMap(propertyMap) {}

	Face3fTree::NodeType operator() (Faceiter const & iter) const
	{ 
		Faceidx fid = *iter;
		Face3fRef ref(m_mesh, fid, m_propertyMap);
		auto box = AABBoxOf<PointEigen3f, Face3fRef>(ref);
		auto node = Face3fTree::NodeType(std::move(box), std::move(ref));
		return std::move(node);
	}

};

//class TestToPair
//{
//public:
//	Face3fTree::NodeType operator() (Faceiter const & iter) const
//	{
//		AABBox<PointEigen3f> box(Eigen::Vector3f::Zero());
//		return Face3fTree::NodeType(box, Face3fRef());
//	}
//};

//struct EdgeIter2Segment3fAABBoxPair
//{
//
//	typedef SurfaceMesh3f::Property_map<Veridx, Point3f> VPropertyMap;
//
//	SurfaceMesh3f const * const m_mesh;
//	VPropertyMap const * const m_propertyMap;
//
//	EdgeIter2Segment3fAABBoxPair(SurfaceMesh3f * mesh, VPropertyMap * propertyMap = nullptr) : 
//		m_mesh(mesh), m_propertyMap(propertyMap) {}
//
//	AABBTree<Segment3f, Point3f>::NodeType * operator() (/*SurfaceMesh3f const * mesh, */Edgeiter const & iter) const
//	{
//		Edgeidx eid = *iter;
//		Veridx vid0 = m_mesh->vertex(eid, 0);
//		Veridx vid1 = m_mesh->vertex(eid, 1);
//		Point3f v0 = (this->m_propertyMap) ? (*m_propertyMap)[vid0] : m_mesh->point(vid0);
//		Point3f v1 = (this->m_propertyMap) ? (*m_propertyMap)[vid1] : m_mesh->point(vid1);
//		float maxx = (std::max)(v0.x(), v1.x());
//		float maxy = (std::max)(v0.y(), v1.y());
//		float maxz = (std::max)(v0.z(), v1.z());
//		float minx = (std::min)(v0.x(), v1.x());
//		float miny = (std::min)(v0.y(), v1.y());
//		float minz = (std::min)(v0.z(), v1.z());
//		auto box = new AABBox<Point3f>(Point3f(minx, miny, minz), Point3f(maxx, maxy, maxz));
//		auto seg = new Segment3f(v0, v1);
//		//std::cout << "box" << std::endl << box->minCor() << std::endl << box->maxCor() << std::endl;
//		//std::cout << "triangle" << std::endl << ps[0] << std::endl << ps[1] << std::endl << ps[2] << std::endl;
//		return new AABBTree<Segment3f, Point3f>::NodeType(box, seg);
//	}
//};

#endif
