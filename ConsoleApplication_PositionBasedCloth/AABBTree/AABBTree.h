#ifndef AABBTREE_H
#define AABBTREE_H

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

/* --------------- AABBox class definition --------------- */

template <typename PointType>
class AABBox
{
public:
	AABBox(PointType minCor, PointType maxCor) :
		m_minCor(minCor), m_maxCor(maxCor)
	{
		m_center = interpolate(m_minCor, m_maxCor, 0.5f);
	}

	AABBox(AABBox<PointType> && box) noexcept :
	m_minCor(std::move(box.m_minCor)), m_maxCor(std::move(box.m_maxCor)) {}

	//float squared_distance(AABBox const & rhs) const
	//{
	//	if (intersection(rhs))
	//		return 0.0f;

	//	float delta_x = (std::max)((std::min)(rhs.m_maxCor.x() - this->m_minCor.x(), this->m_maxCor.x() - rhs.m_minCor.x()), 0.0f);
	//	float delta_y = (std::max)((std::min)(rhs.m_maxCor.y() - this->m_minCor.y(), this->m_maxCor.y() - rhs.m_minCor.y()), 0.0f);
	//	float delta_z = (std::max)((std::min)(rhs.m_maxCor.z() - this->m_minCor.z(), this->m_maxCor.z() - rhs.m_minCor.z()), 0.0f);

	//	return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
	//}

	template <typename Obj>
	float squared_distance(Obj const & obj)
	{
		BOOST_STATIC_ASSERT(sizeof(T) == 0);
	}

	template <typename Obj>
	bool intersection(Obj const & obj, float tolerance)
	{
		BOOST_STATIC_ASSERT(sizeof(T) == 0);
	}

	bool intersection(AABBox<PointType> const & rhs) const
	{
		if (this->m_minCor.x() > rhs.m_maxCor.x() || this->m_maxCor.x() < rhs.m_minCor.x())
			return false;
		if (this->m_minCor.y() > rhs.m_maxCor.y() || this->m_maxCor.y() < rhs.m_minCor.y())
			return false;
		if (this->m_minCor.z() > rhs.m_maxCor.z() || this->m_maxCor.z() < rhs.m_minCor.z())
			return false;
		return true;
	}
	
	friend AABBox<PointType> const & operator+(AABBox<PointType> const & lhs, AABBox<PointType> const & rhs)
	{
		return AABBox<PointType>(
			PointType((std::min)(lhs.m_minCor.x(), rhs.m_minCor.x()),
				(std::min)(lhs.m_minCor.y(), rhs.m_minCor.y()),
				(std::min)(lhs.m_minCor.z(), rhs.m_minCor.z())),
			PointType((std::min)(lhs.m_maxCor.x(), rhs.m_maxCor.x()),
				(std::min)(lhs.m_maxCor.y(), rhs.m_maxCor.y()),
				(std::min)(lhs.m_maxCor.z(), rhs.m_maxCor.z()))
			);
	}

	friend AABBox<PointType> const & operator+(AABBox<PointType> const & lhs, PointType const & rhs)
	{
		return AABBox<PointType>(
			PointType(
			(std::min)(lhs.m_minCor.x(), rhs.x()),
			(std::min)(lhs.m_minCor.y(), rhs.y()),
			(std::min)(lhs.m_minCor.z(), rhs.z())),
			PointType(
			(std::max)(lhs.m_maxCor.x(), rhs.x()),
			(std::max)(lhs.m_maxCor.y(), rhs.y()),
			(std::max)(lhs.m_maxCor.z(), rhs.z())));
	}

	PointType const & minCor()
	{
		return m_minCor;
	}

	PointType const & maxCor()
	{
		return m_maxCor;
	}


private:
	PointType const m_minCor;
	PointType const m_maxCor;
	PointType m_center;

};

template <> template <>
bool AABBox<Point3f>::intersection<Point3f>(Point3f const & point, float tolerance);

template <> template <>
float AABBox<Point3f>::squared_distance<Point3f>(Point3f const & point);

template <typename PointType, typename Obj>
AABBox<PointType> const & AABBoxOf(Obj const & obj);

template <>
AABBox<Point3f> const & AABBoxOf<Point3f, Segment3f>(Segment3f const & segment);
//{
//	Obj::unimplemented;
//}

/* --------------- AABBTree class definition --------------- */

template <typename Primitive, typename PointType>
class AABBTree
{
public:
	typedef std::pair<AABBox<PointType> *, Primitive *> NodeType;
	typedef int Index;

	template <typename IterType, typename toPair>
	AABBTree(IterType & begin, IterType const & end, toPair & topair, int reserveSize = -1):
		tree(new std::vector<NodeType *>())
	{
		if (reserveSize > 0)
			tree->reserve(reserveSize);
		for (; begin != end; ++begin)
		{
			NodeType * p = topair(begin);
			tree->push_back(p);
		}
	}

	std::vector<NodeType *> const * getBoxes()
	{
		return tree;
	}

	~AABBTree()
	{
		for (NodeType * item : tree)
		{
			delete item->first;
			delete item->second;
			delete item;
		}
	}

	int size() const
	{
		return tree->size();
	}

	NodeType const * at(Index idx) const
	{
		return (*tree)[idx];
	}

	void exportAABBoxPositions(GLfloat * & verticesBuffer, GLuint & pointSize) const
	{
		pointSize = tree->size() * 2;
		verticesBuffer = new GLfloat[pointSize * 3];
		int pivot = 0;
		for (auto iter = tree->begin(); iter != tree->end(); ++iter)
		{
			AABBox<Point3f> * box = (*iter)->first;
			Point3f const p = box->minCor();
			verticesBuffer[pivot++] = p.x();
			verticesBuffer[pivot++] = p.y();
			verticesBuffer[pivot++] = p.z();
			Point3f const q = box->maxCor();
			verticesBuffer[pivot++] = q.x();
			verticesBuffer[pivot++] = q.y();
			verticesBuffer[pivot++] = q.z();
		}
	}

	template <typename Obj, typename Foot>
	std::list<std::pair<Index, Foot> * > *
	contactDetection(Obj const & obj, float tolerance);
	//{
	//	//std::cout << "WARNING: default contact detection calling" << std::endl;
	//	return new std::list<Index>();
	//}
	
private:
	std::vector<NodeType *> * tree;

};

/* --------------- conversions implementations --------------- */

struct FaceIter2Triangle3fAABBoxPair
{
	typedef SurfaceMesh3f::Property_map<Veridx, Point3f> VPropertyMap;

	SurfaceMesh3f const * const m_mesh;
	VPropertyMap const * const m_propertyMap;

	FaceIter2Triangle3fAABBoxPair(SurfaceMesh3f * mesh, VPropertyMap * propertyMap = nullptr) :
		m_mesh(mesh), m_propertyMap(propertyMap) {}

	AABBTree<Triangle3f, Point3f>::NodeType * operator() (/*SurfaceMesh3f const * mesh, */Faceiter const & iter) const
	{ 
		Faceidx fid = *iter;
		//std::cout << "face index " << fid << std::endl;
		std::vector<Point3f> ps;
		ps.reserve(3);
		float maxx = NEGATIVE_MAX_FLOAT;
		float maxy = NEGATIVE_MAX_FLOAT;
		float maxz = NEGATIVE_MAX_FLOAT;
		float minx = POSITIVE_MAX_FLOAT;
		float miny = POSITIVE_MAX_FLOAT;
		float minz = POSITIVE_MAX_FLOAT;
		//std::cout << "initial" << std::endl << minx << " " << miny << " " << minz << std::endl
		//	<< maxx << " " << maxy << " " << maxz << std::endl;
		auto range = m_mesh->vertices_around_face(m_mesh->halfedge(fid));
		//CGAL::Vertex_around_face_iterator<SurfaceMesh3f> vbegin , vend;
		for (auto vbegin = range.begin(); vbegin != range.end(); ++vbegin)
		{
			Veridx vid = *vbegin;
			//std::cout << "vertex index" << vid << std::endl;
			Point3f p = (this->m_propertyMap) ? (*m_propertyMap)[vid] : m_mesh->point(vid);
			ps.push_back(p);
			//std::cout << "point" << std::endl << p << std::endl;
			minx = (std::min)(minx, p.x());
			miny = (std::min)(miny, p.y());
			minz = (std::min)(minz, p.z());
			maxx = (std::max)(maxx, p.x());
			maxy = (std::max)(maxy, p.y());
			maxz = (std::max)(maxz, p.z());
			//std::cout << "procedure" << std::endl << minx << " " << miny << " " << minz << std::endl
			//	<< maxx << " " << maxy << " " << maxz << std::endl;
		}
		assert(ps.size() == 3);
		auto box = new AABBox<Point3f>(Point3f(minx, miny, minz), Point3f(maxx, maxy, maxz));
		auto tri = new Triangle3f(ps[0], ps[1], ps[2]);
		//std::cout << "box" << std::endl << box->minCor() << std::endl << box->maxCor() << std::endl;
		//std::cout << "triangle" << std::endl << ps[0] << std::endl << ps[1] << std::endl << ps[2] << std::endl;
		return new AABBTree<Triangle3f, Point3f>::NodeType(box, tri);
	}

};

struct EdgeIter2Segment3fAABBoxPair
{

	typedef SurfaceMesh3f::Property_map<Veridx, Point3f> VPropertyMap;

	SurfaceMesh3f const * const m_mesh;
	VPropertyMap const * const m_propertyMap;

	EdgeIter2Segment3fAABBoxPair(SurfaceMesh3f * mesh, VPropertyMap * propertyMap = nullptr) : 
		m_mesh(mesh), m_propertyMap(propertyMap) {}

	AABBTree<Segment3f, Point3f>::NodeType * operator() (/*SurfaceMesh3f const * mesh, */Edgeiter const & iter) const
	{
		Edgeidx eid = *iter;
		Veridx vid0 = m_mesh->vertex(eid, 0);
		Veridx vid1 = m_mesh->vertex(eid, 1);
		Point3f v0 = (this->m_propertyMap) ? (*m_propertyMap)[vid0] : m_mesh->point(vid0);
		Point3f v1 = (this->m_propertyMap) ? (*m_propertyMap)[vid1] : m_mesh->point(vid1);
		float maxx = (std::max)(v0.x(), v1.x());
		float maxy = (std::max)(v0.y(), v1.y());
		float maxz = (std::max)(v0.z(), v1.z());
		float minx = (std::min)(v0.x(), v1.x());
		float miny = (std::min)(v0.y(), v1.y());
		float minz = (std::min)(v0.z(), v1.z());
		auto box = new AABBox<Point3f>(Point3f(minx, miny, minz), Point3f(maxx, maxy, maxz));
		auto seg = new Segment3f(v0, v1);
		//std::cout << "box" << std::endl << box->minCor() << std::endl << box->maxCor() << std::endl;
		//std::cout << "triangle" << std::endl << ps[0] << std::endl << ps[1] << std::endl << ps[2] << std::endl;
		return new AABBTree<Segment3f, Point3f>::NodeType(box, seg);
	}

};
#endif