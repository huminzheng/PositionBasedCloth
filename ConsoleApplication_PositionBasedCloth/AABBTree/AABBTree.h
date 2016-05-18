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

	AABBTree(std::list<std::pair<PrimitiveRef, AABBox<PointType> > > & other)
	{
		tree.reserve(other.size());
		for (auto & p : other)
		{
			tree.push_back(NodeType(p.second, p.first));
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
	
private:
	std::vector<NodeType> tree;

};

typedef AABBTree<Face3fRef, PointEigen3f> Face3fTree;

typedef AABBTree<Face3fContinuesRef, PointEigen3f> Face3fConTree;

typedef AABBTree<FaceNormalized3fRef, PointEigen3f> Face3fDirTree;

struct ContinuousCollideResult
{
	enum {CLOSE, INTERSECTION} state;
	float time;
	Eigen::Vector3f coord;
};


template<> template<>
std::list<std::pair<typename Face3fTree::Index, Eigen::Vector3f> > *
Face3fTree::contactDetection<Eigen::Vector3f, Eigen::Vector3f>
(Eigen::Vector3f const & point, float tolerance);

template<> template<>
std::list<std::pair<typename Face3fTree::Index, Eigen::Vector3f> > *
Face3fTree::contactDetection<Vertex3fRef, Eigen::Vector3f>
(Vertex3fRef const & point, float tolerance);

template<> template<>
std::list<std::pair<typename Face3fConTree::Index, ContinuousCollideResult> > *
Face3fConTree::contactDetection<Vertex3fContinuesRef, ContinuousCollideResult>
(Vertex3fContinuesRef const & point, float tolerance);

template<> template<>
std::list<std::pair<typename Face3fDirTree::Index, Eigen::Vector3f> > *
Face3fDirTree::contactDetection<Vertex3fRef, Eigen::Vector3f>
(Vertex3fRef const & point, float tolerance);

#endif
