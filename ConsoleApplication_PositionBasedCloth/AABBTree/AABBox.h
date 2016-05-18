#ifndef AABBOX_H
#define AABBOX_H

#include "../Util/BasicTypes.h"
#include "../Util/BasicOperations.h"
#include "../Model/Types.h"
#include "../Model/PrimitiveReference.h"


template <typename PointType>
class AABBox
{
public:
	AABBox(PointType minCor, PointType maxCor) :
		m_minCor(minCor), m_maxCor(maxCor) {}

	AABBox(PointType point) :
		AABBox(point, point) {}

	/* accroding to http://www.cplusplus.com/articles/y8hv0pDG/ */
	AABBox(AABBox<PointType> const & other) :
		m_minCor(other.m_minCor), m_maxCor(other.m_maxCor) 
	{
		//std::cout << "WARNING::call AABB copy constructor" << std::endl;
	}

	AABBox<PointType> & operator=(AABBox<PointType> const & other)
	{
		this->m_minCor = other.m_minCor;
		this->m_maxCor = other.m_maxCor;
		std::cout << "WARNING::call AABB copy assignment" << std::endl;
		return *this;
	}

	AABBox(AABBox<PointType> && other) :
		m_minCor(std::move(other.m_minCor)), m_maxCor(std::move(other.m_maxCor)) 
	{
		//std::cout << "WARNING::call AABB move constructor" << std::endl;
	}


	AABBox<PointType> & operator=(AABBox<PointType> && other)
	{
		if (this = &other)
			return *this;
		this.m_minCor = std::move(other.m_minCor);
		this.m_maxCor = std::move(other.m_maxCor);
		//std::cout << "WARNING::call AABB move assignment" << std::endl;
		return *this;
	}

	AABBox<PointType> & operator+=(AABBox<PointType> const & rhs)
	{
		this->m_minCor.x() = (std::min)(this->m_minCor.x(), rhs.m_minCor.x());
		this->m_minCor.y() = (std::min)(this->m_minCor.y(), rhs.m_minCor.y());
		this->m_minCor.z() = (std::min)(this->m_minCor.z(), rhs.m_minCor.z());
		this->m_maxCor.x() = (std::max)(this->m_maxCor.x(), rhs.m_maxCor.x());
		this->m_maxCor.y() = (std::max)(this->m_maxCor.y(), rhs.m_maxCor.y());
		this->m_maxCor.z() = (std::max)(this->m_maxCor.z(), rhs.m_maxCor.z());
		return *this;
	}

	AABBox<PointType> & operator+=(PointType const & point)
	{
		this->m_minCor.x() = (std::min)(this->m_minCor.x(), point.x());
		this->m_minCor.y() = (std::min)(this->m_minCor.y(), point.y());
		this->m_minCor.z() = (std::min)(this->m_minCor.z(), point.z());
		this->m_maxCor.x() = (std::max)(this->m_maxCor.x(), point.x());
		this->m_maxCor.y() = (std::max)(this->m_maxCor.y(), point.y());
		this->m_maxCor.z() = (std::max)(this->m_maxCor.z(), point.z());
		return *this;
	}

	friend AABBox<PointType> operator+(AABBox<PointType> lhs, AABBox<PointType> const & rhs)
	{
		lhs += rhs;
		return lhs;
	}

	friend AABBox<PointType> operator+(AABBox<PointType> lhs, PointType const & rhs)
	{
		lhs += rhs;
		return lhs;
	}

	PointType const & minCor() const
	{
		return m_minCor;
	}

	PointType const & maxCor() const
	{
		return m_maxCor;
	}

	template <typename Obj>
	float squared_distance(Obj const & obj) const;

	template <typename Obj>
	bool intersection(Obj const & obj, float tolerance) const;

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

private:
	PointType m_minCor;
	PointType m_maxCor;

};

template <typename PointType, typename PrimitiveRef>
AABBox<PointType> AABBoxOf(PrimitiveRef const & ref);

/* ---------- specializations ---------- */
template <> template <>
bool AABBox<Point3f>::intersection<Point3f>(Point3f const & point, float tolerance) const;

template <> template <>
bool AABBox<PointEigen3f>::intersection<PointEigen3f>(PointEigen3f const & point, float tolerance) const;

template <> template <>
float AABBox<Point3f>::squared_distance<Point3f>(Point3f const & point) const;

template <> template <>
float AABBox<PointEigen3f>::squared_distance<PointEigen3f>(PointEigen3f const & point) const;

template <> template <>
float AABBox<PointEigen3f>::squared_distance<AABBox<PointEigen3f> >(AABBox<PointEigen3f> const & box) const;

template <>
AABBox<Point3f> AABBoxOf<Point3f, Segment3f>(Segment3f const & segment);

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, Face3fRef>(Face3fRef const & faceref);

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, FaceNormalized3fRef>(FaceNormalized3fRef const & faceref);

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, Vertex3fRef>(Vertex3fRef const & verref);

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, Edge3fRef>(Edge3fRef const & edgeref);

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, Face3fContinuesRef>(Face3fContinuesRef const & faceconref);

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, Vertex3fContinuesRef>(Vertex3fContinuesRef const & verconref);

#endif
