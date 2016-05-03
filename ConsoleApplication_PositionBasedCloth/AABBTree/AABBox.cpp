#include "AABBox.h"


/* --------- specializations ------------ */

template <> template <>
bool AABBox<Point3f>::intersection<Point3f>(Point3f const & point, float tolerance) const
{
	if (point.x() + tolerance < m_minCor.x() || point.x() - tolerance > m_maxCor.x())
		return false;
	if (point.y() + tolerance < m_minCor.y() || point.y() - tolerance > m_maxCor.y())
		return false;
	if (point.z() + tolerance < m_minCor.z() || point.z() - tolerance > m_maxCor.z())
		return false;
	return true;
}

template <> template <>
bool AABBox<Eigen::Vector3f>::intersection<PointEigen3f>(PointEigen3f const & point, float tolerance) const
{
	if (point.x() + tolerance < m_minCor.x() || point.x() - tolerance > m_maxCor.x())
		return false;
	if (point.y() + tolerance < m_minCor.y() || point.y() - tolerance > m_maxCor.y())
		return false;
	if (point.z() + tolerance < m_minCor.z() || point.z() - tolerance > m_maxCor.z())
		return false;
	return true;
}

template <> template <>
float AABBox<Point3f>::squared_distance<Point3f>(Point3f const & point) const
{
	if (this->intersection<Point3f>(point, 0.0f))
		return 0.0f;

	float delta_x = (std::max)((std::max)(this->m_minCor.x() - point.x(), point.x() - this->m_maxCor.x()), 0.0f);
	float delta_y = (std::max)((std::max)(this->m_minCor.y() - point.y(), point.y() - this->m_maxCor.y()), 0.0f);
	float delta_z = (std::max)((std::max)(this->m_minCor.z() - point.z(), point.z() - this->m_maxCor.z()), 0.0f);

	//std::cout << "delta x " << delta_x << " delta y " << delta_y << " delta z " << delta_z << std::endl;

	return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
}

template <> template <>
float AABBox<Eigen::Vector3f>::squared_distance<PointEigen3f>(PointEigen3f const & point) const
{
	if (this->intersection<PointEigen3f>(point, 0.0f))
		return 0.0f;

	float delta_x = (std::max)((std::max)(this->m_minCor.x() - point.x(), point.x() - this->m_maxCor.x()), 0.0f);
	float delta_y = (std::max)((std::max)(this->m_minCor.y() - point.y(), point.y() - this->m_maxCor.y()), 0.0f);
	float delta_z = (std::max)((std::max)(this->m_minCor.z() - point.z(), point.z() - this->m_maxCor.z()), 0.0f);

	//std::cout << "delta x " << delta_x << " delta y " << delta_y << " delta z " << delta_z << std::endl;

	return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
}

template <>
AABBox<Point3f> AABBoxOf<Point3f, Segment3f>(Segment3f const & segment)
{
	return AABBox<Point3f>(segment.start(), segment.end());
}

class Face3fRef;

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, Face3fRef>(Face3fRef const & faceref)
{
	auto const & mesh = faceref.mesh;
	auto const & fid = faceref.faceidx;
	auto range = mesh.vertices_around_face(mesh.halfedge(fid));
	Veridx vids[3];
	int _i = 0;
	for (auto vid : range)
	{
		vids[_i] = vid;
		_i++;
	}
	AABBox<PointEigen3f> box(faceref.posMap[vids[0]], faceref.posMap[vids[1]]);
	box += faceref.posMap[vids[2]];
	return box;
}

template <>
AABBox<PointEigen3f> AABBoxOf<PointEigen3f, Vertex3fRef>(Vertex3fRef const & verref)
{
	return AABBox<PointEigen3f>(verref.posMap[verref.veridx]);
}


struct Edge3fRef;

template <>
AABBox<Eigen::Vector3f> AABBoxOf<Eigen::Vector3f, Edge3fRef>(Edge3fRef const & edgeref)
{
	auto const & mesh = edgeref.mesh;
	auto const & eid = edgeref.edgeidx;
	return AABBox<Eigen::Vector3f>(
		edgeref.posMap[mesh.vertex(eid, 0)], 
		edgeref.posMap[mesh.vertex(eid, 1)]);
}


