#ifndef CLOTH_MODEL
#define CLOTH_MODEL

#include "..\Render\Model.h"
#include "..\Util\BasicTypes.h"
#include "Types.h"

#include <Eigen\Core>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL\Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <boost/foreach.hpp>

#include <assimp\types.h>
#include <map>
#include <functional>

#include <map>

class SurfaceMeshObject
{
public:
	SurfaceMeshObject(GLuint edges) :
		EDGES(edges), PolyMesh(new SurfaceMesh3f())
	{}

	SurfaceMesh3f* getMesh() const
	{
		return PolyMesh;
	}

	void import(const Mesh mesh);

	void modelTransform(Eigen::Matrix4f const & matrix);

	bool getVPlanarCoord3f(SurfaceMesh3f::Property_map<Veridx, Point3f> & vph);

	static const std::string pname_texCoords;
	static const std::string pname_vertexPlanarCoords;
	static const std::string pname_vertexNormals;
	static const std::string pname_vertexEigenNormals;
	static const std::string pname_vertexVelocities;
	static const std::string pname_vertexMasses;
	static const std::string pname_vertexInversedMasses;
	static const std::string pname_vertexLastPositions;
	static const std::string pname_vertexCurrentPositions;
	static const std::string pname_vertexPredictPositions;
	static const std::string pname_faceNormals;
	static const std::string pname_faceEigenNormals;

	/* -------- exporters for drawing ---------- */
	void exportPos3fNorm3fBuffer(
		GLfloat* & vertexBuffer, GLfloat* & vertexNormalBuffer, GLuint & vertexSize,
		GLuint* & elementBuffer, GLuint & elementSize) const;

	void exportFaceNorm3fBuffer(
		GLfloat* & fBarycenterBuffer, GLfloat* & fNormalBuffer, GLuint & faceSize) const;

	/* -------- set vertices' properties --------- */
	bool useVTexCoord2DAsVPlanarCoord3f();

	void addPositionsProperty()
	{
		SurfaceMesh3f::Property_map<Veridx, Point3f> vprop_handle =
			PolyMesh->add_property_map<Veridx, Point3f>(pname_vertexLastPositions).first;
		for (Veridx vhd : PolyMesh->vertices())
			vprop_handle[vhd] = PolyMesh->point(vhd);
		vprop_handle =
			PolyMesh->add_property_map<Veridx, Point3f>(pname_vertexPredictPositions).first;
		for (Veridx vhd : PolyMesh->vertices())
			vprop_handle[vhd] = PolyMesh->point(vhd);
	}

	/* -------- getters and setters --------- */
	Eigen::VectorXf getPositions() const;
	void setPositions(Eigen::VectorXf const & positions);

	void refreshNormals();

	size_t getVertexSize() { return VERTEX_SIZE; }
	size_t getFaceSize() { return FACE_SIZE; }
	size_t getEdgeSize() { return EDGE_SIZE; }
	
	std::map<Veridx, GLuint> const * getVertices2indices() { return vertices2indices; }
	std::map<Faceidx, GLuint> const * getFaces2indices() { return faces2indices; }
	std::map<Edgeidx, GLuint> const * getEdges2indices() { return edges2indices; }

private:
	const GLuint EDGES;

	SurfaceMesh3f* PolyMesh;

	/* mesh properties hash tables */
	std::map<Veridx, GLuint> * vertices2indices;
	std::map<Faceidx, GLuint> * faces2indices;
	std::map<Edgeidx, GLuint> * edges2indices;

	GLuint VERTEX_SIZE, FACE_SIZE, EDGE_SIZE;

};

#endif
