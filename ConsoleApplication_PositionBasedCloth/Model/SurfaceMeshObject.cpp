#include "SurfaceMeshObject.h"
#include "..\Util\BasicOperations.h"

#include <iostream>


const std::string SurfaceMeshObject::pname_texCoords = "v:texture_coordinates";
const std::string SurfaceMeshObject::pname_vertexPlanarCoords = "v:vertex_planar_coordinates";
const std::string SurfaceMeshObject::pname_vertexNormals = "v:vertex_normals";
const std::string SurfaceMeshObject::pname_vertexEigenNormals = "v:vertex_eigen_normals";
const std::string SurfaceMeshObject::pname_vertexMasses = "v:vertex_masses";
const std::string SurfaceMeshObject::pname_vertexInversedMasses = "v:vertex_inversed_masses";
const std::string SurfaceMeshObject::pname_vertexVelocities = "v:vertex_velocities";
const std::string SurfaceMeshObject::pname_vertexLastPositions = "v:vertex_last_positions";
const std::string SurfaceMeshObject::pname_vertexCurrentPositions = "v:vertex_current_positions";
const std::string SurfaceMeshObject::pname_vertexPredictPositions = "v:vertex_predict_positions";
const std::string SurfaceMeshObject::pname_faceNormals = "v:face_normals";
const std::string SurfaceMeshObject::pname_faceEigenNormals = "v:face_eigen_normals";

void SurfaceMeshObject::import(const Mesh mesh)
{
	/* load vertexes */
	std::map<GLuint, Veridx> vindices2vhandles;
	for (size_t i = 0; i < mesh.vertices.size(); ++i)
	{
		vindices2vhandles[i] = this->PolyMesh->add_vertex(
			Point3f(mesh.vertices[i].Position[0], mesh.vertices[i].Position[1], mesh.vertices[i].Position[2]));
	}
	/* set faces */
	std::vector<Veridx> face_vhandles;
	for (GLuint vindex : mesh.indices)
	{
		face_vhandles.push_back(vindices2vhandles[vindex]);
		if (face_vhandles.size() % EDGES == 0)
		{
			this->PolyMesh->add_face(face_vhandles);
			face_vhandles.clear();
		}
	}
	/* load texcoord 2d */
	SurfaceMesh3f::Property_map<Veridx, Point3f> texCoords = PolyMesh->add_property_map<Veridx, Point3f>(pname_texCoords).first;
	for (size_t _i = 0; _i < mesh.vertices.size(); ++_i)
	{
		texCoords[vindices2vhandles[_i]] = Point3f(mesh.vertices[_i].TexCoords[0], mesh.vertices[_i].TexCoords[1], 0.0f);
	}
	///* compute face normal */
	///* generate vertex normal conditioning on face normal */
	////PolyArrayMesh & mesh = *PolyMesh;
	//SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals =
	//	PolyMesh->add_property_map<Faceidx, Vec3f>(pname_faceNormals, CGAL::NULL_VECTOR).first;
	//SurfaceMesh3f::Property_map<Veridx, Vec3f> vertexNormals =
	//	PolyMesh->add_property_map<Veridx, Vec3f>(pname_vertexNormals, CGAL::NULL_VECTOR).first;
	//CGAL::Polygon_mesh_processing::compute_normals(*PolyMesh, vertexNormals, faceNormals,
	//	CGAL::Polygon_mesh_processing::parameters::vertex_point_map(PolyMesh->points()).geom_traits(Kernelf()));

	/* initial sizes */
	VERTEX_SIZE = PolyMesh->number_of_vertices();
	FACE_SIZE = PolyMesh->number_of_faces();
	EDGE_SIZE = PolyMesh->number_of_edges();

	/* initial global indices */
	vertices2indices = new std::map<Veridx, GLuint>();
	faces2indices = new std::map<Faceidx, GLuint>();
	edges2indices = new std::map<Edgeidx, GLuint>();
	GLuint index = 0;
	for (Veridx vhd : PolyMesh->vertices())
	{
		(*vertices2indices)[vhd] = index++;
	}
	index = 0;
	for (Faceidx fhd : PolyMesh->faces())
	{
		(*faces2indices)[fhd] = index++;
	}
	index = 0;
	for (Edgeidx ehd : PolyMesh->edges())
	{
		(*edges2indices)[ehd] = index++;
	}

	std::cout << "INFO::LOAD MESH " << std::endl;
	std::cout << "> #polygon " << EDGES
		<< ", #vertices " << PolyMesh->number_of_vertices()
		<< ", #edges " << PolyMesh->number_of_edges()
		<< ", #halfedges " << PolyMesh->number_of_halfedges()
		<< ", #faces " << PolyMesh->number_of_faces() << std::endl;
}

void SurfaceMeshObject::modelTransform(Eigen::Matrix4f const & matrix)
{
	for (auto vid : PolyMesh->vertices())
	{
		auto & pos = PolyMesh->point(vid);
		Eigen::Vector4f p, np;
		p << pos.x(), pos.y(), pos.z(), 1.0f;
		np = matrix * p;
		PolyMesh->point(vid) = Point3f(np.x(), np.y(), np.z());
	}
}

Eigen::VectorXf SurfaceMeshObject::getPositions() const
{
	Eigen::VectorXf positions = Eigen::VectorXf(VERTEX_SIZE * 3);
	for (auto iter = PolyMesh->vertices_begin(); iter != PolyMesh->vertices_end(); ++iter)
	{
		Eigen::Vector3f pos_eigen;
		Veridx vh = *iter;
		copy_v3f(pos_eigen, PolyMesh->point(vh));
		positions.block<3, 1>(vertices2indices->at(vh) * 3, 0) = pos_eigen;
	}
	return positions;
}

void SurfaceMeshObject::setPositions(Eigen::VectorXf const & positions)
{
	for (Veridx viter : PolyMesh->vertices())
	{
		Point3f pos_cgal;
		Eigen::Vector3f pos_eigen = positions.block<3, 1>(vertices2indices->at(viter) * 3, 0);
		copy_v3f(pos_cgal, pos_eigen);
		PolyMesh->point(viter) = pos_cgal;
	}
	/* after changing position
	* update normals for consistence
	*/
	refreshNormals();
}

void SurfaceMeshObject::refreshNormals()
{	
	/* refresh normals */
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals;
	SurfaceMesh3f::Property_map<Veridx, Vec3f> vertexNormals;

	auto fnormals = PolyMesh->property_map<Faceidx, Vec3f>(pname_faceNormals);
	if (fnormals.second) faceNormals = fnormals.first;
	else faceNormals = PolyMesh->add_property_map<Faceidx, Vec3f>(pname_faceNormals).first;

	//CGAL::Polygon_mesh_processing::compute_face_normals(mesh, faceNormals);
	
	auto vnormals = PolyMesh->property_map<Veridx, Vec3f>(pname_vertexNormals);
	if (vnormals.second) vertexNormals = vnormals.first;
	else vertexNormals = PolyMesh->add_property_map<Veridx, Vec3f>(pname_vertexNormals).first;

	//CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vertexNormals);
	
	CGAL::Polygon_mesh_processing::compute_normals(*PolyMesh, vertexNormals, faceNormals,
		CGAL::Polygon_mesh_processing::parameters::vertex_point_map(PolyMesh->points()).geom_traits(Kernelf()));

	/* update to eigen vector normals */
	SurfaceMesh3f::Property_map<Faceidx, Eigen::Vector3f> eigenFaceNormals;
	SurfaceMesh3f::Property_map<Veridx, Eigen::Vector3f> eigenVertexNormals;

	auto eigenfnormals = PolyMesh->property_map<Faceidx, Eigen::Vector3f>(pname_faceEigenNormals);
	if (eigenfnormals.second) eigenFaceNormals = eigenfnormals.first;
	else eigenFaceNormals = PolyMesh->add_property_map<Faceidx, Eigen::Vector3f>(pname_faceEigenNormals).first;

	//CGAL::Polygon_mesh_processing::compute_face_normals(mesh, faceNormals);

	auto eigenvnormals = PolyMesh->property_map<Veridx, Eigen::Vector3f>(pname_vertexEigenNormals);
	if (eigenvnormals.second) eigenVertexNormals = eigenvnormals.first;
	else eigenVertexNormals = PolyMesh->add_property_map<Veridx, Eigen::Vector3f>(pname_vertexEigenNormals).first;

	for (auto v : PolyMesh->vertices())
	{
		copy_v3f(eigenVertexNormals[v], vertexNormals[v]);
	}
	for (auto f : PolyMesh->faces())
	{
		copy_v3f(eigenFaceNormals[f], faceNormals[f]);
	}
}

/* export data for VBO and EBO for drawing */
void SurfaceMeshObject::exportPos3fNorm3fBuffer(
	GLfloat* & vertexBuffer, GLfloat* & vertexNormalBuffer, GLuint & vertexSize,
	GLuint* & elementBuffer, GLuint & elementSize) const
{
	SurfaceMesh3f* mesh = this->PolyMesh;
	SurfaceMesh3f::Property_map<Veridx, Vec3f> vertexNormals = PolyMesh->property_map<Veridx, Vec3f>(pname_vertexNormals).first;
	/* data for VBO */
	vertexBuffer = new GLfloat[mesh->number_of_vertices() * 3];
	vertexNormalBuffer = new GLfloat[mesh->number_of_vertices() * 3];
	/* data for EBO */
	elementBuffer = new GLuint[mesh->number_of_faces() * 3 * (EDGES - 2)];

	std::map<Veridx, GLuint> vhandles2vindices;

	/* export data for VBO */
	GLuint pivot = 0;
	for (Veridx vhandle : mesh->vertices())
	{
		vhandles2vindices[vhandle] = pivot;
		// TODO to be tested
		memcpy_s(vertexBuffer + pivot * 3, 3 * sizeof(GLfloat), &(mesh->point(vhandle)), 3 * sizeof(GLfloat));
		memcpy_s(vertexNormalBuffer + pivot * 3, 3 * sizeof(GLfloat), &(vertexNormals[vhandle]), 3 * sizeof(GLfloat));
		++pivot;
	}
	vertexSize = pivot;
	/* export data for EBO */
	pivot = 0;
	for (Faceidx fhandle : mesh->faces())
	{
		CGAL::Vertex_around_face_circulator<SurfaceMesh3f> cfviter(mesh->halfedge(fhandle), *mesh);
		/* for a face with n edges, element buffer is
		(0, 1, 2),  (0, 2, 3),  (0, 3, 4), ..., (0, n-2, n-1)
		rearrange is
		0, 1, (2, 0, 2), (3, 0, 3), ..., (n-2, 0, n-2), n-1
		*/
		Veridx v0 = *cfviter;
		elementBuffer[pivot++] = vhandles2vindices[*(cfviter++)];
		elementBuffer[pivot++] = vhandles2vindices[*(cfviter++)];
		for (size_t _i = 2; _i <= EDGES - 2; ++_i, ++cfviter)
		{
			elementBuffer[pivot++] = vhandles2vindices[*cfviter];
			elementBuffer[pivot++] = vhandles2vindices[v0];
			elementBuffer[pivot++] = vhandles2vindices[*cfviter];
		}
		elementBuffer[pivot++] = vhandles2vindices[*cfviter];
	}
	elementSize = pivot;
	return;
}

void SurfaceMeshObject::exportFaceNorm3fBuffer(GLfloat *& fBarycenterBuffer, GLfloat *& fNormalBuffer, GLuint & faceSize) const
{
	// TODO
	SurfaceMesh3f* mesh = this->PolyMesh;
	SurfaceMesh3f::Property_map<Faceidx, Vec3f> faceNormals = mesh->property_map<Faceidx, Vec3f>(pname_faceNormals).first;

	fBarycenterBuffer = new GLfloat[mesh->number_of_faces() * 3];
	fNormalBuffer = new GLfloat[mesh->number_of_faces() * 3];

	GLuint pivot = 0;
	for (Faceidx fhd : mesh->faces())
	{
		// face normal
		Vec3f normal = faceNormals[fhd];
		memcpy_s(fNormalBuffer + pivot * 3, 3 * sizeof(GLfloat), &normal, 3 * sizeof(GLfloat));
		// face barycenter
		Vec3f barycenter(0.0f, 0.0f, 0.0f);
		size_t edge_cnt = 0;
		for (Veridx viter : vertices_around_face(mesh->halfedge(fhd), *mesh))
		{
			Point3f v = mesh->point(viter);
			barycenter = barycenter + Vec3f(v.x(), v.y(), v.z());
			++edge_cnt;
		}
		barycenter = barycenter * (1.0f / (edge_cnt > 1 ? edge_cnt : 1));
		memcpy_s(fBarycenterBuffer + pivot * 3, 3 * sizeof(GLfloat), &barycenter, 3 * sizeof(GLfloat));
		++pivot;
	}
	faceSize = pivot;
	return;
}

bool SurfaceMeshObject::useVTexCoord2DAsVPlanarCoord3f()
{
	// TODO add return false case
	SurfaceMesh3f::Property_map<Veridx, Point3f> vprop_handle = 
		PolyMesh->add_property_map<Veridx, Point3f>(pname_vertexPlanarCoords).first;
	SurfaceMesh3f::Property_map<Veridx, Point3f> texCoords =
		PolyMesh->property_map<Veridx, Point3f>(pname_texCoords).first;
	for (Veridx vhd : PolyMesh->vertices())
	{
		vprop_handle[vhd] = texCoords[vhd];
	}
	return true;
}

bool SurfaceMeshObject::getVPlanarCoord3f(SurfaceMesh3f::Property_map<Veridx, Point3f> & vph)
{
	// TODO add return false case
	vph = PolyMesh->property_map<Veridx, Point3f>(pname_vertexPlanarCoords).first;
	return true;
}
