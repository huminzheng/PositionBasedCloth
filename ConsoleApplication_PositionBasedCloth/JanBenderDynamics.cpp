#include "JanBenderDynamics.h"
#include "Util\BasicOperations.h"
#include "Util\Geometry.h"
#include "ContactDetection.h"
#include "AABBTree\SpatialHashing.h"

#include "PositionBasedDynamics\PositionBasedDynamics.h"

#define USE_STRETCH_CONSTRAINTS
#define USE_SHEAR_CONSTRAINTS
#define USE_BEND_CONSRTAINTS
#define USE_COLLISION_CONSTRAINTS


void JanBenderDynamics::initial(float density)
{
	SurfaceMesh3f * mesh = this->m_clothPiece->getMesh();
	
	/* ----------- initial vertex positions ---------- */
	auto & lastpos = mesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexLastPositions);
	if (!lastpos.second) lastpos = mesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexLastPositions);
	m_lastPositions = lastpos.first;

	auto & curpos = mesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexCurrentPositions);
	if (!curpos.second) curpos = mesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexCurrentPositions);
	m_currentPositions = curpos.first;
	for (Veridx vid : mesh->vertices())
	{
		copy_v3f(m_currentPositions[vid], mesh->point(vid));
	}

	auto & prepos = mesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPredictPositions);
	if (!prepos.second) prepos = mesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPredictPositions);
	m_predictPositions = prepos.first;

	for (Veridx vid : mesh->vertices())
	{
		copy_v3f(m_currentPositions[vid], mesh->point(vid));
		m_lastPositions[vid] = m_currentPositions[vid];
		m_predictPositions[vid] = m_currentPositions[vid];
	}

	/* ----------- initial planar coordinates ---------- */
	auto placor = mesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPlanarCoords);
	if (!placor.second) placor = mesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPlanarCoords);
	m_planarCoordinates = placor.first;
	SurfaceMesh3f::Property_map<Veridx, Point3f> texCoords =
		mesh->property_map<Veridx, Point3f>(SurfaceMeshObject::pname_texCoords).first;
	for (Veridx vid : mesh->vertices())
	{
		copy_v3f(m_planarCoordinates[vid], texCoords[vid]);
		m_planarCoordinates[vid] = m_planarCoordinates[vid] * 45.0f;
	}

	/* ----------- initial normals ---------- */
	m_clothPiece->refreshNormals();
	m_faceNormals = mesh->property_map<Faceidx, Vec3f>(SurfaceMeshObject::pname_faceNormals).first;
	m_vertexNormals = mesh->property_map<Veridx, Vec3f>(SurfaceMeshObject::pname_vertexNormals).first;

	/* ----------- initial vertex velocities ---------- */
	auto vervel = mesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexVelocities);
	if (!vervel.second) vervel = mesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexVelocities);
	m_vertexVelocities = vervel.first;
	for (Veridx vid : mesh->vertices())
	{
		m_vertexVelocities[vid] = Eigen::Vector3f::Zero();
	}

	/* ----------- initial vertex masses ---------- */
	auto vermass = mesh->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexMasses);
	if (!vermass.second) vermass = mesh->add_property_map<Veridx, float>(SurfaceMeshObject::pname_vertexMasses);
	m_vertexInversedMasses = vermass.first;
	
	auto verinvmass = mesh->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses);
	if (!verinvmass.second) verinvmass = mesh->add_property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses);
	m_vertexMasses = verinvmass.first;

	for (Veridx vid : mesh->vertices())
	{
		m_vertexMasses[vid] = density;
		m_vertexInversedMasses[vid] = 1.0f / density;
	}

}

void JanBenderDynamics::addPermanentConstraints()
{	
	auto mesh = m_clothPiece->getMesh();

#if defined(USE_STRETCH_CONSTRAINTS) && defined(USE_SHEAR_CONSTRAINTS)
	// add stretching and shearing constraints
	for (auto eid : mesh->edges())
	{
		Constraint * cons = new DistanceConstraint(
			m_planarCoordinates, m_predictPositions, m_vertexInversedMasses,
			mesh->vertex(eid, 0), mesh->vertex(eid, 1));
		m_permanentConstraints.push_back(cons);
	}
#endif

#ifdef USE_BEND_CONSRTAINTS
	// add isometric bending constraints
	for (auto eid : mesh->edges())
	{
		Veridx v1 = mesh->vertex(mesh->next_around_target(mesh->halfedge(eid, 0)), 1);
		Veridx v2 = mesh->vertex(eid, 0);
		Veridx v3 = mesh->vertex(eid, 1);
		Veridx v4 = mesh->vertex(mesh->next_around_target(mesh->halfedge(eid, 1)), 1);

		Constraint * cons = new IsometricBendingConstraint(
			m_predictPositions, m_vertexInversedMasses,
			v1, v2, v3, v4);
		m_permanentConstraints.push_back(cons);
	}
#endif
}

void JanBenderDynamics::userSet()
{
	int i = 0;
	for (auto vid : m_clothPiece->getMesh()->vertices())
	{
		if (i == 0 || i == 92)
			//if (i == 1239 || i == 1362 || i == 1035)
			m_vertexInversedMasses[vid] = 0.0f;
		i += 1;
	}
}

void JanBenderDynamics::stepforward(float timeStep)
{
	std::cout << " ------------ new step ------------ " << std::endl;
	m_temporaryConstraints.clear();

	freeForward(timeStep);
	
#ifdef USE_COLLISION_CONSTRAINTS
	genCollConstraints();
#endif
	for (unsigned int _i = 0; _i < m_iterCount; ++_i)
	{
		projectConstraints(m_iterCount);
	}
	updateStates(timeStep);
	velocityUpdate();
	writeBack();
}

void JanBenderDynamics::freeForward(float timeStep)
{
	auto mesh = m_clothPiece->getMesh();
	for (auto vid : mesh->vertices())
	{
		auto v = m_vertexVelocities[vid];
		v += timeStep * m_vertexInversedMasses[vid] * f_ext;
		m_vertexVelocities[vid] = v;
		m_predictPositions[vid] = m_currentPositions[vid] + timeStep * v;
	}
}

void JanBenderDynamics::genCollConstraints()
{
	auto clothMesh = m_clothPiece->getMesh();
	float thickness = 0.3f;

	auto cor = PointEigen3f(500.0f, 500.0f, 500.0f);
	SpatialHashing<Face3fContinuesRef, PointEigen3f> spatial(-1 * cor, cor, cor / 100.0f);
	for (auto fid : clothMesh->faces())
	{
		spatial.insert(Face3fContinuesRef(*clothMesh, fid, m_currentPositions, m_predictPositions));
	}
	for (auto vid : clothMesh->vertices())
	{
		Vertex3fContinuesRef verref(*clothMesh, vid, m_currentPositions, m_predictPositions);
		auto candidates = spatial.candidate(verref);
		
		AABBTree<Face3fContinuesRef, PointEigen3f> faceTree(candidates);
		auto contacts = faceTree.contactDetection<Vertex3fContinuesRef, ContinuousCollideResult>(
			verref, 0.1f);

		for (auto contact : *contacts)
		{
			Face3fContinuesRef const & faceref = faceTree.at(contact.first).second;
			auto const & conInfo = contact.second;
			Veridx fvid[3];
			int _i = 0;
			for (auto fv : clothMesh->vertices_around_face(clothMesh->halfedge(faceref.faceidx)))
			{
				fvid[_i] = fv;
				_i++;
			}
			if (fvid[0] == vid || fvid[1] == vid || fvid[2] == vid)
				continue;
			Constraint * cons = new VertexFaceDirectedDistanceConstraint(
				m_predictPositions, m_vertexInversedMasses,
				m_predictPositions, m_vertexInversedMasses,
				vid, fvid[0], fvid[1], fvid[2],
				true, true, conInfo.state == ContinuousCollideResult::CLOSE ? true : false, 
				thickness, conInfo.time);
			m_temporaryConstraints.push_back(cons);
		}
	}

	//AABBTree<Face3fRef, PointEigen3f> faceTree(
	//	clothMesh->faces_begin(), clothMesh->faces_end(), 
	//	FaceIter2Triangle3fAABBoxPair(*clothMesh, m_predictPositions),
	//	clothMesh->number_of_faces());

	//float thickness = 0.3f;

	//for (auto vid : clothMesh->vertices())
	//{
	//	auto contacts = faceTree.contactDetection<PointEigen3f, Eigen::Vector3f>(
	//		m_predictPositions[vid], 0.1f);

	//	for (auto contact : *contacts)
	//	{
	//		Face3fRef const & faceref = faceTree.at(contact.first).second;
	//		Veridx fvid[3];
	//		int _i = 0;
	//		for (auto fv : clothMesh->vertices_around_face(clothMesh->halfedge(faceref.faceidx)))
	//		{
	//			fvid[_i] = fv;
	//			_i++;
	//		}
	//		if (fvid[0] == vid || fvid[1] == vid || fvid[2] == vid)
	//			continue;
	//		Constraint * cons = new VertexFaceCollisionConstraint(
	//			m_predictPositions, m_vertexInversedMasses,
	//			m_predictPositions, m_vertexInversedMasses,
	//			vid, fvid[0], fvid[1], fvid[2],
	//			true, true, thickness);
	//		m_temporaryConstraints.push_back(cons);
	//	}
	//}
}

void JanBenderDynamics::projectConstraints(int iterCount)
{
	for (auto cons : m_permanentConstraints)
	{
		cons->updateConstraint();
		cons->solvePositionConstraint();
	}
	for (int _i = 0; _i < 5; ++_i)
	{
		for (auto cons : m_temporaryConstraints)
		{
			cons->updateConstraint();
			cons->solvePositionConstraint();
		}
	}
}

void JanBenderDynamics::updateStates(float timeStep)
{
	float invTimeStep = 1.0f / timeStep;
	for (auto vid : m_clothPiece->getMesh()->vertices())
	{
		m_vertexVelocities[vid] = 0.98f * invTimeStep * (m_currentPositions[vid] - m_lastPositions[vid]);
		m_lastPositions[vid] = m_currentPositions[vid];
		m_currentPositions[vid] = m_predictPositions[vid];
	}
}

void JanBenderDynamics::velocityUpdate()
{

}

void JanBenderDynamics::writeBack()
{
	auto mesh = m_clothPiece->getMesh();
	for (auto vid : mesh->vertices())
	{
		copy_v3f(mesh->point(vid), m_currentPositions[vid]);
	}
	m_clothPiece->refreshNormals();
}

void JanBenderDynamics::exportCollisionVertices(GLfloat *& buffer, GLuint & size)
{
	size = m_temporaryConstraints.size();
	buffer = new GLfloat[size * 3];
	memset(buffer, 0, sizeof(GLfloat) * 3 * size);
	int pivot = 0;
	for (auto cons : m_temporaryConstraints)
	{
		if (cons->getTypeId() == 21)
		{
			Veridx vid = ((VertexFaceCollisionConstraint *)cons)->m_v;
			buffer[pivot++] = m_currentPositions[vid].x();
			buffer[pivot++] = m_currentPositions[vid].y();
			buffer[pivot++] = m_currentPositions[vid].z();
		}
	}
}
