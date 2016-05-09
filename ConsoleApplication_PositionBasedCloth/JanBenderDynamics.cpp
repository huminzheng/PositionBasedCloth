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
//#define USE_STATIC_COLLISION
#define USE_CONTINUOUS_COLLISION
//#define USE_SELF_COLLISION
#define USE_RIGIDBODY_COLLISION


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

void JanBenderDynamics::addRigidBody(SurfaceMeshObject * const surfaceMeshObject)
{
	/* ----------- initial rigid bodies' properties ---------- */
	auto const & rbmesh = surfaceMeshObject->getMesh();

	auto & rbcurpos = rbmesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexCurrentPositions);
	if (!rbcurpos.second) rbcurpos = rbmesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexCurrentPositions);
	auto const & cpmap = rbcurpos.first;

	auto & rbprepos = rbmesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPredictPositions);
	if (!rbprepos.second) rbprepos = rbmesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPredictPositions);
	auto const & ppmap = rbprepos.first;

	auto rbvermass = rbmesh->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexMasses);
	if (!rbvermass.second) rbvermass = rbmesh->add_property_map<Veridx, float>(SurfaceMeshObject::pname_vertexMasses);
	auto const & mmap = rbvermass.first;

	auto rbverinvmass = rbmesh->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses);
	if (!rbverinvmass.second) rbverinvmass = rbmesh->add_property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses);
	auto const & immap = rbverinvmass.first;

	for (Veridx vid : rbmesh->vertices())
	{
		copy_v3f(cpmap[vid], rbmesh->point(vid));
		ppmap[vid] = cpmap[vid];
		mmap[vid] = POSITIVE_MAX_FLOAT;
		immap[vid] = 0.0f;
	}

	m_rigidBodies.push_back(surfaceMeshObject);
}

void JanBenderDynamics::addPermanentConstraints()
{	
	auto const & mesh = m_clothPiece->getMesh();

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
	auto const & mesh = m_clothPiece->getMesh();
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
	float thickness = 0.1f;
	auto const cor = PointEigen3f(500.0f, 500.0f, 500.0f);

	/* ---------- check static collision ---------- */
#ifdef USE_STATIC_COLLISION
	{
		SpatialHashing<Face3fRef, PointEigen3f> spatial(-1 * cor, cor, cor / 100.0f);
#ifdef USE_SELF_COLLISION
		for (auto fid : clothMesh->faces())
		{
			spatial.insert(Face3fRef(*clothMesh, fid, m_predictPositions, m_vertexInversedMasses));
		}
#endif
#ifdef USE_RIGIDBODY_COLLISION
		for (auto * ptr : m_rigidBodies)
		{
			auto const & pmap = ptr->getMesh()->property_map<Veridx, PointEigen3f>(SurfaceMeshObject::pname_vertexCurrentPositions);
			auto const & immap = ptr->getMesh()->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses);
			for (auto fid : ptr->getMesh()->faces())
			{
				spatial.insert(Face3fRef(*ptr->getMesh(), fid, pmap.first, immap.first));
			}
		}
#endif
		//for (int _i = 0; _i < 10; ++_i)
		//{
		//	std::cout << "test map" << std::endl;
		//	Vertex3fRef verref(*clothMesh, (Veridx) 0, m_predictPositions, m_vertexInversedMasses);
		//	std::cout << verref.point((Veridx)0) << std::endl;
		//}

		// BUG so strange bugs when with this simplified loop
		//for (auto vid : clothMesh->vertices())
		for (auto it = clothMesh->vertices_begin(); it != clothMesh->vertices_end(); ++it)
		{
			Veridx vid = *it;
			Vertex3fRef verref(*clothMesh, vid, m_predictPositions, m_vertexInversedMasses);
			auto candidates = spatial.candidate(verref);
			//if (candidates.size() > 0)
			//	std::cout << "candidates" << std::endl;

			AABBTree<Face3fRef, PointEigen3f> faceTree(candidates);
			auto contacts = faceTree.contactDetection<Vertex3fRef, Eigen::Vector3f>(
				verref, 0.1f);

			for (auto contact : *contacts)
			{
				Face3fRef const & faceref = faceTree.at(contact.first).second;
				Veridx fvid[3];
				int _i = 0;
				for (auto fv : clothMesh->vertices_around_face(clothMesh->halfedge(faceref.faceidx)))
				{
					fvid[_i] = fv;
					_i++;
				}
				if (fvid[0] == vid || fvid[1] == vid || fvid[2] == vid)
					continue;
				Constraint * cons = new VertexFaceCollisionConstraint(
					m_predictPositions, m_vertexInversedMasses,
					m_predictPositions, m_vertexInversedMasses,
					vid, fvid[0], fvid[1], fvid[2],
					true, true, thickness);
				m_temporaryConstraints.push_back(cons);
			}
		}


	}
#endif

#ifdef USE_CONTINUOUS_COLLISION
	/* ---------- check continuous collision ---------- */
	{
		SpatialHashing<Face3fContinuesRef, PointEigen3f> spatial(-1 * cor, cor, cor / 100.0f);
#ifdef USE_SELF_COLLISION
		for (auto fid : clothMesh->faces())
		{
			spatial.insert(Face3fContinuesRef(*clothMesh, fid, m_currentPositions, m_predictPositions, m_vertexInversedMasses));
		}
#endif
#ifdef USE_RIGIDBODY_COLLISION
		for (auto * ptr : m_rigidBodies)
		{
			auto const & cpmap = ptr->getMesh()->property_map<Veridx, PointEigen3f>(SurfaceMeshObject::pname_vertexCurrentPositions).first;
			auto const & ppmap = ptr->getMesh()->property_map<Veridx, PointEigen3f>(SurfaceMeshObject::pname_vertexPredictPositions).first;
			auto const & immap = ptr->getMesh()->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses).first;
			for (auto fid : ptr->getMesh()->faces())
			{
				spatial.insert(Face3fContinuesRef(*ptr->getMesh(), fid, cpmap, ppmap, immap));
			}
		}
#endif
		// BUG so strange bugs when with this simplified loop
		//for (Veridx vid : clothMesh->vertices())
		for (auto it = clothMesh->vertices_begin(); it != clothMesh->vertices_end(); ++it)
		{
			Veridx vid = *it;
			Vertex3fContinuesRef verref(*clothMesh, vid, m_currentPositions, m_predictPositions, m_vertexInversedMasses);
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
					m_predictPositions, verref.invMass,
					m_predictPositions, faceref.invMass,
					vid, fvid[0], fvid[1], fvid[2],
					true, true, conInfo.state == ContinuousCollideResult::CLOSE ? true : false,
					thickness, conInfo.time);
				m_temporaryConstraints.push_back(cons);
			}
		}
	}
#endif

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
	auto const & mesh = m_clothPiece->getMesh();
	for (auto vid : mesh->vertices())
	{
		copy_v3f(mesh->point(vid), m_currentPositions[vid]);
	}
	m_clothPiece->refreshNormals();
}

void JanBenderDynamics::exportCollisionVertices(GLfloat *& buffer, GLuint *& type, GLuint & size)
{
	int capacity = m_temporaryConstraints.size();
	buffer = new GLfloat[capacity* 3];
	type = new GLuint[capacity* 1];
	memset(buffer, 0, sizeof(GLfloat) * 3 * capacity);
	memset(type, 0, sizeof(GLuint) * 1 * capacity);
	size = 0;
	for (auto cons : m_temporaryConstraints)
	{
		if (cons->getTypeId() == 21)
		{
			Veridx vid = ((VertexFaceCollisionConstraint *)cons)->m_v;
			buffer[size * 3] = m_currentPositions[vid].x();
			buffer[size * 3 + 1] = m_currentPositions[vid].y();
			buffer[size * 3 + 2] = m_currentPositions[vid].z();
			type[size] = 21u;
			size += 1;
		}
		else if (cons->getTypeId() == 22)
		{
			Veridx vid = ((VertexFaceDirectedDistanceConstraint *)cons)->m_v;
			buffer[size * 3] = m_currentPositions[vid].x();
			buffer[size * 3 + 1] = m_currentPositions[vid].y();
			buffer[size * 3 + 2] = m_currentPositions[vid].z();
			type[size] = 22u;
			size += 1;
		}
	}
}
