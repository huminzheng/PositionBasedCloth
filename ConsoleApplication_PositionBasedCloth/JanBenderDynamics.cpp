#include "JanBenderDynamics.h"
#include "Util\BasicOperations.h"
#include "Util\Geometry.h"
#include "Util\Profile.h"
#include "AABBTree\SpatialHashing.h"

#include "PositionBasedDynamics\PositionBasedDynamics.h"

//#define USE_STRETCH_CONSTRAINTS
//#define USE_SHEAR_CONSTRAINTS
#define USE_FEM_TRIANGLE_CONSTRAINTS
#define USE_BEND_CONSRTAINTS

#define USE_VELOCITY_CONSTRAINTS
//#define USE_FIXED_POINTS

#define USE_COLLISION_CONSTRAINTS

//#define USE_STATIC_COLLISION
#define USE_NORMALIZED_STATIC_COLLISION
//#define USE_CONTINUOUS_COLLISION

//#define USE_SELF_COLLISION
#define USE_RIGIDBODY_COLLISION

namespace
{
	static const std::string profileFile = "E:/Microsoft Visual Studio 2015/Workspace/ConsoleApplication_PositionBasedCloth/log/LogFile.txt";

	void ProfileNewFrame()
	{
		std::ofstream outfile;
		outfile.open(profileFile, std::ios_base::app);
		outfile << std::endl;
	}

	void ProfileResult(const ScopedProfiler::TimeCount start, const ScopedProfiler::TimeCount end)
	{
		std::ofstream outfile;
		outfile.open(profileFile, std::ios_base::app);
		outfile << std::to_string(double(end - start) / 1000000.0) << ", ";
	}
}

#define PROFILE_SCOPE							\
	ScopedProfiler profiler(ProfileResult);		\
	(profiler);

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
		//copy_v3f(m_planarCoordinates[vid], texCoords[vid]);
		//m_planarCoordinates[vid] = m_planarCoordinates[vid] * 10.0f;
		m_planarCoordinates[vid] = m_currentPositions[vid]/* * 0.7f*/;
	}

	/* ----------- initial normals ---------- */
	m_clothPiece->refreshNormals();
	m_faceEigenNormals = mesh->property_map<Faceidx, Eigen::Vector3f>(SurfaceMeshObject::pname_faceEigenNormals).first;
	m_vertexEigenNormals = mesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexEigenNormals).first;

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
	surfaceMeshObject->refreshNormals();

	auto & rbcurpos = rbmesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexCurrentPositions);
	if (!rbcurpos.second) rbcurpos = rbmesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexCurrentPositions);
	auto const & cpmap = rbcurpos.first;

	auto & rbprepos = rbmesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPredictPositions);
	if (!rbprepos.second) rbprepos = rbmesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexPredictPositions);
	auto const & ppmap = rbprepos.first;

	auto & rbvel = rbmesh->property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexVelocities);
	if (!rbvel.second) rbvel = rbmesh->add_property_map<Veridx, Eigen::Vector3f>(SurfaceMeshObject::pname_vertexVelocities);
	auto const & vmap = rbvel.first;

	auto & rbfnormal = rbmesh->property_map<Faceidx, Eigen::Vector3f>(SurfaceMeshObject::pname_faceEigenNormals);
	if (!rbfnormal.second) rbfnormal = rbmesh->add_property_map<Faceidx, Eigen::Vector3f>(SurfaceMeshObject::pname_faceEigenNormals);
	auto const & fnmap = rbfnormal.first;

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
		vmap[vid] = Eigen::Vector3f::Zero();
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
		Veridx v1 = mesh->vertex(eid, 0);
		Veridx v2 = mesh->vertex(eid, 1);
		if (!mesh->has_valid_index(v1) || !mesh->has_valid_index(v2))
			continue;

		Constraint * cons = new DistanceConstraint(
			m_planarCoordinates, m_predictPositions, m_vertexInversedMasses,
			v1, v2);
		m_permanentConstraints.push_back(cons);
	}
#endif

#if defined(USE_FEM_TRIANGLE_CONSTRAINTS)
	// add fem based triangle constraints
	for (auto fid : mesh->faces())
	{
		Veridx v[3];
		int _i = 0;
		for (auto vid : mesh->vertices_around_face(mesh->halfedge(fid)))
		{
			v[_i] = vid;
			++_i;
		}
		if (!mesh->has_valid_index(v[0]) || !mesh->has_valid_index(v[1]) || !mesh->has_valid_index(v[2]))
			continue;

		Constraint * cons = new FEMTriangleConstraint(
			m_predictPositions, m_vertexInversedMasses,
			v[0], v[1], v[2], 
			m_params->YoungModulo_xx, m_params->YoungModulo_yy, m_params->YoungModulo_xy,
			m_params->PoissonRation_xy, m_params->PoissonRation_yx);
		m_permanentConstraints.push_back(cons);
	}
#endif

#ifdef USE_BEND_CONSRTAINTS
	// add isometric bending constraints
	for (auto eid : mesh->edges())
	{
		if (!mesh->has_valid_index(mesh->face(mesh->halfedge(eid, 0))) ||
			!mesh->has_valid_index(mesh->face(mesh->halfedge(eid, 1))))
			continue;

		Veridx v1 = mesh->vertex(mesh->next_around_target(mesh->halfedge(eid, 0)), 0);
		Veridx v2 = mesh->vertex(eid, 0);
		Veridx v3 = mesh->vertex(eid, 1);
		Veridx v4 = mesh->vertex(mesh->next_around_target(mesh->halfedge(eid, 1)), 1);

		if (!mesh->has_valid_index(v1) || !mesh->has_valid_index(v2) ||
			!mesh->has_valid_index(v3) || !mesh->has_valid_index(v4))
			continue;

		Constraint * cons = new IsometricBendingConstraint(
			m_predictPositions, m_vertexInversedMasses,
			v1, v4, v2, v3, m_params->IsometricBendingStiff);
		m_permanentConstraints.push_back(cons);
	}
#endif
}

void JanBenderDynamics::userSet()
{
	int i = 0;
#ifdef USE_FIXED_POINTS
	for (auto vid : m_clothPiece->getMesh()->vertices())
	{
		if (i == 0 || i == 129)
			//if (i == 1239 || i == 1362 || i == 1035)
			m_vertexInversedMasses[vid] = 0.0f;
		i += 1;
	}
#endif
	static int count = 0;
	++count;
	
	for (auto * rgptr : m_rigidBodies)
	{
		Eigen::Matrix3f const & rot33 = rotate_matrix(Eigen::Vector3f(0.0f, 1.0f, 0.0f), 0.01f);
		Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
		rot.block<3, 3>(0, 0) = rot33;
		Eigen::Matrix4f trans;
		trans << 1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.07f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f;
		//rgptr->affine(trans * rot);

	}
}

void JanBenderDynamics::stepforward(float timeStep)
{
	std::cout << " ------------ new step ------------ " << std::endl;

	ProfileNewFrame();

	m_temporaryConstraints.clear();

	//for (auto vid : m_clothPiece->getMesh()->vertices())
	//{
	//	auto offset = m_predictPositions[vid] - m_currentPositions[vid];
	//	if (offset.squaredNorm() > 1e-20)
	//		std::cout << "offset invalid at " << vid << std::endl;
	//}

	{
		PROFILE_SCOPE;
		freeForward(timeStep);
	}

	//for (auto vid : m_clothPiece->getMesh()->vertices())
	//{
	//	Eigen::Vector3f offset = m_predictPositions[vid] - m_currentPositions[vid];
	//	if ((offset - Eigen::Vector3f(0.0f, -0.01f, 0.0f)).squaredNorm() > 1e-5)
	//		std::cout << "offset invalid at " << vid << std::endl;
	//}

	{
		PROFILE_SCOPE;

#ifdef USE_COLLISION_CONSTRAINTS
		genCollConstraints();
#endif
	}

	//for (unsigned int _i = 0; _i < m_params->IterCount; ++_i)
	{
		projectConstraints(m_params->IterCount);
	}

	{
		PROFILE_SCOPE;
		updateStates(timeStep);
	}

#ifdef USE_VELOCITY_CONSTRAINTS
	velocityUpdate();
#endif
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

		//Eigen::Vector3f vsq = (v - timeStep * f_ext);
		//if (vsq.squaredNorm() > 1e-10)
		//	std::cout << "velocity invalid " << vid << " with v = " << vsq.squaredNorm() << std::endl;
	}
}


void JanBenderDynamics::genCollConstraints()
{
	auto clothMesh = m_clothPiece->getMesh();
	float rigidbodyThickness = m_params->ClothThickness;
	float clothThickness = m_params->ClothThickness;
	auto const cor = PointEigen3f(500.0f, 500.0f, 500.0f);

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
				verref, clothThickness);

			for (auto contact : *contacts)
			{
				Face3fContinuesRef const & faceref = faceTree.at(contact.first).second;
				auto const & conInfo = contact.second;
				Veridx fvid[3];
				int _i = 0;
				for (auto fv : faceref.mesh.vertices_around_face(faceref.mesh.halfedge(faceref.faceidx)))
				{
					fvid[_i] = fv;
					_i++;
				}
				if (fvid[0] == vid || fvid[1] == vid || fvid[2] == vid)
					continue;
				Constraint * cons = new VertexFaceSidedDistanceConstraint(
					m_predictPositions, verref.invMass,
					faceref.posMap1, faceref.invMass,
					vid, fvid[0], fvid[1], fvid[2],
					true, true, conInfo.state == ContinuousCollideResult::CLOSE ? true : false,
					clothThickness, conInfo.time);
				m_temporaryConstraints.push_back(cons);
			}
		}
	}
#endif

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
			auto const & pmap = ptr->getMesh()->property_map<Veridx, PointEigen3f>(SurfaceMeshObject::pname_vertexCurrentPositions).first;
			auto const & vmap = ptr->getMesh()->property_map<Veridx, PointEigen3f>(SurfaceMeshObject::pname_vertexVelocities).first;
			auto const & immap = ptr->getMesh()->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses).first;
			for (auto fid : ptr->getMesh()->faces())
			{
				spatial.insert(Face3fRef(*ptr->getMesh(), fid, pmap, vmap, immap));
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
			if (candidates.empty())
				continue;
			//std::cout << "candidates size " << candidates.size() << std::endl;
			AABBTree<Face3fRef, PointEigen3f> faceTree(candidates);
			//std::cout << "facetree size " << faceTree.size() << std::endl;
			//std::cout << faceTree.at(0).second.posMap[(Veridx) 0] << std::endl;
			auto contacts = faceTree.contactDetection<Vertex3fRef, Eigen::Vector3f>(
				verref, clothThickness);

			for (auto contact : *contacts)
			{
				Face3fRef const & faceref = faceTree.at(contact.first).second;
				Veridx fvid[3];
				int _i = 0;
				for (auto fv : faceref.mesh.vertices_around_face(faceref.mesh.halfedge(faceref.faceidx)))
				{
					fvid[_i] = fv;
					_i++;
				}
				if (fvid[0] == vid || fvid[1] == vid || fvid[2] == vid)
					continue;
				Constraint * cons = new VertexFaceDistanceConstraint(
					m_predictPositions, m_vertexVelocities, m_vertexInversedMasses,
					faceref.posMap, faceref.velMap, faceref.invMass,
					vid, fvid[0], fvid[1], fvid[2],
					true, true, clothThickness);
				m_temporaryConstraints.push_back(cons);
			}
		}


	}
#endif

	/* ---------- check static collision ---------- */
#ifdef USE_NORMALIZED_STATIC_COLLISION
	{
		SpatialHashing<FaceNormalized3fRef, PointEigen3f> spatial(-1 * cor, cor, cor / 100.0f);

#ifdef USE_RIGIDBODY_COLLISION
		for (auto * ptr : m_rigidBodies)
		{
			auto const & pmap = ptr->getMesh()->property_map<Veridx, PointEigen3f>(SurfaceMeshObject::pname_vertexCurrentPositions).first;
			auto const & vmap = ptr->getMesh()->property_map<Veridx, PointEigen3f>(SurfaceMeshObject::pname_vertexVelocities).first;
			auto const & nmap = ptr->getMesh()->property_map<Faceidx, PointEigen3f>(SurfaceMeshObject::pname_faceEigenNormals).first;
			auto const & immap = ptr->getMesh()->property_map<Veridx, float>(SurfaceMeshObject::pname_vertexInversedMasses).first;
			for (auto fid : ptr->getMesh()->faces())
			{
				FaceNormalized3fRef temp(*ptr->getMesh(), fid, pmap, vmap, nmap, immap);
				spatial.insert(std::move(temp));
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
			if (candidates.empty())
				continue;
			//std::cout << "candidates size " << candidates.size() << std::endl;
			AABBTree<FaceNormalized3fRef, PointEigen3f> faceTree(candidates);
			//std::cout << "facetree size " << faceTree.size() << std::endl;
			//std::cout << faceTree.at(0).second.posMap[(Veridx) 0] << std::endl;
			auto contacts = faceTree.contactDetection<Vertex3fRef, Eigen::Vector3f>(
				verref, clothThickness);

			for (auto contact : *contacts)
			{
				FaceNormalized3fRef const & faceref = faceTree.at(contact.first).second;
				Veridx fvid[3];
				int _i = 0;
				for (auto fv : faceref.mesh.vertices_around_face(faceref.mesh.halfedge(faceref.faceidx)))
				{
					fvid[_i] = fv;
					_i++;
				}
				if (fvid[0] == vid || fvid[1] == vid || fvid[2] == vid)
					continue;
				Constraint * cons = new VertexFaceDirectedDistanceConstraint(
					m_predictPositions, m_vertexVelocities, m_vertexInversedMasses,
					faceref.posMap, faceref.velMap, faceref.normalMap, faceref.invMass,
					vid, faceref.faceidx, fvid[0], fvid[1], fvid[2],
					clothThickness);
				m_temporaryConstraints.push_back(cons);
			}
		}


	}
#endif

}

void JanBenderDynamics::projectConstraints(int iterCount)
{
	//if (m_temporaryConstraints.empty())
	{
		PROFILE_SCOPE;

		for (int iterNo = 0; iterNo < iterCount; ++iterNo)
		{
			for (auto cons : m_permanentConstraints)
			{
				//cons->updateConstraint();
				//cons->solvePositionConstraint();
				//continue;
				switch (cons->getTypeId())
				{
				case ConstraintType::DistanceConstraint_Type:
					cons->updateConstraint();
					cons->solvePositionConstraint(m_params->ProjectDistanceStiff);
					//cons->solvePositionConstraint(std::pow(m_params->ProjectDistanceStiff, 1.0f / iterCount));
					break;
				case ConstraintType::IsometricBendingConstraint_Type:
					cons->updateConstraint();
					cons->solvePositionConstraint(m_params->ProjectIsometricBendingStiff);
					//cons->solvePositionConstraint(std::pow(m_params->ProjectIsometricBendingStiff, 1.0f / iterCount));
					break;
				case ConstraintType::FEMTriangleConstraint_Type:
					//std::cout << "FEMTriangleConstraint_Type" << std::endl;
					cons->updateConstraint();
					cons->solvePositionConstraint(m_params->ProjectFEMTriangleStiff);
					//cons->solvePositionConstraint(std::pow(m_params->ProjectFEMTriangleStiff, 1.0f / iterCount));
					break;
				default:
					break;
				}
			}
		}
	}

	{
		PROFILE_SCOPE;

		for (int iterNo = 0; iterNo < iterCount; ++iterNo)
		{
			for (auto cons : m_temporaryConstraints)
			{
				cons->updateConstraint();
				cons->solvePositionConstraint();
			}
		}
	}
}

void JanBenderDynamics::updateStates(float timeStep)
{
	float invTimeStep = 1.0f / timeStep;
	//if (m_temporaryConstraints.empty())
	for (auto vid : m_clothPiece->getMesh()->vertices())
	{
		m_vertexVelocities[vid] = /*0.98f * */invTimeStep * (m_predictPositions[vid] - m_currentPositions[vid]);
		m_lastPositions[vid] = m_currentPositions[vid];
		m_currentPositions[vid] = m_predictPositions[vid];
	}

#define FIXED_ANCHOR_POINT

#ifdef FIXED_ANCHOR_POINT
	{
		auto iter = m_clothPiece->getMesh()->vertices_begin();
		Veridx vid = *iter++;
		m_vertexVelocities[vid] = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		m_predictPositions[vid] = m_currentPositions[vid];
		m_currentPositions[vid] = m_lastPositions[vid];

		for (size_t _i = 0; _i < 58; ++_i, ++iter);
		vid = *iter++;
		m_vertexVelocities[vid] = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
		m_predictPositions[vid] = m_currentPositions[vid];
		m_currentPositions[vid] = m_lastPositions[vid];
	}
#endif
}

void JanBenderDynamics::velocityUpdate()
{
	for (auto cons : m_permanentConstraints)
	{
		cons->solveVelocityConstraint();
	}
	for (auto cons : m_temporaryConstraints)
	{
		cons->solveVelocityConstraint();
	}
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
		if (cons->getTypeId() == ConstraintType::VertexFaceDistanceConstraint_Type)
		{
			Veridx vid = ((VertexFaceDistanceConstraint *)cons)->m_v;
			buffer[size * 3] = m_currentPositions[vid].x();
			buffer[size * 3 + 1] = m_currentPositions[vid].y();
			buffer[size * 3 + 2] = m_currentPositions[vid].z();
			type[size] = 21u;
			size += 1;
		}
		else if (cons->getTypeId() == ConstraintType::VertexFaceSidedDistanceConstraint_Type)
		{
			Veridx vid = ((VertexFaceSidedDistanceConstraint *)cons)->m_v;
			buffer[size * 3] = m_currentPositions[vid].x();
			buffer[size * 3 + 1] = m_currentPositions[vid].y();
			buffer[size * 3 + 2] = m_currentPositions[vid].z();
			type[size] = 22u;
			size += 1;
		}
		else if (cons->getTypeId() == ConstraintType::VertexFaceDirectedDistanceConstraint_Type)
		{
			Veridx vid = ((VertexFaceDirectedDistanceConstraint *)cons)->m_v;
			buffer[size * 3] = m_currentPositions[vid].x();
			buffer[size * 3 + 1] = m_currentPositions[vid].y();
			buffer[size * 3 + 2] = m_currentPositions[vid].z();
			type[size] = 23u;
			size += 1;
		}
	}
}
