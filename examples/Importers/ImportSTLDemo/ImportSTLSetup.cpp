#include "ImportSTLSetup.h"
#include <vector>
#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "LoadMeshFromSTL.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../../Utils/b3ResourcePath.h"
#include <iostream>



class ImportSTLSetup : public CommonRigidBodyBase
{

	const char* m_fileName;
	btVector3 m_scaling;
	
public:
    ImportSTLSetup(struct GUIHelperInterface* helper, const char* fileName);
    virtual ~ImportSTLSetup();
    
	virtual void initPhysics();
	virtual void resetCamera()
	{
		float dist = 113.5;
		float pitch = -28;
		float yaw = -136;
		float targetPos[3]={0.47,0,-0.64};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}

};


ImportSTLSetup::ImportSTLSetup(struct GUIHelperInterface* helper, const char* fileName)
:CommonRigidBodyBase(helper),
m_scaling(btVector3(0.1,0.1,0.1))
{
	if (fileName)
	{
		m_fileName = fileName;
		m_scaling = btVector3(0.01,0.01,0.01);
	} else
	{
		//m_fileName = "/Users/tommy/bullet3/data/l_finger_tip.stl";
		m_fileName = "/Users/tommy/bullet3/data/dice.STL";
	}
}

ImportSTLSetup::~ImportSTLSetup()
{
    
}



void ImportSTLSetup::initPhysics()
{
	m_guiHelper->setUpAxis(2);
	this->createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);

	
    char relativeFileName[1024];
  if (!b3ResourcePath::findResourcePath(m_fileName, relativeFileName, 1024))
        {
                b3Warning("Cannot find file %s\n", m_fileName);
                return;
        }

	
	btVector3 shift(0,0,0);
//	int index=10;
	
	{
		char tmp[] = "/Users/tommy/bullet3/data/dice.stl";
		//relativeFileName = "/Users/tommy/bullet3/data/dice.stl";
		GLInstanceGraphicsShape* gfxShape = LoadMeshFromSTL(tmp);
		//btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[0];
		//btRigidBody* body = btRigidBody::upcast(obj);

		
		const GLInstanceVertex& v = gfxShape->m_vertices->at(0);
		btConvexHullShape* shape = new btConvexHullShape((const btScalar*)(&(v.xyzw[0])), gfxShape->m_numvertices, sizeof(GLInstanceVertex));
		btVector3 color(1,1,1);
		btVector3 scaling(0.1,0.1,0.1);
		shape->setLocalScaling(scaling);
		

		/*
			if (m_options & OptimizeConvexObj)
			{
				shape->optimizeConvexHull();
			}
			if (m_options & ComputePolyhedralFeatures)
			{
				shape->initializePolyhedralFeatures();    
			}
			m_collisionShapes.push_back(shape);
		*/	

		
			btTransform startTransform;
			startTransform.setIdentity();
			btScalar	mass(1.f); 
			btVector3 position(0,20,0);
			startTransform.setOrigin(position);
			btRigidBody* body = createRigidBody(mass,startTransform,shape);
		

		/*{
			bool useObjForRendering = ((m_options & ObjUseConvexHullForRendering)!=0);
			if (useObjForRendering) {
				int shapeId = m_guiHelper->getRenderInterface()->registerShape(&glmesh->m_vertices->at(0).xyzw[0], 
																				glmesh->m_numvertices, 
																				&glmesh->m_indices->at(0), 
																				glmesh->m_numIndices,
																				B3_GL_TRIANGLES,-1);
				shape->setUserIndex(shapeId);
				int renderInstance = m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId,
																								position,
																								startTransform.getRotation(),
																								color,scaling);
				body->setUserIndex(renderInstance);
			}
		*/
	/*
		btTransform trans;
		trans.setIdentity();
		trans.setRotation(btQuaternion(btVector3(1,0,0),SIMD_HALF_PI));
		
		btVector3 position = trans.getOrigin();
		btQuaternion orn = trans.getRotation();
		
		btVector3 color(0,0,1);
	*/	
		//int shapeId = m_guiHelper->getRenderInterface()->registerShape(&gfxShape->m_vertices->at(0).xyzw[0], gfxShape->m_numvertices, &gfxShape->m_indices->at(0), gfxShape->m_numIndices);
		
		
	
   		m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
		//m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId,position,orn,color,m_scaling);
		//std::cout << "body is in world? :"<< body->isInWorld() << std::endl;
	}
}

class CommonExampleInterface*    ImportSTLCreateFunc(struct CommonExampleOptions& options)
{
	return new ImportSTLSetup(options.m_guiHelper, options.m_fileName);
}
