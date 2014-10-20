#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "Simulator.h"
#include "Quad.h"
Quad *theQuad;
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif






void preTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	Simulator* env = (Simulator*)world->getWorldUserInfo();
	env->m_quads[0]->apply();
}

void Simulator::initPhysics()
{
	// Setup the basic world
	
	setTexturing(true);
	setShadows(true);
	
	setCameraDistance(btScalar(5.));
	
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	
	m_solver = new btSequentialImpulseConstraintSolver;
	
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;
	m_dynamicsWorld->setInternalTickCallback(preTickCallback,this,true);
	
	
	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));
		
#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT
	
	}
	btVector3 startOffset(0,2.5,0);
	addQuad(startOffset);
	theQuad = m_quads[0];
	clientResetScene();
}

void Simulator::addQuad(const btVector3& startOffset)
{
	Quad* quad = new Quad (m_dynamicsWorld, startOffset);
	m_quads.push_back(quad);
}

void Simulator::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;
	
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
		
		
	}
	
	renderme();
	theQuad->drawForce();
	
	glFlush();
	
	glutSwapBuffers();
}

void Simulator::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	renderme();
	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
	
	glFlush();
	glutSwapBuffers();
}

void Simulator::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 'p':
		{
			theQuad->throttle += 0.2;
			break;
		}
		case 'n':
		{
			theQuad->throttle = fmax(0.0, theQuad->throttle - 0.2);
			break;
		}
		case 't':
		{
			// 油门
			theQuad->throttle = 2.0;
			break;
		}
		case 'y':
		{
			// 油门
			theQuad->throttle = 1.25;
			break;
		}

		case 'b':
		{
			// 断电
			theQuad->throttle = 0.0;
			break;
		}
		case 'c':
		{
			theQuad->switchAutoPilot();
			break;
		}
		default:
			DemoApplication::keyboardCallback(key, x, y);
			break;
	}
	
	
}



void Simulator::exitPhysics()
{
	
	int i;
	
	for (i=0;i<m_quads.size();i++)
	{
		Quad* quad = m_quads[i];
		delete quad;
	}
	
	//cleanup in the reverse order of creation/initialization
	
	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
	
	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	
	//delete dynamics world
	delete m_dynamicsWorld;
	
	//delete solver
	delete m_solver;
	
	//delete broadphase
	delete m_broadphase;
	
	//delete dispatcher
	delete m_dispatcher;
	
	delete m_collisionConfiguration;
	
	
}