#define CONSTRAINT_DEBUG_SIZE 0.2f


#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "Simulator.h"
float force = 0;
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#define MASS 0.5
#define ARM 0.2
#define WIDTH 0.05
#define THICK 0.02
#define TORQUE_K 1.0
float error(float range = 1.0)
{
	float rdm = (random() % 1000) / 1000.0;
	return range * (2 * rdm - 1);
}
class Quad
{
	btDynamicsWorld* m_ownerWorld;
	btCompoundShape* compound;
	btCollisionShape* m_shapes[2];
	btRigidBody* m_bodies[1];
	
	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);
		
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);
		
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		
		m_ownerWorld->addRigidBody(body);
		
		return body;
	}
	
public:
	Quad (btDynamicsWorld* ownerWorld, const btVector3& positionOffset) : m_ownerWorld (ownerWorld)
	{
		// Setup the geometry
		m_shapes[0] = new btBoxShape(btVector3(WIDTH, THICK, 2 * ARM));
		m_shapes[1] = new btBoxShape(btVector3(WIDTH, THICK, 2 * ARM));
		btCompoundShape* compound = new btCompoundShape();
		btTransform localtran;
		localtran.setIdentity();
		compound->addChildShape(localtran, m_shapes[0]);
		localtran.getBasis().setEulerZYX(0,M_PI_2,0);
		compound->addChildShape(localtran, m_shapes[1]);
		// Setup all the rigid bodies
		btTransform offset;
		offset.setIdentity();
		offset.setOrigin(positionOffset);
		
		m_bodies[0] = localCreateRigidBody(btScalar(MASS), offset, compound);
		
		// Setup some damping on the m_bodies
		for (int i = 0; i < 1; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(0.1, 0.1);
		}
	}
	
	virtual	~Quad ()
	{
		int i;
		
		// Remove all bodies and shapes
		for ( i = 0; i < 1; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();
			
			delete m_bodies[i]; m_bodies[i] = NULL;
		}
		for ( i = 0; i < 2; ++i)
		{
			delete m_shapes[i]; m_shapes[i] = NULL;
		}
		delete compound; compound = NULL;
	}
	void apply(btVector4 throttle)
	{
		btTransform tran = btTransform(m_bodies[0]->getOrientation());
		m_bodies[0]->activate();
		m_bodies[0]->applyForce(tran*btVector3(0.0, throttle[0] + error(0.3), 0.0), tran*btVector3(0.0, 0.0, ARM));
		m_bodies[0]->applyTorque(tran*btVector3(0.0, TORQUE_K * throttle[0], 0.0));
		m_bodies[0]->applyForce(tran*btVector3(0.0, throttle[1] + error(0.3), 0.0), tran*btVector3(ARM, 0.0, 0.0));
		m_bodies[0]->applyTorque(tran*btVector3(0.0, -TORQUE_K * throttle[1], 0.0));
		m_bodies[0]->applyForce(tran*btVector3(0.0, throttle[2] + error(0.3), 0.0), tran*btVector3(0.0, 0.0, -ARM));
		m_bodies[0]->applyTorque(tran*btVector3(0.0, TORQUE_K * throttle[2], 0.0));
		m_bodies[0]->applyForce(tran*btVector3(0.0, throttle[3] + error(0.3), 0.0), tran*btVector3(-ARM, 0.0, 0.0));
		m_bodies[0]->applyTorque(tran*btVector3(0.0, -TORQUE_K * throttle[3], 0.0));
	}
};


void preTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
	Simulator* env = (Simulator*)world->getWorldUserInfo();
	env->m_quads[0]->apply(btVector4(force ,force,force,force));
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
	btVector3 startOffset(0,0.5,0);
	addQuad(startOffset);
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
			force += 0.5;
			break;
		}
		case 'n':
		{
			force = fmax(0.0, force - 0.5);
			break;
		}
		case 't':
		{
			// 油门
			force = 2.0;
			break;
		}
		case 'b':
		{
			// 断电
			force = 0.0;
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