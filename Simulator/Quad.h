#ifndef QUAD_H
#define QUAD_H

#define MASS 0.5
#define ARM 0.2
#define WIDTH 0.05
#define THICK 0.02
#define TORQUE_K 0.5
#define MAX_THROTTLE 3.0
#define MIN_THROTTLE 0.0
#define MAX_THROTTLE_STEP 0.1
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "GlutStuff.h"
float error(float range);
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
	btVector3 rotor[4] = {
		btVector3(	 0.0,	0.0,	 ARM),
		btVector3(	-ARM,	0.0,	 0.0),
		btVector3(	 0.0,	0.0,	-ARM),
		btVector3(	 ARM,	0.0,	 0.0)
	};
	float hand[4] = {1.0, -1.0, 1.0, -1.0};
	btVector4 actual;
	bool autoPilot = false;
	btVector3 force;
	
public:
	Quad (btDynamicsWorld* ownerWorld, const btVector3& positionOffset);
	~Quad ();
	void apply(void);
	btScalar throttle;
	btVector4 signal;
	void (*control)(Quad *quad);
	btVector3 getAcceleration();
	void getRotation(btScalar &yaw, btScalar &pitch, btScalar &roll);
	btScalar getHeight();
	void drawForce();
	void switchAutoPilot();
};

#endif
