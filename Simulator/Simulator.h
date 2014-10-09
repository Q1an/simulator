#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class Simulator : public GlutDemoApplication
{
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	
	btBroadphaseInterface*	m_broadphase;
	
	btCollisionDispatcher*	m_dispatcher;
	
	btConstraintSolver*	m_solver;
	
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	
public:
	btAlignedObjectArray<class Quad*> m_quads;
	void initPhysics();
	
	void exitPhysics();
	
	virtual ~Simulator()
	{
		exitPhysics();
	}
	
	void addQuad(const btVector3& startOffset);
	
	virtual void clientMoveAndDisplay();
	
	virtual void displayCallback();
	
	virtual void keyboardCallback(unsigned char key, int x, int y);
	
	static DemoApplication* Create()
	{
		Simulator* demo = new Simulator();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
	
};


#endif
