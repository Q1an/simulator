#include "Quad.h"
#include "Control.h"

float error(float range = 1.0)
{
	float rdm = (random() % 1000) / 1000.0;
	return range * (2 * rdm - 1);
}
float slice(float x, float lower, float upper)
{
	return fmin(fmax(x, lower), upper);
}
Quad::Quad (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
:	m_ownerWorld (ownerWorld), throttle(btVector4(0.0, 0.0, 0.0, 0.0)),
	actual(btVector4(0.0, 0.0, 0.0, 0.0)),
	control(controlFunction)
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
Quad::~Quad()
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

void Quad::apply(void)
{
		if (autoPilot)
		{
			control(this);
		}
		btTransform tran = btTransform(m_bodies[0]->getOrientation());
		m_bodies[0]->activate();
		for (int i = 0; i < 4; i++)
		{
			throttle[i] = slice(throttle[i], MIN_THROTTLE, MAX_THROTTLE);
			actual[i] = slice(throttle[i], actual[i] - MAX_THROTTLE_STEP, actual[i] + MAX_THROTTLE_STEP);
			m_bodies[0]->applyForce(tran*btVector3(0.0, actual[i] + error(0.1), 0.0), tran*rotor[i]);
			m_bodies[0]->applyTorque(tran*btVector3(0.0, hand[i]*TORQUE_K * actual[i], 0.0));
		}
	force = m_bodies[0]->getTotalForce()-m_bodies[0]->getGravity()*MASS;
}
btVector3 Quad::getAcceleration()
{
	btTransform tran = btTransform(m_bodies[0]->getOrientation());
	return tran * force / MASS;
}
void vertex(btVector3 &v)
{
	glVertex3d(v.getX(), v.getY(), v.getZ());
}
void Quad::drawForce()
{
	btTransform tr = btTransform(m_bodies[0]->getWorldTransform());
	
	glBegin(GL_LINES);
	
	// x
	glColor3f(255.f,0,0);
	btVector3 vX = tr.getOrigin()+ 0.1*(getAcceleration());
	vertex(tr.getOrigin());	vertex(vX);	glEnd();
	
}
void Quad::switchAutoPilot()
{
	autoPilot = !autoPilot;
}