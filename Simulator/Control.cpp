#include "Control.h"
btScalar yaw = 0, pitch = 0, roll = 0;
btScalar dyaw = 0, dpitch = 0, droll = 0;

void updateEuler(Quad *quad)
{
	btScalar nyaw, npitch, nroll;
	quad->getRotation(nyaw, npitch, nroll);
	nyaw += error(0.01);
	npitch += error(0.01);
	nroll += error(0.01);
	dyaw = nyaw - yaw;
	dpitch = npitch - pitch;
	droll = nroll - roll;
	yaw = nyaw;
	pitch = npitch;
	roll = nroll;
}
void controlFunction(Quad *quad)
{
	btVector3 acc = quad->getAcceleration();
	updateEuler(quad);
	quad->signal[0] = quad->throttle + 2.0 * roll + 3.0 * droll;
	quad->signal[1] = quad->throttle + 2.0 * yaw + 3.0 * dyaw;
	quad->signal[2] = quad->throttle - 2.0 * roll - 3.0 * droll;
	quad->signal[3] = quad->throttle - 2.0 * yaw - 3.0 * dyaw;
}