#include "Control.h"

void controlFunction(Quad *quad)
{
	btVector3 acc = quad->getAcceleration();
	quad->throttle[3] = 10.0;
}