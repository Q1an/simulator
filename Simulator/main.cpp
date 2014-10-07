#include "Simulator.h"
#include "GlutStuff.h"

int main(int argc,char* argv[])
{
	Simulator simu;
	simu.initPhysics();
	return glutmain(argc, argv, 640, 480, "Simulator", &simu);
}