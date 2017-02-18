#include "URRobotJointPControl.h"
#include "LoopInterface.h"
#include <stdio.h>
int main(int argc, char** argv) {
	URRobotInterface* ur5 = new URRobotJointPControl();
	if(!ur5->initialize(50000)){
		fprintf(stderr, "Robot init failed.\n");
		return 0;
	}

	LoopInterface stop_signal;
	while(!stop_signal.should_stop()){
		ur5->controlLoop();
	}
	ur5->powerOffRobot();
	printf("Bye Bye~\n");
	delete ur5;
	return 0;
}

