#include <stdio.h>
#include <string.h>
#include "robotinterface.h"
#include "Configuration.h"
#include "microprocessor_commands.h"
#include "microprocessor_definitions.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include <signal.h>
#include <pthread.h>
#include "ur_kin.h"

pid_t pid;
struct sched_param sch_param;

const double home_position[6] = {0.0, -1.570796, 0.0, -1.570796, 0.0, 0.0};
const double zero_vector[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

//These values are default values in GUI from manufacturer.
const double MAX_JOINT_ACC = 1432.4 * M_PI / 180.0;
const double MAX_JOINT_VEL = 183.3 * M_PI / 180.0;
const double DEFAULT_JOINT_ACC = 80.0 * M_PI / 180.0;
const double DEFAULT_JOINT_VEL = 60.0 * M_PI / 180.0;
const double SAFE_JOINT_ACC = MAX_JOINT_ACC * 0.7;
const double SAFE_JOINT_VEL = MAX_JOINT_VEL * 0.7;

const double LIMIT_JOINT_POSITON[2] = {-340.0*M_PI/180.0, 340.0*M_PI/180.0};


bool robot_running = true;

bool interface_opend = false;
bool robot_power_on = false;
bool brake_released = false;
bool robot_initialized = false;

/**  state variables **/
double robot_current_time = 0.0;
double robot_joint_angle[6] = {0.0,0.0,0.0,0.0,0.0};			//robotinterface_get_actual_position //see also robotinterface_get_actual
double robot_joint_velocity[6] = {0.0,0.0,0.0,0.0,0.0};			//robotinterface_get_actual_velocity //see also robotinterface_get_actual
double robot_joint_current[6] = {0.0,0.0,0.0,0.0,0.0};			//robotinterface_get_actual_current
double robot_tool_acc_x[6] = {0.0,0.0,0.0,0.0,0.0};				//robotinterface_get_tool_accelerometer_readings
double robot_tool_acc_y[6] = {0.0,0.0,0.0,0.0,0.0};				//robotinterface_get_tool_accelerometer_readings
double robot_tool_acc_z[6] = {0.0,0.0,0.0,0.0,0.0};				//robotinterface_get_tool_accelerometer_readings
double robot_tcp_position[6] = {0.0,0.0,0.0,0.0,0.0};			//robotinterface_get_tcp
double robot_tcp_speed[6] = {0.0,0.0,0.0,0.0,0.0};				//robotinterface_get_tcp_speed
double robot_tcp_wrench[6] = {0.0,0.0,0.0,0.0,0.0};				//robotinterface_get_tcp_wrench
double robot_tcp_force[6] = {0.0,0.0,0.0,0.0,0.0};				//robotinterface_get_tcp_force
double robot_target_position[6] = {0.0,0.0,0.0,0.0,0.0};		//robotinterface_get_target_position		//see also robotinterface_get_target
double robot_target_velocity[6] = {0.0,0.0,0.0,0.0,0.0};		//robotinterface_get_target_velocity		//see also robotinterface_get_target
double robot_target_acceleration[6] = {0.0,0.0,0.0,0.0,0.0};	//robotinterface_get_target_acceleration	//see also robotinterface_get_target
double robot_target_current[6] = {0.0,0.0,0.0,0.0,0.0};			//robotinterface_get_target_current
double robot_target_moment[6] = {0.0,0.0,0.0,0.0,0.0};			//robotinterface_get_target_moment
double robot_tcp_force_scalar = 0.0;							//robotinterface_get_tcp_force_scalar
double robot_tcp_power = 0.0;									//robotinterface_get_tcp_power
double robot_power = 0.0;										//robotinterface_get_power
double robot_tcp_payload = 0.0;									//robotinterface_get_tcp_payload

#define ROBOT_STATE_SIGNAL_NO_POWER 		0x01
#define ROBOT_STATE_SIGNA_SECURITY_STOP		0x02
#define ROBOT_STATE_SIGNA_EMERGENCY_STOP  	0x04
#define ROBOT_STATE_SIGNA_SAFETY_SIGNAL		0x08

int robot_safety_signal = 0;


struct robot_running_state{
	int robot_mode;
	int is_power_on;
	int is_security_stopped;
	int is_emergency_stopped;
	int is_extra_button_pressed;
	int is_safety_signal_such_that_we_should_stop;
	int joints_mode[6];
} robot_running_state;

void timeStamp(const char *msg);
void stopRobot(int sig);
void* waitOperatorsPermission(void* arg);
void print_joint_mode();
int initRobot(bool negative);
void powerOffRobot(void);
int movej(const double* q, double qd);
int moveHome(double qd);
void readRobotState(void);
void printRobotState(void);
void delayNSteps(unsigned int n);
void delayNSeconds(unsigned int n);
void recordRobotState(void);
void waitPermission(const char * message, void* signalVar);


void timeStamp(const char *msg) {
	static bool first_run = true;
	static timeval t0;
	timeval t1, t_elapsed;
	if(first_run)
	{
		first_run = false;
		gettimeofday(&t0, 0);
	}

	gettimeofday(&t1, 0);
	timersub(&t1, &t0, &t_elapsed);
	printf("\t[%10.3lf] : %s\n", t_elapsed.tv_sec + t_elapsed.tv_usec / 1000000.0, msg);
}
//-----------------------signal
void stopRobot(int sig){
	printf("stop robot.\n");
	robot_running = false;
}

void addStopSignal(void)
{
	signal(SIGINT, stopRobot);
}

inline bool isRunningRobot(void)
{
	return robot_running;
}

void* waitOperatorsPermission(void* arg){
	int * flag = (int*) arg;
	char msg = 'y';
	while(robot_running){
		printf("\n---Do you want to proceed?[y,n]\n\n");
		msg = getchar();getchar();
		if(msg == 'y')
		{
			*flag = 1;
			return NULL;
		}
		else if(msg == 'n'){
			robot_running = false;
			return NULL;
		}
		else{
			printf("only 'y' or 'n'\n");
		}
	}
	
	return NULL;
}


void print_joint_mode() {
	int joint;
	int mode;
	for (joint = 0; joint < 6; ++joint) {
		mode = robotinterface_get_joint_mode(joint);
		switch(mode) {
		case JOINT_MOTOR_INITIALISATION_MODE:
			printf("Joint %i mode: JOINT_MOTOR_INITIALISATION_MODE\n", joint);
			break;
		case JOINT_BOOTING_MODE:
			printf("Joint %i mode: JOINT_BOOTING_MODE\n", joint);
			break;
		case JOINT_POWER_OFF_MODE:
			printf("Joint %i mode: JOINT_POWER_OFF_MODE\n", joint);
			break;
		case JOINT_BOOTLOADER_MODE:
			printf("Joint %i mode: JOINT_BOOTLOADER_MODE\n", joint);
			break;
		case JOINT_CALIBRATION_MODE:
			printf("Joint %i mode: JOINT_CALIBRATION_MODE\n", joint);
			break;
		case JOINT_SECURITY_STOPPED_MODE:
			printf("Joint %i mode: JOINT_SECURITY_STOPPED_MODE\n", joint);
			break;
		case JOINT_FAULT_MODE:
			printf("Joint %i mode: JOINT_FAULT_MODE\n", joint);
			break;
		case JOINT_RUNNING_MODE:
			printf("Joint %i mode: JOINT_RUNNING_MODE\n", joint);
			break;
		case JOINT_INITIALISATION_MODE:
			printf("Joint %i mode: JOINT_INITIALISATION_MODE\n", joint);
			break;
		case JOINT_IDLE_MODE:
			printf("Joint %i mode: JOINT_IDLE_MODE\n", joint);
			break;
		default:
			printf("Joint %i mode: Unknown mode\n", joint);
		}
	}
}

/*
initialize robot.
User should go on step by step.
*/
int initRobot(bool negative){
	int i=0;
	int k=0;
	int retryCount = 0;

	printf("Loading robot configuration\n");
	configuration_load();

	printf("Setting RT priority\n");
	pid = getpid();
	sch_param.sched_priority = 99;
	if (sched_setscheduler(pid, SCHED_FIFO, &sch_param) == 0) {
		printf("- Prioority set\n");
	} else {
		printf("- Priority not set, error: %i\n", errno);
		return 0;
	}


	//interface open
	printf("\n\n----------Interface Open---------\n");
	if(robot_running){
		printf("interface open\n");
		sleep(1.0);
		robotinterface_open(0);
		retryCount = 2500;
		while(retryCount>0 && !robotinterface_is_robot_connected()){
			robotinterface_read_state_blocking();
			robotinterface_command_empty_command();
			robotinterface_send();
			retryCount--;
		}
	}
		
	if(robot_running==false || !robotinterface_is_robot_connected()){
		printf("robot connection failed\n");
		robotinterface_close();
		return 0;
	}

	retryCount = 2500;
	while(robot_running && retryCount>0){
		robotinterface_read_state_blocking();
		readRobotState();
		if(robot_running_state.robot_mode == ROBOT_NO_POWER_MODE){
			break;
		}
		robotinterface_command_empty_command();
		robotinterface_send();
		retryCount--;
	}

	if(robot_running==false || retryCount <= 0){
		printf("robot is not ready for turning on power\n");
		robotinterface_close();
		return 0;
	}

	//robot power on
	printf("\n\n----------Robot Power On---------\n");
	delayNSeconds(2);
	if(robot_running){
		printf("robot power on\n");
		for (i = 0; i < 10 && !robotinterface_is_power_on_robot(); ++i) {
			retryCount=250;
			while (retryCount > 0) {
				robotinterface_read_state_blocking();
				if(retryCount==250)
					robotinterface_power_on_robot();
				readRobotState();
				robotinterface_command_empty_command();
				robotinterface_send();
				if (robot_running_state.is_power_on) {
					break;
				}
				retryCount--;
			}
		}
	}

	if(robot_running==false || !robot_running_state.is_power_on){
		printf("robot cannot be turned on\n");
		robotinterface_power_off_robot();
		robotinterface_close();
		return 0;
	}

	printf("waiting for every joints are ready to release\n");
	bool all_joint_ready = false;
	i=0;
	while(robot_running){
		robotinterface_read_state_blocking();
		readRobotState();
		robotinterface_command_empty_command();
		robotinterface_send();
		
		if(i==125){
			all_joint_ready = true;
			for(k=0;k<6;k++){
				all_joint_ready = all_joint_ready && 
				(robot_running_state.joints_mode[k] == JOINT_IDLE_MODE);
			}
			if(all_joint_ready==true){
				break;
			}
			i=0;
		}
		i++;
	}
	if(robot_running==false || !all_joint_ready){
		printf("robot joints cannot be ready\n");
		robotinterface_power_off_robot();
		robotinterface_close();
		return 0;
	}

	//brake release
	printf("\n\n----------Brake Releasing---------\n");
	delayNSeconds(2);

	if(robot_running){
		printf("release brake\n");
		robotinterface_power_on_robot();

		all_joint_ready = false;
		i=0;
		while(robot_running){
			robotinterface_read_state_blocking();
			readRobotState();
			robotinterface_command_empty_command();
			robotinterface_send();
			if(i==125){
				all_joint_ready = true;
				for(k=0;k<6;k++){
					all_joint_ready = all_joint_ready && 
					(robot_running_state.joints_mode[k] == JOINT_INITIALISATION_MODE);
				}
				if(all_joint_ready==true){
					break;
				}
				i=0;
			}
			i++;
		}
	}
	if(robot_running==false || !all_joint_ready){
		printf("robot cannot be ready for initializing\n");
		robotinterface_power_off_robot();
		robotinterface_close();
		return 0;
	}


	printf("\n\n----------Joints Initializing---------\n");
	int move_joints = 0;
	waitPermission("This process will rotate every joints.\n\tBe sure that there is nothing in the way.",(void*)&move_joints);
	while(!move_joints && robot_running){
		robotinterface_read_state_blocking();
		robotinterface_command_empty_command();
		robotinterface_send();
	}
	double init_velocity[6];
	for(int i=0;i<6;i++){
		init_velocity[i] = (negative)?-0.14:0.14;
	}
	if(robot_running){
		i=0;
		do {
			++i;
			robotinterface_read_state_blocking();
			readRobotState();
			if ((i & 255) == 0) { 
				printf("Current joint modes:\n");
				print_joint_mode(); 
				printf("\n");
			}
			for (k=0; k<6; ++k) {
				if(robot_running_state.joints_mode[k] == JOINT_RUNNING_MODE){
					init_velocity[k] = 0.0;
				}
			}
			robotinterface_command_velocity(init_velocity);
			robotinterface_send();
		} while (robot_running_state.robot_mode == ROBOT_INITIALIZING_MODE && robot_running);
	}
	if(robot_running==false || robot_running_state.robot_mode!=ROBOT_RUNNING_MODE){
		printf("robot cannot be initialized\n");
		robotinterface_power_off_robot();
		robotinterface_close();
		return 0;
	}

	return 1;
}

int initWithMoveHome(void){
	if(!initRobot(true)){
		printf("............init failed. terminate program\n");
		return 0;
	}

	printf("\n............move home\n");
	moveHome(0.1);
	return 1;
}

/*
 *
 */
void powerOffRobot(void){
	printf("\n............robot will turn off after %i sec\n",3);
	delayNSeconds(3);
	robotinterface_power_off_robot();
	robotinterface_close();
}

/*
movej : Not completed yet.
There's no accelleration at the beginning position 
and no decelleration at the end position.
*/
int movej(const double* q, double qd){
	qd = fabs(qd);

	double speed_vector[6];
	double dx[6];
	bool all_joint_done = false;
	int i=0;
	do{
		all_joint_done = true;
		robotinterface_read_state_blocking();
		readRobotState();
		for(i=0;i<6;i++){
			dx[i] = q[i] - robot_joint_angle[i];
			if(fabs(dx[i])>0.0020472){
				speed_vector[i] = qd * dx[i]/fabs(dx[i]);
				all_joint_done = all_joint_done && false;
			}
			else{
				speed_vector[i] = 0.0;
			}
		}
		robotinterface_command_velocity(speed_vector);

		robotinterface_send();
	}while(!all_joint_done && robot_running);

	if(all_joint_done)
		return 1;

	return 0;
}

//
//int movej(const double* q, double qd, double qdd){
//	qd = fabs(qd);
//	qdd = fabs(qdd);
//
//	double qdd_array[6];
//	double qd_array[6];
//	double q_array[6];
//	int i= 0;
//
//	//first read current state.
//	robotinterface_read_state_blocking();
//	readRobotState();
//	robotinterface_command_empty_command();
//	robotinterface_send();
//
//	for(i=0;i<6;i++){
//		int sign = q[i] - q_array[i];
//		qd_array[i] = qd * sign;
//		qdd_array[i] = qdd * sign;
//	}
//
//	bool all_joint_complete = false;
//	do{
//		robotinterface_read_state_blocking();
//		readRobotState();
//
//		// if every joints arrive at goal position, all_joint_done will remain True.
//		all_joint_complete = true;
//		for(i=0;i<6;i++){
//			if(fabs(q_array[i] - robot_joint_angle[i]) > 0.0020472)
//				all_joint_complete =all_joint_complete && false;
//		}
//
//		robotinterface_command_position_velocity_acceleration(q_array,qd_array,qdd_array);
//
//		robotinterface_send();
//	}while(!all_joint_complete && robot_running);
//
//	return 0;
//}
//
//int movej(const double* q, const double* qd, const double qdd){
//	double qdd_array[6];
//	double qd_array[6];
//	double q_array[6];
//
//	//acceleration bounding
//	double _qdd = fabs(qdd);
//	if(_qdd > SAFE_JOINT_ACC)
//		_qdd = SAFE_JOINT_ACC;
//	_qdd = _qdd * qdd / fabs(qdd);
//
//	for(int i=0;i<6;i++){
//		//position bounding
//		if(q[i]<LIMIT_JOINT_POSITON[0] || q[i]>LIMIT_JOINT_POSITON[1])
//			return 0;
//
//		//velocity bounding
//		if(fabs(qd[i]) > SAFE_JOINT_VEL)
//			qd_array[i] = qd[i] / fabs(qd[i]) * SAFE_JOINT_VEL;
//		else
//			qd_array[i] = qd[i];
//
//		//acceleration set.
//		qdd_array[i] = _qdd;
//	}
//
//	bool all_joint_complete = false;
//	do{
//		robotinterface_read_state_blocking();
////		robotinterface_get_actual_position(actual_q);
//		readRobotState();
//
//		// if every joints arrive at goal position, all_joint_done will remain True.
//		all_joint_complete = true;
//		for(int i=0;i<6;i++){
//			if(fabs(q_array[i] - robot_joint_angle[i]) > 0.0020472)
//				all_joint_complete =all_joint_complete && false;
//		}
//
////		robotinterface_command_position_velocity_acceleration(q_array,qd_array,qdd_array);
//		robotinterface_command_position_velocity_acceleration(q_array,zero_vector,zero_vector);
//		robotinterface_send();
//	}while(!all_joint_complete && robot_running);
//
//	return 1;
//}

/*
Move robot to HomePosition.
It uses movej.
*/
int moveHome(double qd){
	return movej(home_position,0.1);
//	double speed_vector[6] = {DEFAULT_JOINT_VEL, DEFAULT_JOINT_VEL, DEFAULT_JOINT_VEL, DEFAULT_JOINT_VEL, DEFAULT_JOINT_VEL, DEFAULT_JOINT_VEL};
//	return movej(home_position,speed_vector,DEFAULT_JOINT_ACC);
}

void speedHome_NonBlocking(double qd){
	qd = fabs(qd);

	double speed_vector[6];
	double dx[6];

	readRobotState();
	for(int i=0;i<6;i++){
		dx[i] = home_position[i] - robot_joint_angle[i];
		if(fabs(dx[i])>0.0020472){
			speed_vector[i] = qd * dx[i]/fabs(dx[i]);
		}
		else{
			speed_vector[i] = 0.0;
		}
	}
	robotinterface_command_velocity(speed_vector);
}

/*
 *
 */
message_t security_message;
void readRobotState(void){
	robot_current_time = robotinterface_get_time();
	robotinterface_get_actual(robot_joint_angle,robot_joint_velocity);
	robotinterface_get_actual_current(robot_joint_current);
	robotinterface_get_tool_accelerometer_readings(robot_tool_acc_x,robot_tool_acc_y,robot_tool_acc_z);
	robotinterface_get_tcp(robot_tcp_position);
	robotinterface_get_tcp_speed(robot_tcp_speed);
	robotinterface_get_tcp_wrench(robot_tcp_wrench);
	robotinterface_get_tcp_force(robot_tcp_force);
	robotinterface_get_target(robot_target_position,robot_target_velocity,robot_target_acceleration);
	robotinterface_get_target_current(robot_target_current);
	robotinterface_get_target_moment(robot_target_moment);
	robot_tcp_force_scalar = robotinterface_get_tcp_force_scalar();
	robot_tcp_power = robotinterface_get_tcp_power();
	robot_power = robotinterface_get_power();
	robot_tcp_payload = robotinterface_get_tcp_payload();

	robot_running_state.robot_mode = robotinterface_get_robot_mode();
	robot_running_state.is_power_on = robotinterface_is_power_on_robot();
	robot_running_state.is_security_stopped = robotinterface_is_security_stopped();
	robot_running_state.is_emergency_stopped = robotinterface_is_emergency_stopped();
	robot_running_state.is_extra_button_pressed = robotinterface_is_extra_button_pressed();
	robot_running_state.is_safety_signal_such_that_we_should_stop = robotinterface_is_safety_signal_such_that_we_should_stop();

	robot_safety_signal = 0;
	if(!robot_running_state.is_power_on)
		robot_safety_signal += ROBOT_STATE_SIGNAL_NO_POWER;
	if(robot_running_state.is_security_stopped)
		robot_safety_signal += ROBOT_STATE_SIGNA_SECURITY_STOP;
	if(robot_running_state.is_emergency_stopped)
		robot_safety_signal += ROBOT_EMERGENCY_STOPPED_MODE;
	if(robot_running_state.is_safety_signal_such_that_we_should_stop)
		robot_safety_signal += ROBOT_STATE_SIGNA_SAFETY_SIGNAL;

	for(int i=0;i<6;i++)
		robot_running_state.joints_mode[i] = robotinterface_get_joint_mode(i);

	//Error message
	if(robotinterface_has_security_message()){
		int err_state[10]={0,},err_argument[10]={0,};
		robotinterface_get_security_message(&security_message,err_state,err_argument);
		printf("\n%s\n",security_message.text);
	}


}

void printRobotState(void){
	printf("\n-------robot state--------\n");
	printf("%.5lf,%.5lf,%.5lf,%.5lf,%.5lf,%.5lf\n",robot_joint_angle[0],robot_joint_angle[1],robot_joint_angle[2],robot_joint_angle[3],robot_joint_angle[4],robot_joint_angle[5]);
	printf("%.5lf,%.5lf,%.5lf,%.5lf,%.5lf,%.5lf\n",robot_joint_velocity[0],robot_joint_velocity[1],robot_joint_velocity[2],robot_joint_velocity[3],robot_joint_velocity[4],robot_joint_velocity[5]);
}
/*
Wait for n control cycles.
125 cycles will take 1 sec.
*/
void delayNSteps(unsigned int n){
	for(unsigned int i=0;i<n;i++){
		robotinterface_read_state_blocking();
		robotinterface_command_empty_command();
		robotinterface_send();
	}
}

/*
 *
 */
void delayNSeconds(unsigned int n)
{
	const unsigned int MAX_STEP = n * 125;
	for(unsigned int i=MAX_STEP;i>0;i--){
		robotinterface_read_state_blocking();
		robotinterface_command_empty_command();
		robotinterface_send();
	}
}

/*
 *	Exception : another thread is already running.
 *	It has to be managed.
 */
pthread_t thread;
void waitPermission(const char * message, void* signalVar){
	printf("------------%s------------\n",message);
	pthread_create(&thread, NULL,waitOperatorsPermission, signalVar);
}
/*

*/
void recordRobotState(void){
	
}
