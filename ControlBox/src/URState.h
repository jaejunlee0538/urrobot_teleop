//
// Created by ub1404 on 17. 2. 16.
//

#ifndef PLAYGROUND_UR5STATE_H
#define PLAYGROUND_UR5STATE_H
#include "Button.h"
#include <stdint.h>
struct URState {
  uint64_t sequence;		//loop count.
  uint8_t robot_mode_id;	//robot mode											[robotinterface_get_robot_mode]
  uint8_t joint_mode_id[6]; //joint mode of each joints 							[robotinterface_get_joint_mode]

  double q_act[6];		//actual joint angles						[robotinterface_get_actual]
  double qd_act[6];		//actual angular velocities					[robotinterface_get_actual]
  double i_act[6];		//currents in each joints					[robotinterface_get_target_current]

  double q_des[6];		//target joint angles						[robotinterface_get_target]
  double qd_des[6];		//target joint angular velocities			[robotinterface_get_target]
  double qdd_des[6];		//target joint angular accelerations		[robotinterface_get_target]
  double i_des[6];		//target currents							[robotinterface_get_target_current]
  double moment_des[6];	//??										[robotinterface_get_target_moment]

  bool is_power_on_robot;//														[robotinterface_is_power_on_robot]
  bool is_security_stopped;//													[robotinterface_is_security_stopped]
  bool is_emergency_stopped;//													[robotinterface_is_emergency_stopped]
  bool is_extra_button_pressed;//Small black button behind teaching pendant	[robotinterface_is_extra_button_pressed]
  bool is_power_button_pressed; //??											[robotinterface_is_power_button_pressed]
  bool is_safety_signal_such_that_we_should_stop; // 							[robotinterface_is_safety_signal_such_that_we_should_stop]
  bool is_robot_connected;                                                    //[robotinterface_is_robot_connected]
  uint8_t digital_inputs;
  /*****************************************************************************/

  double tcp_force[6];	//3 forces along xyz and 3 twists of xyz.	[robotinterface_get_tcp_force]
  double tcp_speed[6];	//3 linear velocity and 3 angular velocity.	[robotinterface_get_tcp_speed]
  double tcp_wrench[6];	//tcp_wrench is one of ur configuration.	[robotinterface_set_tcp_wrench]
  double tcp_pose[6];	//tcp_pose is one of ur configuration.		[robotinterface_get_tcp]
  double tcp_force_scalar;//											[robotinterface_get_tcp_force_scalar]
  double tcp_power;		//											[robotinterface_get_tcp_power]
  double tcp_payload;	//payload									[robotinterface_set_tcp_payload]
  double tcp_payload_cog;//center of gravity of payload w.r.t tool frame		[robotinterface_get_tcp_payload_cog]
  double power;	//													[robotinterface_get_power]

  Button extra_button;
};
#endif //PLAYGROUND_UR5STATE_H
