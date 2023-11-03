#ifndef ROBOT_SETUP_H
#define ROBOT_SETUP_H

#include "b3RobotSimulatorClientAPI.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include "../LieGroupRobotics/type.h"

#include <jsoncpp/json/json.h>
#pragma comment(lib, "jsoncpp.lib")

using namespace Eigen;
using namespace std;

class Robot
{
private:
	class b3RobotSimulatorClientAPI* sim;
	int robot_id;
	int actuated_joint_num;
	int eef_num;
	int line_id;
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;
   
public:
	Robot(class b3RobotSimulatorClientAPI* sim,int robot_id);
	void InitializeRobot();
	JVec get_q();
	JVec get_q_dot();
	SE3 get_eef_pose();
	void reset_q(JVec q);	
	void set_torques(JVec torques ,JVec  max_torques);
	Vector3d get_eef_forces();
	Vector3d get_eef_moments();
	void apply_ext_forces(Vector3d forces);
	void apply_ext_torques(Vector3d torques);
	JVec calc_inverse_dynamics();
	virtual ~Robot();

};
#endif  //ROBOT_SETUP_H
