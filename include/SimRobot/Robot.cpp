#include "Robot.h"
// Constructor for robots with GUI
void Robot::InitializeRobot(){
    this->robot_id = robot_id;
    int urdf_joint_num = this->sim->getNumJoints(this->robot_id);
    this->actuated_joint_num = 0;
    for (int i = 0; i < urdf_joint_num; i++)
    {
        b3JointInfo jointInfo;
        this->sim->getJointInfo(this->robot_id, i, &jointInfo);
        if (jointInfo.m_jointName[0] && jointInfo.m_jointType!=eFixedType)
        {
            this->actuated_joint_name.push_back(jointInfo.m_jointName);
            this->actuated_joint_id.push_back(i);		
            // initialize motor
            b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
            controlArgs.m_maxTorqueValue  = 0.0;
            this->sim->setJointMotorControl(this->robot_id,i,controlArgs);				
            b3RobotSimulatorJointMotorArgs controlArgs2(CONTROL_MODE_TORQUE);
            controlArgs2.m_maxTorqueValue  = 0.0;			
            this->sim->setJointMotorControl(this->robot_id,i,controlArgs2);
            actuated_joint_num++;	
        }
        
    }
    this->eef_num = urdf_joint_num-1;
    this->actuated_joint_num = actuated_joint_num;
    this->sim->enableJointForceTorqueSensor(this->robot_id,this->eef_num,1);
}

Robot::Robot( class b3RobotSimulatorClientAPI* sim, int robot_id){
    // Initialize the robot with GUI
    this->sim=sim;
    this->robot_id=robot_id;
    this->InitializeRobot();
}
	
JVec Robot::get_q(){
    JVec q(this->actuated_joint_num);
    b3JointSensorState jointStates;
    int numJoints = sim->getNumJoints(this->robot_id);
	for (int i = 0; i < this->actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robot_id,this->actuated_joint_id.at(i), &jointStates)){
			q[i] = jointStates.m_jointPosition;
		}
	}    
    return q;
}
JVec Robot::get_q_dot(){
    JVec q_dot(this->actuated_joint_num);
    b3JointSensorState jointStates;
    int numJoints = sim->getNumJoints(this->robot_id);
	for (int i = 0; i < this->actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robot_id,this->actuated_joint_id.at(i), &jointStates)){
			q_dot[i] = jointStates.m_jointVelocity;
		}
	}    
    return q_dot;
}
SE3 Robot::get_eef_pose(){
	SE3 pose = SE3::Identity(4,4);
	b3LinkState linkState;
	bool computeVelocity = true;
	bool computeForwardKinematics = true;
	sim->getLinkState(this->robot_id, this->eef_num, computeVelocity, computeForwardKinematics, &linkState);
	JVec pos(3,1);
	pos<< linkState.m_worldLinkFramePosition[0], linkState.m_worldLinkFramePosition[1] , linkState.m_worldLinkFramePosition[2];
	btQuaternion orn = btQuaternion(linkState.m_worldLinkFrameOrientation[0],linkState.m_worldLinkFrameOrientation[1],linkState.m_worldLinkFrameOrientation[2],linkState.m_worldLinkFrameOrientation[3]);
	btMatrix3x3 R = btMatrix3x3(orn);
	pose(0,3) = pos[0];
	pose(1,3) = pos[1];
	pose(2,3) = pos[2];		
	for(int i =0;i<3;i++){
		btVector3 r = R[i];
		for(int j =0;j<3;j++){		
			pose(i,j) = r[j];
		}
	}
	return pose;
}

void Robot::reset_q(JVec q){
	for (int i = 0; i<q.size();i++){
		this->sim->resetJointState(this->robot_id,this->actuated_joint_id.at(i),q[i]);
	}
}
JVec saturate(JVec val, JVec max_val){
	JVec retVal = Map<JVec>(val.data(), val.rows(), val.cols());
	for (int i = 0; i<val.size();i++){
		retVal[i] = min(max(val[i],-max_val[i]),max_val[i]);
	}
	return retVal;
}
void Robot::set_torques(JVec torques ,JVec  max_torques){
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	JVec saturated_torques = saturate(torques,max_torques);
	for (int i = 0; i<torques.size();i++){
		controlArgs.m_maxTorqueValue  =saturated_torques[i];
		sim->setJointMotorControl(this->robot_id,this->actuated_joint_id.at(i),controlArgs);
	}	
}
Vector3d Robot::get_eef_forces(){
	Vector3d ret;
	b3JointSensorState jointStates;
	int numJoints = sim->getNumJoints(this->robot_id);
	if(sim->getJointState(this->robot_id,this->eef_num, &jointStates)){
        ret[0]=jointStates.m_jointForceTorque[0];
        ret[1]=jointStates.m_jointForceTorque[1];
        ret[2]=jointStates.m_jointForceTorque[2];
	}
	return ret;
}	
Vector3d Robot::get_eef_moments(){
	Vector3d ret;
	b3JointSensorState jointStates;
	int numJoints = sim->getNumJoints(this->robot_id);
	if(sim->getJointState(this->robot_id,this->eef_num, &jointStates)){
        ret[0]=jointStates.m_jointForceTorque[3];
        ret[1]=jointStates.m_jointForceTorque[4];
        ret[2]=jointStates.m_jointForceTorque[5];
	}
	return ret;
}	

void Robot::apply_ext_forces(Vector3d forces){
	btVector3 force(-forces(0),-forces(1),-forces(2)) ;
	btVector3 position(0,0,0);
	sim->applyExternalForce(this->robot_id,this->eef_num,force,position,EF_LINK_FRAME);
	//sim->applyExternalTorque(this->robotId,this->eef_num,torque,EF_LINK_FRAME);
}
void Robot::apply_ext_torques(Vector3d torques){
	btVector3 torque(-torques(0),-torques(1),-torques(2)) ;
	sim->applyExternalTorque(this->robot_id,this->eef_num,torque,EF_LINK_FRAME);
}
JVec Robot::calc_inverse_dynamics(){
    JVec torques;
    double q[JOINTNUM];
    double q_dot[JOINTNUM];
    double q_ddot[JOINTNUM];
    double jointForcesOutput[JOINTNUM];
    JVec q_ = this->get_q();
    JVec q_dot_ = this->get_q_dot();
    for(int i =0;i<JOINTNUM;i++){
        q[i] = q_[i];
        q_dot[i] = q_dot_[i];
        q_ddot[i] = 0;
    }

    this->sim->calculateInverseDynamics(this->robot_id,q,q_dot,q_ddot,jointForcesOutput);
    for(int i =0;i<JOINTNUM;i++)
        torques[i] = jointForcesOutput[i];
    return torques;
}
// Destructor
Robot::~Robot() {
    
    // Cleanup resources if needed
    // Add your cleanup code here
}