#ifndef __XENO__
#define __XENO__
#endif
#include "main.h"
#include <stdio.h>
RT_TASK Physics_task;
ROBOT_INFO robot_info;
b3RobotSimulatorClientAPI* sim;
double CONTROL_RATE = 1000.0;
btScalar fixedTimeStep = 1. / CONTROL_RATE;
Robot *robot;
LR_Control *control;
JVec q;
JVec q_dot;
JVec q_des=JVec::Zero();

JVec kMaxTorques=JVec::Ones()*1000;
Vector3d eef_forces;
Vector3d eef_moments;


unsigned int cycle_ns = 1000000; /* 1 ms */
double gt=0;
void Physics_run(void *arg){
	RTIME beginCycle, endCycle;
	RTIME beginRead, beginReadbuf, beginWrite, beginWritebuf, beginCompute;
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    double dt=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
    while (run)
	{
        q = robot->get_q();
	    q_dot = robot->get_q_dot();
	    JVec torques = lr::GravityForces(q,control->g,control->Mlist, control->Glist, control->Slist);

	    robot->set_torques(torques,kMaxTorques);
	    //robot->apply_ext_forces(Vector3d(robot_info.act.F_ext[0],robot_info.act.F_ext[1],robot_info.act.F_ext[2]));
	    robot->apply_ext_forces(Vector3d(10,10,10));        
        //robot_info.act.q = JVec(sin(gt*M_PI*1),sin(gt*M_PI*2),sin(gt*M_PI*4),sin(gt*M_PI*8),sin(gt*M_PI*16),sin(gt*M_PI*32),sin(gt*M_PI*64)) ;
        gt+=dt;
    	sim->stepSimulation();
	    robot_info.act.q = q;
	    robot_info.act.q_dot = q_dot;
	    robot_info.act.tau = torques;        
        rt_task_wait_period(NULL); 	//wait for next cycle
    }
}
void signal_handler(int signum)
{
	rt_task_delete(&Physics_task);
	//rt_task_delete(&safety_task);
	//rt_task_delete(&print_task);
	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
    printf("║                       Stopped!                     ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	
    exit(1);

    
}

void runQtApplication(int argc, char* argv[]) {
  QApplication a(argc, argv);
  // style our application with custom dark style
  QApplication::setStyle(new DarkStyle);

  // create frameless window (and set windowState or title)
  FramelessWindow framelessWindow;

  // create our mainwindow instance
  MainWindow *mainWindow = new MainWindow;
  // add the mainwindow to our custom frameless window
  framelessWindow.resize(1600,600);
  framelessWindow.setContent(mainWindow);
  framelessWindow.show();
  a.exec();
}

int main(int argc, char* argv[]){
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);    
	mlockall(MCL_CURRENT|MCL_FUTURE);


    const char* urdfFilePath = "/opt/RobotInfo/urdf/satellite/arm.urdf";
    bool with_gui=0;
    //-----------------Sim Setup------------------------
    sim = new b3RobotSimulatorClientAPI();
    bool isConnected;
    isConnected = sim->connect(eCONNECT_DIRECT);

    if (!isConnected)
    {
        printf("Cannot connect\n");
        return -1;
    }
    sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
    sim->setTimeOut(10);
    sim->syncBodies();	
    sim->setTimeStep(fixedTimeStep);
    sim->setGravity(btVector3(0, 0, -9.8));
    b3RobotSimulatorSetPhysicsEngineParameters args;
    sim->getPhysicsEngineParameters(args);
    int robotId = sim->loadURDF(urdfFilePath);
    int planeId = sim->loadURDF("/opt/RobotInfo/urdf/plane/plane.urdf");
    sim->setRealTimeSimulation(false);
    robot = new Robot(sim,robotId);	
    robot_info.act.F_ext=Vector6d::Zero();
    control = new LR_Control();
    control->LRSetup();

    std::thread qtThread(runQtApplication, argc, argv);

	// RTIndy7 control
	rt_task_create(&Physics_task, "Physics_task", 0, 90, 0);
	rt_task_start(&Physics_task, &Physics_run, NULL);


    pause();
    qtThread.join();	



	signal_handler(0);
    return 0;
}