#ifndef MAIN_H  // 헤더 파일 중복 포함 방지를 위한 전처리기
#define MAIN_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>
#include <string>
#include "iostream"
#include <fstream>
#include <sstream>

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <thread>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

//QT
#include <QApplication>
#include <QSplitter>
#include <QTreeView>
#include <QListView>
#include <QTableView>
#include <QStandardItemModel>
#include <QScreen>

#include "DarkStyle.h"
#include "framelesswindow.h"
#include "mainwindow.h"
#include <iostream>

#include <QApplication>
#include <QResource>
#include <QTextCodec>


#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
//Bullet
#include "b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"
#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
//Control
#include "liegroup_robotics.h"
#include "Robot.h"
#include "LR_Control.h"

#define NSEC_PER_SEC 			1000000000

static int run = 1;
#define ASSERT_EQ(a, b) assert((a) == (b));

typedef struct STATE{
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;

    double s_time;
}state;

typedef struct ROBOT_INFO{
	int Position;
	int aq_inc[JOINTNUM];
	int atoq_per[JOINTNUM];
	short dtor_per[JOINTNUM];
	int statusword[JOINTNUM];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;

	STATE act;
	STATE des;
	STATE nom;

}ROBOT_INFO;
extern ROBOT_INFO robot_info;
extern std::mutex g_pages_mutex;
extern unsigned int cycle_ns;
extern double gt;

#endif  // MAIN_H

