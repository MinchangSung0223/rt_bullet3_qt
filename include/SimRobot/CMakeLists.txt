cmake_minimum_required(VERSION 3.8)
project(SimRobot LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c" "*.h")

set(BULLET_PHYSICS_SOURCE_DIR  "/opt/bullet3")
set(BULLET_PHYSICS_LIB  "/opt/bullet3/build_cmake/local_install/lib")

INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator		
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
		${SIM_ROBOT_HEADERS_DIR}
  )
  
SET(RobotSimulator_SRCS_GUI
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.h	
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsClientExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelper.cpp
#	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelperTCP.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsServerExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsClientExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelper.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsSharedMemoryCommands.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsSharedMemoryPublic.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/PhysicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
	${SIM_ROBOT_HEADERS_DIR}	
)	

SET(BULLET_GUI_LIBS BulletRobotics BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common)

find_package(Eigen3 3.0 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED)
add_library(${PROJECT_NAME} SHARED STATIC  ${SOURCE_FILES}  ${RobotSimulator_SRCS_GUI} )
add_library(sub::SimRobot ALIAS ${PROJECT_NAME} )
target_include_directories( ${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen jsoncpp ${BULLET_GUI_LIBS})
target_compile_definitions(${PROJECT_NAME} PUBLIC -DBT_USE_DOUBLE_PRECISION)	




