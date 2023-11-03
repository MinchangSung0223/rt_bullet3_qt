cmake_minimum_required(VERSION 3.8)
project(LR LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
set(JOINTNUM 7 CACHE STRING "Number of joints")
file(REMOVE ${PROJECT_SOURCE_DIR}/type.h)
configure_file(${PROJECT_SOURCE_DIR}/type_template.h.in ${PROJECT_SOURCE_DIR}/type.h)
file(GLOB SOURCE_FILES "*.cpp" "*.c" "*.h")

add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES}
            )

add_library(sub::LR ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})

find_package(Eigen3 3.0 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen jsoncpp)
