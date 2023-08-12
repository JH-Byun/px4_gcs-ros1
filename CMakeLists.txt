cmake_minimum_required(VERSION 2.8.0)
project(px4_gcs)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -std=gnu++11")

# find_package(catkin REQUIRED COMPONENTS qt_build roscpp mavros_msgs geometry_msgs sensor_msgs nav_msgs keyboard cmake_modules)
find_package(catkin REQUIRED COMPONENTS roscpp mavros_msgs geometry_msgs sensor_msgs nav_msgs keyboard cmake_modules)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
find_package(LibXml2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread timer)
find_package(Eigen REQUIRED)

include(${QT_USE_FILE})
include_directories( include 
		${catkin_INCLUDE_DIRS} 
		${LIBXML2_INCLUDE_DIR} 
		${Boost_INCLUDE_DIR} 
		${Eigen_INCLUDE_DIRS}
		${CMAKE_CURRENT_BINARY_DIR})

catkin_package(
		INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}		
)

# rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here


file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
	include/main_window.hpp
	include/qnode.hpp)

file(GLOB_RECURSE QT_MOC2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
	include/qcustomplot.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_CPP(QT_MOC_H ${QT_MOC2})


#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
	src/main.cpp
	src/main_window.cpp
	src/qcustomplot.cpp 
	src/qnode.cpp
	src/modules/keyboard_module.cpp
	src/modules/drawing_module.cpp)

add_executable(px4_gcs ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${QT_MOC_H})
target_link_libraries(px4_gcs ${QT_LIBRARIES} ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES} ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${Boost_TIMER_RELEASE} ${Eigen_LIBRARIES})
add_dependencies(px4_gcs keyboard_generate_messages_cpp)

add_executable(px4_gcs_keyboard src/main_keyboard.cpp src/keyboard_handler.cpp)
target_link_libraries(px4_gcs_keyboard ${catkin_LIBRARIES})
add_dependencies(px4_gcs_keyboard keyboard_generate_messages_cpp)
