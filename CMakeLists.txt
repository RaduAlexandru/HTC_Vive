cmake_minimum_required(VERSION 2.8)
project(vive_rviz)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)		# Header folder
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DAPC_CAPTURE_DEBUG=1")


find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	image_transport
)

find_package(catkin REQUIRED
#	rviz
	#pcl
	#pcl_ros
	#geometry_msgs
	#sensor_msgs
)

find_package( catkin REQUIRED COMPONENTS roscpp tf cv_bridge)

find_package(Boost REQUIRED COMPONENTS
		thread
		filesystem
		)

#find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
#include(${QT_USE_FILE})


find_package( OpenMP REQUIRED)
find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
#find_package(PCL  REQUIRED)
find_package(PCL 1.8 REQUIRED)	
#find_package(rviz REQUIRED )
set (VRUI_INCLUDE_DIR /usr/local/include/Vrui-4.2)
find_package(X11 REQUIRED)
#set (VRUI_LIBS /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libVrui.g++-3.so /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libGeometry.g++-3.so )
file(GLOB VRUI_LIBS  /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/*.so )

#VTK stuff
#set (VTK_USE_FILE /home/shuttle/Sources/VTK/CMake/UseVTK.cmake)
#include(${VTK_USE_FILE})
#message (${VTK_USE_FILE})
#file(GLOB VTK_LIBRARIES /home/shuttle/Sources/VTK-build/lib/*.so )
#find_package(VTK REQUIRED) #with this set it will get the system vtk at version 5.8
#set (VTK_INCLUDE_DIRS /home/shuttle/Sources/vtkVRUI-superbuild-build/install/include/vtk-7.1/ )
#include_directories(${VTK_INCLUDE_DIRS})
#message( ${VTK_INCLUDE_DIRS} )
#message (${VTK_LIBRARIES}  )

#attempt2
set(VTK_DIR "/home/system/Sources/VTK-build/")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
#message( ${VTK_INCLUDE_DIRS} )
#message (${VTK_LIBRARIES}  )



set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

include_directories(${PROJECT_INCLUDE_DIR})
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${VRUI_INCLUDE_DIR}   
  ${X11_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS} 
  ${GLUT_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
	

catkin_package()

#qt4_wrap_cpp(MOC_SRCS
#	include/vive_rviz.h
#)

message (${PCL_LIBRARIES})

add_executable(vive_rviz
		src/vive_rviz.cpp
)
target_link_libraries(vive_rviz ${PCL_LIBRARIES}   ${catkin_LIBRARIES} ${VRUI_LIBS} ${X11_LIBRARIES} ${QT_LIBRARIES} ${VTK_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} )
