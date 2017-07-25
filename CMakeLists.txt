cmake_minimum_required(VERSION 2.8)
project(vive_kin)

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


find_package( OpenMP REQUIRED)
find_package( OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
#find_package(PCL  REQUIRED)
find_package(PCL 1.8 REQUIRED)
set (VRUI_INCLUDE_DIR /usr/local/include/Vrui-4.2)
find_package(X11 REQUIRED)
#set (VRUI_LIBS /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libVrui.g++-3.so /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/libGeometry.g++-3.so )
file(GLOB VRUI_LIBS  /usr/local/lib/x86_64-linux-gnu/Vrui-4.2/*.so )



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



catkin_package()


add_executable(vive_kin
		src/vive_kin.cpp
)
target_link_libraries(vive_kin ${PCL_LIBRARIES}   ${catkin_LIBRARIES} ${VRUI_LIBS} ${X11_LIBRARIES} ${QT_LIBRARIES} ${VTK_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${OpenCV_LIBS} )
