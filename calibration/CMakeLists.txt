cmake_minimum_required(VERSION 2.8.3)

# set default build type
IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel.")
ELSE()
   SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel.")
ENDIF()

include(cmake/configure_os.cmake)

IF(IS_WINDOWS)
	include(cmake/configure_arch.cmake)
ENDIF()

SET(LIBRARY_NAME "calibration")
PROJECT("${LIBRARY_NAME}")

ADD_DEFINITIONS(-D_SOURCE_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}")
ADD_DEFINITIONS(-D_OUTPUT_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}/output")
ADD_DEFINITIONS(-D_PARAM_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}/params")

# Special settings for OpenCV integrated with OpenNI (compiled from source code)
IF(IS_LINUX)
  set(OpenCV_DIR "/opt/opencv/2.4.10/share/OpenCV/")
  find_package(OpenCV 2.4.9 REQUIRED)
ELSE()
	find_package(OpenCV REQUIRED) 
ENDIF()

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

IF(IS_LINUX)

	find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		rospy
		message_generation
	)

	generate_messages(
		DEPENDENCIES
		std_msgs
	)

	catkin_package(
		CATKIN_DEPENDS
		roscpp
		std_msgs
		rospy
	)

	include_directories(
		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		include
	)

	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h)
	target_link_libraries (${LIBRARY_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

	add_executable(${LIBRARY_NAME}_node src/${LIBRARY_NAME}_node.cpp)
	target_link_libraries(${LIBRARY_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME} VirtualKinect)
	
	install(PROGRAMS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ELSE()
	
	include_directories(
		${PCL_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		include
	)

	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h)
	target_link_libraries (${LIBRARY_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})
 
	add_executable(${LIBRARY_NAME}_node src/${LIBRARY_NAME}_node.cpp)
	target_link_libraries(${LIBRARY_NAME}_node ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME})
	target_link_libraries(${LIBRARY_NAME}_node "${CMAKE_BINARY_DIR}/../../devel/$(Configuration)/VirtualKinect.lib")	
	
	set_target_properties(${LIBRARY_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	set_target_properties(${LIBRARY_NAME}_node PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")	
ENDIF()
