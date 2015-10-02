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

SET(LIBRARY_NAME "registration")
PROJECT("${LIBRARY_NAME}")

ADD_DEFINITIONS(-D_SOURCE_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}")
ADD_DEFINITIONS(-D_OUTPUT_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}/output")
ADD_DEFINITIONS(-D_PARAM_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}/params")
ADD_DEFINITIONS(-D_LIB3D_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}/lib3d")

# Special settings for OpenCV integrated with OpenNI (compiled from source code)
IF(IS_LINUX)
  set(CMAKE_CXX_FLAGS -std=c++0x)
  set(OpenCV_DIR "/opt/opencv/2.4.10/share/OpenCV/")
  find_package(OpenCV 2.4 REQUIRED)
ELSE()
  find_package(OpenCV REQUIRED) 
ENDIF()

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost COMPONENTS system filesystem REQUIRED)

IF(IS_LINUX)

	find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		rospy
		message_generation
	)

	add_service_files(
		FILES
		SRV_Registration.srv
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
		${Boost_INCLUDE_DIR}
		include
	)

	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h)
	target_link_libraries (${LIBRARY_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} boost_filesystem boost_system)
 	add_dependencies(${LIBRARY_NAME} ${LIBRARY_NAME}_generate_messages)

	add_executable(${LIBRARY_NAME}_test src/${LIBRARY_NAME}_test.cpp )
	target_link_libraries(${LIBRARY_NAME}_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME} )
	target_link_libraries(${LIBRARY_NAME}_test RGBD_Capture VirtualKinect boost_filesystem boost_system)

	add_executable(${LIBRARY_NAME}_node src/${LIBRARY_NAME}_node.cpp)
	target_link_libraries(${LIBRARY_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME})

	install(PROGRAMS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ELSE()
	include_directories(
		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
		include
	)

	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h)
	target_link_libraries (${LIBRARY_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})
 
	add_executable(${LIBRARY_NAME}_test src/${LIBRARY_NAME}_test.cpp )	
	target_link_libraries(${LIBRARY_NAME}_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME})
	target_link_libraries(${LIBRARY_NAME}_test "${CMAKE_BINARY_DIR}/../../devel/$(Configuration)/RGBD_Capture.lib")
	target_link_libraries(${LIBRARY_NAME}_test "${CMAKE_BINARY_DIR}/../../devel/$(Configuration)/VirtualKinect.lib")

	add_executable(${LIBRARY_NAME}_create_lib2d src/${LIBRARY_NAME}_create_lib2d.cpp )
	target_link_libraries(${LIBRARY_NAME}_create_lib2d ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME})
	target_link_libraries(${LIBRARY_NAME}_create_lib2d "${CMAKE_BINARY_DIR}/../../devel/$(Configuration)/RGBD_Capture.lib")
	target_link_libraries(${LIBRARY_NAME}_create_lib2d "${CMAKE_BINARY_DIR}/../../devel/$(Configuration)/VirtualKinect.lib")
	
	set_target_properties(${LIBRARY_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	set_target_properties(${LIBRARY_NAME}_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	set_target_properties(${LIBRARY_NAME}_create_lib2d PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
ENDIF()
