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

SET(LIBRARY_NAME "RGBD_Capture")
PROJECT("${LIBRARY_NAME}")

ADD_DEFINITIONS(-D_SOURCE_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}")
ADD_DEFINITIONS(-D_OUTPUT_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}/output")
ADD_DEFINITIONS(-D_PARAM_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}/params")

# Special settings for OpenCV integrated with OpenNI (compiled from source code)
IF(IS_LINUX)
  set(OpenCV_DIR "/opt/opencv/2.4.10/share/OpenCV/")
  find_package(OpenCV 2.4.9 REQUIRED)
ELSE()
	OPTION(WINDOWS_ADDITIONAL_FEATURES "Activates some projects for the <identification> module that include the <dirent.h> header, and copy OpenNI DLLs into the executable directories." FALSE)
	find_package(OpenCV REQUIRED) 
ENDIF()

IF(WINDOWS_ADDITIONAL_FEATURES)
	SET(PCL_FOUND FALSE)
	SET(OPENNI2_INCLUDE_DIRS "C:/Program Files/OpenNI2/Include")
	SET(OPENNI2_LIBRARY "C:/Program Files/OpenNI2/Lib/OpenNI2.lib")
	include(../../identification/cmake/find_pcl.cmake)
ELSE()
	find_package(PCL REQUIRED)
ENDIF()


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

IF(IS_LINUX)

	find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		rospy
		message_generation
	)

	add_service_files(
		FILES
		SRV_Capture.srv
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

	add_library (VirtualKinect src/VirtualKinect.cpp include/VirtualKinect.h)
	target_link_libraries (VirtualKinect ${OpenCV_LIBS})
 
	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h)
	target_link_libraries (${LIBRARY_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} VirtualKinect)
 	add_dependencies(${LIBRARY_NAME} ${LIBRARY_NAME}_generate_messages)

	add_executable(capture_test src/capture_test.cpp)
	target_link_libraries(capture_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME} VirtualKinect)

	add_executable(capture_node src/capture_node.cpp)
	target_link_libraries(capture_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${LIBRARY_NAME})

	install(PROGRAMS DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
ELSE()
	
	include_directories(
		${PCL_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		include
	)

	add_library (VirtualKinect src/VirtualKinect.cpp include/VirtualKinect.h)
	target_link_libraries (VirtualKinect ${OpenCV_LIBS})
 
	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h)
	target_link_libraries (${LIBRARY_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} VirtualKinect)
 
	add_executable(capture_test src/capture_test.cpp)
	target_link_libraries(capture_test ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME} VirtualKinect)
	
	set_target_properties(${LIBRARY_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	set_target_properties(VirtualKinect PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	set_target_properties(capture_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	
	IF (WINDOWS_ADDITIONAL_FEATURES)
		add_executable(capture_process src/captureProcess.cpp)
		target_link_libraries(capture_process ${OpenCV_LIBS} ${LIBRARY_NAME}) 
		set_target_properties(capture_process PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
		
		MESSAGE(STATUS "OPENNI2_LIBRARY = ${OPENNI2_LIBRARY}; OPENNI2_INCLUDE_DIRS = ${OPENNI2_INCLUDE_DIRS}")
		include_directories(${OPENNI2_INCLUDE_DIRS})
		link_directories("${OPENNI2_LIBRARY}/..")
		
		SET(EXECUTABLE_NAME "capture_id")
		add_executable(${EXECUTABLE_NAME} src/${EXECUTABLE_NAME}.cpp)
		target_link_libraries(${EXECUTABLE_NAME} ${OpenCV_LIBS} ${LIBRARY_NAME} ${OPENNI2_LIBRARY})
		set_target_properties(${EXECUTABLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
		include("${CMAKE_CURRENT_SOURCE_DIR}/../../identification/cmake/link_openni_dlls.cmake")
	ENDIF()
	
ENDIF()
