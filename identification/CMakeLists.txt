cmake_minimum_required(VERSION 2.8.3)

include(cmake/configure_os.cmake)

IF(IS_WINDOWS)
	include(cmake/configure_arch.cmake)
	OPTION(WINDOWS_SET_TARGET_OUTPUT_PATH "Set the generated libs and exes to the folder 'devel'." FALSE)
ENDIF()

SET(LIBRARY_NAME "identification")
PROJECT("${LIBRARY_NAME}")

ADD_DEFINITIONS(-D_SOURCE_DIRECTORY_="${CMAKE_CURRENT_SOURCE_DIR}")

SET(EXTERNAL_LIBRARIES "")
LIST(APPEND EXTERNAL_LIBRARIES "opencv")

foreach(EXTERNAL_LIB ${EXTERNAL_LIBRARIES}) 
	include(cmake/find_${EXTERNAL_LIB}.cmake)
endforeach(EXTERNAL_LIB)

IF(IS_LINUX)

	find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		rospy
		message_generation
	)

	add_service_files(
		FILES
		SRV_Identification.srv
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
		include
	)

	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h src/tools.cpp include/tools.h)
	target_link_libraries (${LIBRARY_NAME} ${OpenCV_LIBS})
 	add_dependencies(${LIBRARY_NAME} ${LIBRARY_NAME}_generate_messages)

	add_executable(${LIBRARY_NAME}_library_create src/${LIBRARY_NAME}_library_create.cpp)
	target_link_libraries(${LIBRARY_NAME}_library_create  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${LIBRARY_NAME})

	add_executable(${LIBRARY_NAME}_library_test src/${LIBRARY_NAME}_library_test.cpp)
	target_link_libraries(${LIBRARY_NAME}_library_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${LIBRARY_NAME})

	add_executable(${LIBRARY_NAME}_example src/${LIBRARY_NAME}_example.cpp)
	target_link_libraries(${LIBRARY_NAME}_example  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${LIBRARY_NAME})

	add_executable(${LIBRARY_NAME}_node src/${LIBRARY_NAME}_node.cpp)
	target_link_libraries(${LIBRARY_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${LIBRARY_NAME})

	install(PROGRAMS ./scripts/ros_testing_client.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
	
ELSE()
	include_directories(include)
	
	add_library (${LIBRARY_NAME} src/${LIBRARY_NAME}.cpp include/${LIBRARY_NAME}.h src/tools.cpp include/tools.h)
	target_link_libraries (${LIBRARY_NAME} ${OpenCV_LIBS})

	IF(WINDOWS_SET_TARGET_OUTPUT_PATH)
		set_target_properties(${LIBRARY_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	ENDIF()
	
	IF(pcl_FOUND AND USE_PCL)
	# SYNTHETIC DEPTH IMAGE CREATION PROGRAM
		SET(EXECUTABLE_NAME "depth_from_object")
		add_executable (${EXECUTABLE_NAME} src/${EXECUTABLE_NAME}.cpp)

		foreach(EXTERNAL_LIB ${EXTERNAL_LIBRARIES}) 
			IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake")
				include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake)
			ENDIF()
		endforeach(EXTERNAL_LIB)

		target_link_libraries (${EXECUTABLE_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${LIBRARY_NAME})
		
		IF(WINDOWS_SET_TARGET_OUTPUT_PATH)
			set_target_properties(${EXECUTABLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
		ENDIF()
	
	ENDIF()

	# IDENTIFICATION YML LIBRARY CREATION PROGRAM
	SET(EXECUTABLE_NAME "${LIBRARY_NAME}_library_create")
	add_executable (${EXECUTABLE_NAME} src/${EXECUTABLE_NAME}.cpp)

	foreach(EXTERNAL_LIB ${EXTERNAL_LIBRARIES}) 
		IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake")
			include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake)
		ENDIF()
	endforeach(EXTERNAL_LIB)

	target_link_libraries (${EXECUTABLE_NAME} ${OpenCV_LIBS} ${LIBRARY_NAME})

	IF(WINDOWS_SET_TARGET_OUTPUT_PATH)
		set_target_properties(${EXECUTABLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	ENDIF()
	
	# IDENTIFICATION YML LIBRARY TESTING PROGRAM
	SET(EXECUTABLE_NAME "${LIBRARY_NAME}_library_test")
	add_executable (${EXECUTABLE_NAME} src/${EXECUTABLE_NAME}.cpp)

	foreach(EXTERNAL_LIB ${EXTERNAL_LIBRARIES}) 
		IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake")
			include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake)
		ENDIF()
	endforeach(EXTERNAL_LIB)

	target_link_libraries (${EXECUTABLE_NAME} ${OpenCV_LIBS} ${LIBRARY_NAME})

	IF(WINDOWS_SET_TARGET_OUTPUT_PATH)
		set_target_properties(${EXECUTABLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	ENDIF()
	
	# IDENTIFICATION EXAMPLE PROGRAM
	SET(EXECUTABLE_NAME "${LIBRARY_NAME}_example")
	add_executable (${EXECUTABLE_NAME} src/${EXECUTABLE_NAME}.cpp)

	foreach(EXTERNAL_LIB ${EXTERNAL_LIBRARIES}) 
		IF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake")
			include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/link_${EXTERNAL_LIB}_dlls.cmake)
		ENDIF()
	endforeach(EXTERNAL_LIB)

	target_link_libraries (${EXECUTABLE_NAME} ${OpenCV_LIBS} ${LIBRARY_NAME})
	
	IF(WINDOWS_SET_TARGET_OUTPUT_PATH)
		set_target_properties(${EXECUTABLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../../devel")
	ENDIF()
	
ENDIF()
