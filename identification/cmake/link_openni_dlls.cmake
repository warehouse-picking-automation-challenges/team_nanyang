SET(OPENNI_DLL "${OPENNI2_LIBRARY}/../../Redist/OpenNI2.${DLL_EXT}")
IF(EXISTS "${OPENNI_DLL}")
	add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENNI_DLL}" $<TARGET_FILE_DIR:${EXECUTABLE_NAME}>)
ENDIF()

SET(OPENNI_DLL_DIR "${OPENNI2_LIBRARY}/../../Redist/OpenNI2")
IF(EXISTS "${OPENNI_DLL_DIR}")
	LIST(APPEND OpenNI_INIS_List "PS1080" "PSLink")
	LIST(APPEND OpenNI_DLLS_List "Kinect" "OniFile" ${OpenNI_INIS_List})

	foreach(ARCH "Debug" "Release")
		file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${ARCH}/OpenNI2/Drivers")
		
		foreach(OpenNI_DLL ${OpenNI_DLLS_List}) 
			add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENNI_DLL_DIR}/Drivers/${OpenNI_DLL}.${DLL_EXT}" "${CMAKE_CURRENT_BINARY_DIR}/${ARCH}/OpenNI2/Drivers/${OpenNI_DLL}.${DLL_EXT}")
		endforeach(OpenNI_DLL) 
		
		foreach(OpenNI_PDB ${OpenNI_DLLS_List}) 
			add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENNI_DLL_DIR}/Drivers/${OpenNI_PDB}.pdb" "${CMAKE_CURRENT_BINARY_DIR}/${ARCH}/OpenNI2/Drivers/${OpenNI_PDB}.pdb")
		endforeach(OpenNI_PDB)

		foreach(OpenNI_INI ${OpenNI_INIS_List}) 
			add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENNI_DLL_DIR}/Drivers/${OpenNI_INI}.pdb" "${CMAKE_CURRENT_BINARY_DIR}/${ARCH}/OpenNI2/Drivers/${OpenNI_INI}.ini")
		endforeach(OpenNI_INI)		
	endforeach(ARCH)
ENDIF()