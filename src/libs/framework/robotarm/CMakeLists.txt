if (BUILD_ROBOTARM)
add_library(robotarm SHARED 
	device/CyclicPositionRobotArm 
	simulation/RobotArmDriverSimulation 
	web/RobotArmHandler
	RobotArm.cpp
)
target_link_libraries(robotarm ${RealtimeRCC_libraries})
target_link_libraries(robotarm RCCCore)

install(TARGETS robotarm LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT robotarm RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT robotarm)

	# Webpages for robotarm
	add_custom_target(robotarm_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET robotarm_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/robotarm/
	)
	
	install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/robotarm COMPONENT robotarm PATTERN ".svn" EXCLUDE)
endif (BUILD_ROBOTARM)