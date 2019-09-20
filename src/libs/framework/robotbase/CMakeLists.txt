if (BUILD_ROBOTBASE)

add_library(robotbase SHARED 
	RobotBase 
	simulation/RobotBaseDriverSimulation
	device/RobotBaseSim 
	web/RobotBaseHandler
)

target_link_libraries(robotbase ${RealtimeRCC_libraries})
target_link_libraries(robotbase RCCCore cartesianposition)

install(TARGETS robotbase LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT robotbase RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT robotbase)
	# Webpages for robotbase
	add_custom_target(robotbase_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET robotbase_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/robotbase/
	)
	
	install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/robotbase COMPONENT robotbase PATTERN ".svn" EXCLUDE)
endif (BUILD_ROBOTBASE)