if (BUILD_ARMKINEMATICS)
	add_library(armkinematics SHARED ArmKinematics rpi/ArmKinematicBlocks web/ArmKinematicsHandler generic/Brandstoetter2014)
	target_link_libraries(armkinematics ${RealtimeRCC_libraries})
	target_link_libraries(armkinematics RCCCore)
	
	install(TARGETS armkinematics LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT armkinematics RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT armkinematics)
	
	# Webpages for armkinematics
	add_custom_target(armkinematics_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET armkinematics_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/armkinematics/
	)
	
	install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/armkinematics COMPONENT armkinematics PATTERN ".svn" EXCLUDE)
endif (BUILD_ARMKINEMATICS)