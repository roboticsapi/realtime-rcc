if (BUILD_IO)
add_library(io SHARED
	io 
	web/IOWeb
	rpi/IO.cpp 
	simulation/IODeviceSimulation)
target_link_libraries(io ${RealtimeRCC_libraries})
target_link_libraries(io RCCCore)

	# Webpages for IO
	add_custom_target(io_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET io_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/io/
	)

install(TARGETS io LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT io RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT io)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/io/ COMPONENT io PATTERN ".svn" EXCLUDE)
endif (BUILD_IO)