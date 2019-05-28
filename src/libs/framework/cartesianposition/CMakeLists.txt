if (BUILD_CARTESIANPOSITION)

add_library(cartesianposition SHARED 
	CartesianPosition 
	web/CartesianPositionHandler
	device/CartesianPositionDevice
)

target_link_libraries(cartesianposition ${RealtimeRCC_libraries})
target_link_libraries(cartesianposition RCCCore)

install(TARGETS cartesianposition LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT robotbase RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT cartesianposition)
	# Webpages for cartesianposition
	add_custom_target(cartesianposition_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET cartesianposition_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/cartesianposition/
	)
	
	install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/cartesianposition COMPONENT cartesianposition PATTERN ".svn" EXCLUDE)
endif (BUILD_CARTESIANPOSITION)