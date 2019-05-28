if (BUILD_SCHUNKWSG_CAN OR BUILD_SCHUNKWSG_SIM)

	add_library(schunk_wsg SHARED
		rpi/SchunkWsgBlocks
		schunkwsgcoreLib)
	
	target_link_libraries(schunk_wsg ${RealtimeRCC_libraries})
	target_link_libraries(schunk_wsg RCCCore)
	
	# Webpages for Schunk WSG
	add_custom_target(schunk_wsg_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET schunk_wsg_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/schunkwsg/
	)
	
	if (BUILD_SCHUNKWSG_CAN)
		
		add_library(schunk_wsg_can SHARED
			driver/SchunkWsgCanDevice
			driver/IO
			schunkwsgLib)
		
		target_link_libraries(schunk_wsg_can ${RealtimeRCC_libraries})
		target_link_libraries(schunk_wsg_can RCCCore)
		target_link_libraries(schunk_wsg_can schunk_wsg)
		
	endif (BUILD_SCHUNKWSG_CAN)
		
	if (BUILD_SCHUNKWSG_SIM)
		
		add_library(schunk_wsg_sim SHARED
			simulation/SchunkWsgSimDevice
			schunkwsgsimLib
			web/web)
		
		target_link_libraries(schunk_wsg_sim ${RealtimeRCC_libraries})
		target_link_libraries(schunk_wsg_sim RCCCore)
		target_link_libraries(schunk_wsg_sim schunk_wsg)
		
	endif (BUILD_SCHUNKWSG_SIM)

endif (BUILD_SCHUNKWSG_CAN OR BUILD_SCHUNKWSG_SIM)

