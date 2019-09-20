if (BUILD_SCHUNK_LWA_SIM OR BUILD_SCHUNK_LWA_CAN)
	
	# All files used for real robot and simulation
	add_library(schunk_lwa SHARED
		lwa_core
		#web/web
		kinematics/Schunk_KinGeo
		)
	
	target_link_libraries(schunk_lwa ${RealtimeRCC_libraries})
	target_link_libraries(schunk_lwa RCCCore)
		
	install(TARGETS schunk_lwa LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT schunk.lwa RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT schunk.lwa)

	# Webpages for Schunk LWA
	add_custom_target(schunk_lwa_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET schunk_lwa_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/schunk_lwa/
	)
	
	# Simulation of Schunk LWA
	if(BUILD_SCHUNK_LWA_SIM)
		add_library(schunk_lwa_sim SHARED 
			lwa_sim
			simulation/lwa_simulation
			)
			
		target_link_libraries(schunk_lwa_sim ${RealtimeRCC_libraries})
		target_link_libraries(schunk_lwa_sim RCCCore schunk_lwa robotarm armkinematics)
	endif()
	
	# Schunk LWA over CANopen
	if(BUILD_SCHUNK_LWA_CAN)
		add_library(schunk_lwa_can SHARED 
			driver/SchunkLwaDriver
			util/CANopenUtil
			web/web
			lwa_can
			)
		
		target_link_libraries(schunk_lwa_can ${RealtimeRCC_libraries})
		target_link_libraries(schunk_lwa_can RCCCore schunk_lwa robotarm armkinematics)
	endif()
endif()

