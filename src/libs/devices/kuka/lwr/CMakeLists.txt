if (BUILD_KUKA_LWR_FRI OR BUILD_KUKA_LWR_SIM)
	# Core Library
	add_library(kuka_lwr SHARED Lwr 
		device/LwrDevice 
		kinematics/Lbr_Kin 
		rpi/Lbr 
		web/web
	)
	target_link_libraries(kuka_lwr ${RealtimeRCC_libraries})
	target_link_libraries(kuka_lwr RCCCore)
	install(TARGETS kuka_lwr LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.lwr RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.lwr)
	
	# Webpages for LWR
	add_custom_target(kuka_lwr_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET kuka_lwr_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/kuka/lwr/
	)
	install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/kuka/lwr COMPONENT kuka.lwr PATTERN ".svn" EXCLUDE)
endif()

# FRI Library
if (BUILD_KUKA_LWR_FRI)
	add_library(kuka_lwr_fri SHARED LwrFri 
		fri_lwr/FRI 
		fri_lwr/frisdk/friremote 
		fri_lwr/frisdk/friudp
	)
	target_link_libraries(kuka_lwr_fri ${RealtimeRCC_libraries})
	target_link_libraries(kuka_lwr_fri RCCCore kuka_lwr robotarm io)
	install(TARGETS kuka_lwr_fri LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.lwr.fri RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.lwr.fri)
endif()

# Simulaton Library
if(BUILD_KUKA_LWR_SIM)
	add_library(kuka_lwr_sim SHARED LwrSim 
		simulation/LwrSimulation
	)
	target_link_libraries(kuka_lwr_sim ${RealtimeRCC_libraries})
	target_link_libraries(kuka_lwr_sim RCCCore kuka_lwr robotarm io)
	install(TARGETS kuka_lwr_sim LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.lwr.sim RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.lwr.sim)
endif ()

if(BUILD_KUKA_IIWA_FRI OR BUILD_KUKA_IIWA_SIM)
# IIWA Library
	add_library(kuka_iiwa SHARED Iiwa 
		device/IiwaDevice 
		rpi/controller
	)
	target_link_libraries(kuka_iiwa ${RealtimeRCC_libraries})
	target_link_libraries(kuka_iiwa RCCCore robotarm armkinematics io kuka_lwr)
	install(TARGETS kuka_iiwa LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.iiwa RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.iiwa)
# FRI Library
	if(BUILD_KUKA_IIWA_FRI)
	add_library(kuka_iiwa_fri SHARED IiwaFri 
		fri_iiwa/Fri 
		fri_iiwa/frisdk/friLBRClient 
		fri_iiwa/frisdk/friLBRState
		fri_iiwa/frisdk/friLBRCommand 
		fri_iiwa/frisdk/friClientApplication 
		fri_iiwa/frisdk/friMonitoringMessageDecoder 
		fri_iiwa/frisdk/friCommandMessageEncoder 
		fri_iiwa/frisdk/pb_frimessages_callbacks
		fri_iiwa/frisdk/FRIMessages.pb.c 
		fri_iiwa/frisdk/pb_decode 
		fri_iiwa/frisdk/pb_encode
	)
	target_link_libraries(kuka_iiwa_fri ${RealtimeRCC_libraries})
	target_link_libraries(kuka_iiwa_fri RCCCore kuka_iiwa robotarm armkinematics io)
	install(TARGETS kuka_iiwa_fri LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.iiwa.fri RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.iiwa.fri)
	endif()

	if(BUILD_KUKA_IIWA_SIM)
# Simulation Library
	add_library(kuka_iiwa_sim SHARED IiwaSim 
		simulation/IiwaSimulation
	)
	target_link_libraries(kuka_iiwa_sim ${RealtimeRCC_libraries})
	target_link_libraries(kuka_iiwa_sim RCCCore kuka_iiwa robotarm io)
	install(TARGETS kuka_iiwa_sim LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.iiwa.sim RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.iiwa.sim)
	endif()
endif()
