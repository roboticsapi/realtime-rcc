if (BUILD_KUKA_KR_SIM OR BUILD_KUKA_KR_RSI)
	
	# All files used for real robot and simulation
	add_library(kuka_kr SHARED
		kuka_kr
		device/KR_Robot_Device
		device/KR_Arm_Device
		device/KR_ExtAxis_Device
		device/KR_Controller
		kinematics/KR_Kin
		)
	
	target_link_libraries(kuka_kr ${RealtimeRCC_libraries})
	target_link_libraries(kuka_kr RCCCore)
		
	install(TARGETS kuka_kr LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.kr RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.kr)

	# Webpages for KUKA
	add_custom_target(kuka_kr_web ALL DEPENDS configfiles)
	add_custom_command(
		TARGET kuka_kr_web
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/kuka_kr/
	)
	#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/kuka_kr COMPONENT kuka.kr PATTERN ".svn" EXCLUDE)
	
	# Simulation of KUKA Robot
	if(BUILD_KUKA_KR_SIM)
		add_library(kuka_kr_sim SHARED 
			kuka_kr_sim
			simulation/KR_Controller_Sim
			simulation/KR_Arm_Sim
			)
			
		target_link_libraries(kuka_kr_sim ${RealtimeRCC_libraries})
		target_link_libraries(kuka_kr_sim RCCCore kuka_kr robotarm armkinematics io)
			
		install(TARGETS kuka_kr_sim LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.kr.sim RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.kr.sim)
	endif()
	
	if(BUILD_KUKA_KR_RSI)
		add_library(kuka_kr_rsi SHARED 
			kuka_kr_rsi
			rsi/KR_Controller_RSI
			rsi/KR_Arm_RSI
			)
			
		target_link_libraries(kuka_kr_rsi ${RealtimeRCC_libraries})
		target_link_libraries(kuka_kr_rsi RCCCore kuka_kr robotarm armkinematics io)
			
		install(TARGETS kuka_kr_rsi LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.kr.rsi RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.kr.rsi)
	endif()
	
endif ()

