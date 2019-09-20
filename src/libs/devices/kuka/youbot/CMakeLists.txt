if (BUILD_KUKA_YOUBOT_SIM OR BUILD_KUKA_YOUBOT_EC)

add_library(kuka_youbot SHARED
	 
	web/web
	rpi/youBotBlocks
	
	kinematics/yb_Kin
	kinematics/yb_KindynChain
	kinematics/base_Kin
	kinematics/PositionPreservingProjector	
	kinematics/PalletizingProjector
	
	youBot
	
)

target_link_libraries(kuka_youbot RCCCore)
target_link_libraries(kuka_youbot robotbase)
target_link_libraries(kuka_youbot robotarm)
target_link_libraries(kuka_youbot ${RealtimeRCC_libraries})
install(TARGETS kuka_youbot LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.youbot RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.youbot)



if (BUILD_KUKA_YOUBOT_SIM)
add_library(kuka_youbot_sim SHARED
		
	simulation/youBotBase
	simulation/youBotArm
	simulation/youBotGripper
	
	youBotSimulation
	 
)

target_link_libraries(kuka_youbot_sim RCCCore)
target_link_libraries(kuka_youbot_sim kuka_youbot)
target_link_libraries(kuka_youbot_sim robotbase)
target_link_libraries(kuka_youbot_sim robotarm)
target_link_libraries(kuka_youbot_sim ${RealtimeRCC_libraries})

install(TARGETS kuka_youbot_sim LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.youbot.sim RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.youbot.sim)
endif (BUILD_KUKA_YOUBOT_SIM)


if (BUILD_KUKA_YOUBOT_EC)
add_library(kuka_youbot_ec SHARED

	ethercat/EthercatAxis 
	ethercat/EthercatMaster 
	ethercat/EthercatSlave 
	ethercat/youBotArmEthercat
	ethercat/youBotBaseEthercat
	ethercat/youBotGripperEthercat
	ethercat/youBotArmJointImpedanceController
	ethercat/youBotArmPositionController
	
	youBotEthercat
	
)

target_link_libraries(kuka_youbot_ec RCCCore)
target_link_libraries(kuka_youbot_ec kuka_youbot)
target_link_libraries(kuka_youbot_ec robotbase)
target_link_libraries(kuka_youbot_ec robotarm)
target_link_libraries(kuka_youbot_ec ${RealtimeRCC_libraries})

install(TARGETS kuka_youbot_ec LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT kuka.youbot.ec RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT kuka.youbot.ec)

endif (BUILD_KUKA_YOUBOT_EC)


# Webpages for youBot
add_custom_target(kuka_youbot_web ALL DEPENDS configfiles)

add_custom_command(
	TARGET kuka_youbot_web
	POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ ${webdir}/xsl/kuka/youbot/
)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web/xsl/ DESTINATION ${INSTALL_PATH}/web/xsl/kuka/youbot COMPONENT kuka.youbot PATTERN ".svn" EXCLUDE)

endif (BUILD_KUKA_YOUBOT_SIM OR BUILD_KUKA_YOUBOT_EC)
