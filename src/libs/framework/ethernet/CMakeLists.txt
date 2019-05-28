if (BUILD_ETHERNET_STANDARD)
	
	# All files used for real robot and simulation
	add_library(ethernet_standard SHARED
			ethernet_standard
			standard/EthernetServerUDPstandard
		)
	
	target_link_libraries(ethernet_standard ${RealtimeRCC_libraries})
	target_link_libraries(ethernet_standard RCCCore)
		
	install(TARGETS ethernet_standard LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT ethernet.standard RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT ethernet.standard)
endif ()

if (BUILD_ETHERNET_RTNET)
	
	# All files used for real robot and simulation
	add_library(ethernet_rtnet SHARED
			ethernet_rtnet
			rtnet/EthernetServerUDPrtnet
		)
	
	target_link_libraries(ethernet_rtnet ${RealtimeRCC_libraries})
	target_link_libraries(ethernet_rtnet RCCCore)
		
	install(TARGETS ethernet_rtnet LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT ethernet.rtnet RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT ethernet.rtnet)
endif ()
