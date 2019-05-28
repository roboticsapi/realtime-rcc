if (BUILD_SOCKETCAN)

add_library(socketcan SHARED
	driver/SocketCanDevice
	canLib
	)

target_link_libraries(socketcan ${RealtimeRCC_libraries})
target_link_libraries(socketcan RCCCore)

install(TARGETS socketcan LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT socketcan RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT socketcan)

endif (BUILD_SOCKETCAN)
