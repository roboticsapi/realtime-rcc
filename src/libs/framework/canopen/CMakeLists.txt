if (BUILD_CANOPEN)

add_library(canopen SHARED
	driver/CanOpenDevice
	driver/CanOpenMaster
	CanOpen
)

target_link_libraries(canopen ${RealtimeRCC_libraries})

install(TARGETS canopen LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT canopen RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT canopen)
endif (BUILD_CANOPEN)
