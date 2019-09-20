if (BUILD_RTSOCKETCAN)

add_library(rtsocketcan SHARED
	driver/RtSocketCanDevice
	rtCanLib
	)

target_link_libraries(rtsocketcan ${RealtimeRCC_libraries})
target_link_libraries(rtsocketcan RCCCore)

# Todo: detect xenomai?
include_directories("/usr/xenomai/include")

install(TARGETS rtsocketcan LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT rtsocketcan RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT rtsocketcan)

endif (BUILD_RTSOCKETCAN)
