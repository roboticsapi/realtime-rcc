if (BUILD_RPICORE)

add_library(rpicore SHARED 
	rpicore 
	rpi/OTG
	rpi/OTG2
	rpi/ValueCoding
)

target_link_libraries(rpicore ${RealtimeRCC_module_libraries})
target_link_libraries(rpicore RCCCore)

install(TARGETS rpicore LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT core RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT core)

endif (BUILD_RPICORE)