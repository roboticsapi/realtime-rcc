if (BUILD_BECKHOFF_IO)
	
	# All files used for real robot and simulation
	add_library(beckhoffio SHARED
		device/AbstractBeckhoffIODevice
		device/DummyDevice
		device/EL1008
		device/EL2008
		device/EL2024
		device/EL4004
		device/EL3064
		beckhoffio
		)
	
	target_link_libraries(beckhoffio ${RealtimeRCC_libraries})
	target_link_libraries(beckhoffio RCCCore io ethercat)
		
endif ()
