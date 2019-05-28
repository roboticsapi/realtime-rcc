if (BUILD_ETHERCAT)

if(WIN32)
	set(SOEM_ARCH win32)
elseif(HAVE_REALTIME)
	set(SOEM_ARCH xenomai)
else()
	set(SOEM_ARCH linux)
endif()

add_library(soem SHARED
	SOEM/ethercatbase 
	SOEM/ethercatcoe
	SOEM/ethercatconfig
	SOEM/ethercatdc
	SOEM/ethercatfoe
	SOEM/ethercatmain
	SOEM/ethercatprint
	SOEM/ethercatsoe
	
	SOEM/oshw/${SOEM_ARCH}/nicdrv 
	SOEM/oshw/${SOEM_ARCH}/oshw 
	SOEM/osal/${SOEM_ARCH}/osal
)

include_directories(SOEM/ SOEM/oshw/${SOEM_ARCH} SOEM/osal/)

add_library(ethercat SHARED 
	driver/SOEMEthercatMaster
	
	interface/EthercatSlave 
	interface/CoEEthercatSlave
	
	ethercatLib
)

target_link_libraries(ethercat ${RealtimeRCC_libraries})
target_link_libraries(ethercat RCCCore)
target_link_libraries(ethercat soem)

install(TARGETS ethercat LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT ethercat RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT ethercat)
install(TARGETS soem LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT ethercat RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT ethercat)
set(CPACK_COMPONENT_ethercat_DEPENDS core)
endif (BUILD_ETHERCAT)
