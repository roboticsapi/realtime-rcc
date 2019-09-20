set(RCCCore_o ModuleBase Module Fragment TypeKit Device ExtensionLoader
	DeviceInterface
	DeviceInstanceT
	NetAST Net NetParser NetExecutor NetModules Registry RegistryImpl
	SynchronizationRule
	VirtualRCCDevice CrashDump 
	ResourceManager 
	NetExecutorLookup
	Server/HTTPHandlers Server/HTTPServer Server/mongoose 
	Server/DIOProtocol 
	Server/DirectIONet/Scanner Server/DirectIONet/Parser
	Server/DirectIONet/DIONet Server/DirectIONet/DirectIONetConverter
	Server/websocket/WebsocketStreamHandler Server/websocket/SHA1 Server/websocket/WebsocketDIO
	Server/DIOProtocolParser/Scanner Server/DIOProtocolParser/Parser
	Server/DIOProtocolParser/DIOProtocolP Server/DIOProtocolParser/DIOProtocolParser
	scheduling/SchedulingConditionP scheduling/Scanner scheduling/Parser
	scheduling/SchedulingConditionParser scheduling/SchedulingCondition
	)
set(RealtimeRCC_o RealtimeRCC)

# DirectIO grammar parser
set(DirectIODir "Server/DirectIONet")
add_custom_command(
	OUTPUT ${DirectIODir}/Scanner.cpp ${DirectIODir}/Parser.cpp
	COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/${DirectIODir}/
	COMMAND ${COCO_EXEC} -o ${CMAKE_CURRENT_BINARY_DIR}/${DirectIODir}/ -frames ${COCO_DIR} -namespace DirectIONet ${CMAKE_CURRENT_SOURCE_DIR}/${DirectIODir}/DirectIONet.atg 
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${DirectIODir}/DirectIONet.atg
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/${DirectIODir}/DirectIONet.h.in"
	"${CMAKE_CURRENT_SOURCE_DIR}/${DirectIODir}/DirectIONet.h"
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/${DirectIODir}/DIONet.hpp"
	"${CMAKE_CURRENT_BINARY_DIR}/${DirectIODir}/DIONet.hpp"
)

# DirectIOProtocol grammar parser
set(DIOPDir "Server/DIOProtocolParser")
add_custom_command(
	OUTPUT ${DIOPDir}/Scanner.cpp ${DIOPDir}/Parser.cpp
	COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/${DIOPDir}/
	COMMAND ${COCO_EXEC} -o ${CMAKE_CURRENT_BINARY_DIR}/${DIOPDir}/ -frames ${COCO_DIR} -namespace DIOProtocolP ${CMAKE_CURRENT_SOURCE_DIR}/${DIOPDir}/DIOProtocolP.atg 
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${DIOPDir}/DIOProtocolP.atg
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/${DIOPDir}/DIOProtocolP_inc.h.in"
	"${CMAKE_CURRENT_SOURCE_DIR}/${DIOPDir}/DIOProtocolP_inc.h"
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/${DIOPDir}/DIOProtocolP.hpp"
	"${CMAKE_CURRENT_BINARY_DIR}/${DIOPDir}/DIOProtocolP.hpp"
)

# Scheduling grammar parser
set(SchedulingDir "scheduling")
add_custom_command(
	OUTPUT ${SchedulingDir}/Scanner.cpp ${SchedulingDir}/Parser.cpp
	COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/${SchedulingDir}/
	COMMAND ${COCO_EXEC} -o ${CMAKE_CURRENT_BINARY_DIR}/${SchedulingDir}/ -frames ${COCO_DIR} -namespace SchedCond ${CMAKE_CURRENT_SOURCE_DIR}/${SchedulingDir}/SchedulingConditionP.atg 
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${SchedulingDir}/SchedulingConditionP.atg
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/${SchedulingDir}/SchedulingConditionP_inc.h.in"
	"${CMAKE_CURRENT_SOURCE_DIR}/${SchedulingDir}/SchedulingConditionP_inc.h"
)

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/${SchedulingDir}/SchedulingConditionP.hpp"
	"${CMAKE_CURRENT_BINARY_DIR}/${SchedulingDir}/SchedulingConditionP.hpp"
)


add_library(RCCCore SHARED ${RCCCore_o})
target_link_libraries(RCCCore ${RealtimeRCC_libraries})
add_dependencies(RCCCore configfiles)

add_executable(RealtimeRCC ${RealtimeRCC_o} ${RealtimeRCC_rc})
target_link_libraries(RealtimeRCC ${RealtimeRCC_libraries})
target_link_libraries(RealtimeRCC RCCCore)
if(BUILD_WITH_OCL)
	target_link_libraries(RealtimeRCC ${OrocosTB_libraries})
endif(BUILD_WITH_OCL)

install (TARGETS RCCCore LIBRARY DESTINATION ${INSTALL_PATH} COMPONENT core RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT core)
install (TARGETS RealtimeRCC RUNTIME DESTINATION ${INSTALL_PATH} COMPONENT core)
