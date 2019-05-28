cmake_minimum_required(VERSION 2.8)

PROJECT ( RealtimeRCC CXX C )

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/config")

set(PROJ_SOURCE_DIR ${PROJECT_SOURCE_DIR})

# Version number for RCC
set(RCC_VERSION_MAJOR 1)
set(RCC_VERSION_MINOR 0)

include(src/CMakeIncludes.txt)

# Download prIDE
set(BUILD_PRIDE OFF CACHE BOOL "Download and install prIDE")

###############################################################################################
# end of configuration properties
###############################################################################################

# Only enable on Linux, Visual Studio 11 enables C++11 by default
if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

set(INSTALL_PATH RealtimeRCC CACHE STRING "Path below CMAKE_INSTALL_PREFIX to install RealtimeRCC")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${INSTALL_PATH}")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

message(STATUS "Running on system: ${CMAKE_SYSTEM_NAME} on architecture: ${CMAKE_SYSTEM_PROCESSOR}")

set(OROCOS_INSTALL "/usr/local" CACHE PATH "RTT installation path")
	
# detect whether we are running on 64bit Linux or not
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
	set(LINUX_HAVE_64BIT TRUE)
	# we cannot build reflexxes at the moment on 64bit linux
	set(BUILD_REFLEXXES OFF)
	add_definitions(-fPIC)
	message(STATUS "Running on 64bit Linux")
else()
	set(LINUX_HAVE_64BIT FALSE)
	message(STATUS "Running on 32bit Linux")
endif()

set(COCO_DIR "${PROJECT_SOURCE_DIR}/libs/Coco")
set(COCO_EXEC "${COCO_DIR}/Coco-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")

find_package(Boost REQUIRED COMPONENTS filesystem regex system)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(OROCOS-RTT)
include_directories(${OROCOS-RTT_INCLUDE_DIRS})
link_directories(${OrocosRTT_LIBRARY_DIRS})

set(Project_WC_REVISION "0")

set(RCC_VERSION_PATCH ${Project_WC_REVISION})

configure_file(
	"${PROJECT_SOURCE_DIR}/src/rcc/version_svn.h.in"
	"${PROJECT_SOURCE_DIR}/src/rcc/version_svn.h"
)

configure_file(
	"${PROJECT_SOURCE_DIR}/web/index.htm.in"
	"${PROJECT_SOURCE_DIR}/web/index.htm"
)

# TODO FIND EIGEN
find_path(EIGEN_INCLUDE Eigen/Core PATHS /usr/include/eigen3/)
include_directories(${EIGEN_INCLUDE})

set(RealtimeRCC_libraries ${RealtimeRCC_libraries} ${Boost_LIBRARIES} orocos-kdl orocos-rtt-${OROCOS_TARGET})
set(RealtimeRCC_module_libraries ${RealtimeRCC_libraries})

add_definitions(-DOROCOS_TARGET=${OROCOS_TARGET})
add_definitions(-DBOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS)

set(HAVE_REALTIME FALSE)

if (${OROCOS_TARGET} STREQUAL "lxrt")
	set(HAVE_REALTIME TRUE)
	set(BUILD_WITH_RTNET ON CACHE BOOL "Use RT-net sockets")
endif ()

if (${OROCOS_TARGET} STREQUAL "xenomai")
	set(HAVE_REALTIME TRUE)
	set(BUILD_WITH_RTNET ON CACHE BOOL "Use RT-net sockets")
	set(RealtimeRCC_libraries ${RealtimeRCC_libraries} rtdm)
	link_directories("/usr/xenomai/lib")
endif ()

if(HAVE_REALTIME)
	add_definitions(-DREALTIME)
endif ()

if(BUILD_WITH_RTNET AND HAVE_REALTIME)
	add_definitions(-DRTNET)
	find_path(rtnet-inc rtnet.h /usr/local/rtnet/include)
	include_directories(${rtnet-inc})
endif ()

set(md5file "${PROJECT_SOURCE_DIR}/src/rcc/version_bin_md5.inc")

if(WIN32)
	set(makemd5 "makemd5.cmd")
else(WIN32)
	set(makemd5 "./makemd5.sh")
endif(WIN32)

add_custom_command(OUTPUT ${md5file}
	COMMAND ${makemd5}
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/rcc/
	DEPENDS ${PROJECT_SOURCE_DIR}/src/rcc/*.hpp)

add_custom_target(configfiles ALL DEPENDS ${md5file})
#add_custom_target(configfiles ALL)

if (NOT ${PROJECT_SOURCE_DIR} EQUAL ${PROJECT_BINARY_DIR})
	set(webdir ${PROJECT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/web)

	add_custom_command(
		TARGET configfiles
		POST_BUILD

		# copy conf.d directory		
		COMMAND ${CMAKE_COMMAND} -E remove_directory ${PROJECT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/conf.d
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/conf.d/ ${PROJECT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/conf.d
		
		# copy web directory
		COMMAND ${CMAKE_COMMAND} -E remove_directory ${webdir}
		COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/web ${webdir}
		COMMAND ${CMAKE_COMMAND} -E remove_directory ${webdir}/.svn
	)
	
	if(BUILD_PRIDE)
		add_custom_command(
			TARGET configfiles
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E remove_directory ${webdir}/pride
			COMMAND wget -q https://download.roboticsapi.org/upload/pride.zip -O pride.zip
			COMMAND unzip -q pride.zip -d ${webdir}/pride/
		)
		
		install(DIRECTORY ${webdir}/pride DESTINATION ${INSTALL_PATH}/web/pride COMPONENT pride PATTERN ".svn" EXCLUDE)
		set(CPACK_COMPONENT_pride_DEPENDS core)
		
	endif()
endif ()

# enable <rtt/...> and <libs/...> #include
include_directories(${PROJECT_SOURCE_DIR}/src)

add_subdirectory(src)

# Use configuration files for release
#install (FILES conf.d/release/release.extensions.xml DESTINATION ${INSTALL_PATH}/conf.d RENAME extensions.xml COMPONENT core)
#install (FILES conf.d/release/release.devices.xml DESTINATION ${INSTALL_PATH}/conf.d RENAME devices.xml COMPONENT core)

# Include generated web directory (including extension files and pride, if activated)
install (DIRECTORY ${PROJECT_SOURCE_DIR}/web DESTINATION ${INSTALL_PATH} COMPONENT core PATTERN ".svn" EXCLUDE)

set(CPACK_SET_DESTDIR Off)
set(CPACK_PACKAGE_NAME "RealtimeRCC-${OROCOS_TARGET}")
set(CPACK_PACKAGE_CONTACT "RoboticsAPI Team <info@roboticsapi.org>")
set(CPACK_PACKAGE_VENDOR "ISSE, Augsburg University")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Robot Control Core")
set(CPACK_PACKAGE_VERSION_MAJOR ${RCC_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${RCC_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${RCC_VERSION_PATCH})

set(CPACK_PACKAGE_INSTALL_DIRECTORY "RealtimeRCC")
set(CPACK_PACKAGE_EXECUTABLES "RealtimeRCC;Realtime Robot Contorl Core")

# add custom _real for component packaging bugfix
set(CPACK_DEBIAN_PACKAGE_DEPENDS_REAL "liborocos-rtt-gnulinux2.6, liborocos-kdl, realtimercc-core") #, libboost-filesystem >= ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}, libboost-regex >= ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE http://www.roboticsapi.org/)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS FALSE)
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEB_COMPONENT_INSTALL ON)

set(CPACK_NSIS_MENU_LINKS "http://www.roboticsapi.org" "RoboticsAPI Website")
set(CPACK_NSIS_MUI_ICON "${PROJECT_SOURCE_DIR}/src/rcc/RealtimeRCC.ico")
set(CPACK_NSIS_EXECUTABLES_DIRECTORY "${INSTALL_PATH}")

# Ignore groups on linux
set(CPACK_COMPONENTS_IGNORE_GROUPS ON)
if(LINUX_HAVE_64BIT)
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
else(LINUX_HAVE_64BIT)
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
endif(LINUX_HAVE_64BIT)

include(CPack)
