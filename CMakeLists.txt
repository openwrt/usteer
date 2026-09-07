cmake_minimum_required(VERSION 3.10)
INCLUDE (CheckIncludeFiles)
INCLUDE(FindPkgConfig)

PROJECT(usteerd C)

IF("${CMAKE_SYSTEM_NAME}" MATCHES "Linux" AND NOT NL_CFLAGS)
  FIND_PROGRAM(PKG_CONFIG pkg-config)
  IF(PKG_CONFIG)
    EXECUTE_PROCESS(
		COMMAND pkg-config --silence-errors --cflags libnl-tiny
		OUTPUT_VARIABLE NL_CFLAGS
		OUTPUT_STRIP_TRAILING_WHITESPACE)
    EXECUTE_PROCESS(
		COMMAND pkg-config --silence-errors --libs libnl-tiny
		OUTPUT_VARIABLE NL_LIBS
		OUTPUT_STRIP_TRAILING_WHITESPACE)
  ENDIF()
ENDIF()

CHECK_INCLUDE_FILES(pcap/pcap.h HAVE_PCAP_H)
IF(NOT HAVE_PCAP_H)
	UNSET(HAVE_PCAP_H CACHE)
	MESSAGE(FATAL_ERROR "pcap/pcap.h is not found")
ENDIF()

SET(SOURCES main.c local_node.c node.c sta.c policy.c ubus.c remote.c remote-message.c parse.c netifd.c timeout.c event.c measurement.c band_steering.c)

IF(NL_CFLAGS)
	ADD_DEFINITIONS(${NL_CFLAGS})
	SET(SOURCES ${SOURCES} nl80211.c)
ENDIF()

ADD_DEFINITIONS(-Os -Wall -Werror --std=gnu99 -g3 -Wmissing-declarations)

FIND_LIBRARY(libjson NAMES json-c json)
ADD_EXECUTABLE(usteerd ${SOURCES})
ADD_EXECUTABLE(fakeap fakeap.c timeout.c)

TARGET_LINK_LIBRARIES(usteerd ubox ubus blobmsg_json
			${LIBS_EXTRA} ${libjson} ${NL_LIBS})
TARGET_LINK_LIBRARIES(fakeap ubox ubus)

ADD_EXECUTABLE(ap-monitor monitor.c parse.c)
TARGET_LINK_LIBRARIES(ap-monitor ubox pcap blobmsg_json)

OPTION(BUILD_TESTING "Build remote message regression tests" OFF)
IF(BUILD_TESTING)
  ENABLE_TESTING()
  ADD_EXECUTABLE(test-remote-message tests/remote-message.c remote-message.c parse.c)
  TARGET_INCLUDE_DIRECTORIES(test-remote-message PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  TARGET_COMPILE_OPTIONS(test-remote-message PRIVATE -UNDEBUG)
  TARGET_LINK_LIBRARIES(test-remote-message ubox)
  ADD_TEST(NAME remote-message COMMAND test-remote-message)
  ADD_EXECUTABLE(test-remote-socket tests/remote-socket.c remote-message.c)
  TARGET_INCLUDE_DIRECTORIES(test-remote-socket PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  TARGET_COMPILE_OPTIONS(test-remote-socket PRIVATE -UNDEBUG -ffunction-sections -fdata-sections)
  SET_TARGET_PROPERTIES(test-remote-socket PROPERTIES LINK_FLAGS "-Wl,--gc-sections")
  TARGET_LINK_LIBRARIES(test-remote-socket ubox)
  ADD_TEST(NAME remote-socket COMMAND test-remote-socket)
ENDIF()

SET(CMAKE_INSTALL_PREFIX /usr)

INSTALL(TARGETS usteerd
	RUNTIME DESTINATION sbin
)
