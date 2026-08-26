cmake_minimum_required(VERSION 3.16)

add_executable(fujibuspacket_tests
    FujiBusPacketTests.cpp
    ${CMAKE_SOURCE_DIR}/lib/bus/rs232/FujiBusPacket.cpp
)

target_include_directories(fujibuspacket_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include/      # for debug.h, etc.
    ${CMAKE_SOURCE_DIR}/lib/
    ${CMAKE_SOURCE_DIR}/lib/bus/
    ${CMAKE_SOURCE_DIR}/components_pc/
)

target_link_libraries(fujibuspacket_tests PRIVATE
)

add_test(NAME fujibuspacket_tests COMMAND fujibuspacket_tests)

# ------------------------------------------------------------------------------
# Policy check: no platform-specific BUILD_* ifdefs in the shared device bases.
# Customization there must go through a subclass override, never a #ifdef.
# This is a script, not a compiled test, so it's wired in directly via
# add_test rather than as a doctest executable.
# ------------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_test(
    NAME no_build_ifdefs_in_fujidevice
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/check_no_build_ifdefs.py
            ${CMAKE_SOURCE_DIR}/lib/device/fujiDevice
            ${CMAKE_SOURCE_DIR}/lib/device/fujiClock
)

if(FUJINET_TARGET STREQUAL "ATARI")
  add_executable(sio_dstats_tests
    sioNetwork-DSTATS-test.cpp
    ${CMAKE_SOURCE_DIR}/lib/device/NDevice/NDevice.cpp
    ${CMAKE_SOURCE_DIR}/lib/device/sio/sioNetwork.cpp
    )
  target_include_directories(sio_dstats_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include/
    ${CMAKE_SOURCE_DIR}/lib/
    ${CMAKE_SOURCE_DIR}/lib/bus/
    ${CMAKE_SOURCE_DIR}/lib/sam/
    ${CMAKE_SOURCE_DIR}/lib/device/
    ${CMAKE_SOURCE_DIR}/lib/config/
    ${CMAKE_SOURCE_DIR}/lib/printer-emulator/
    ${CMAKE_SOURCE_DIR}/lib/FileSystem/
    ${CMAKE_SOURCE_DIR}/lib/encoding/
    ${CMAKE_SOURCE_DIR}/lib/media/
    ${CMAKE_SOURCE_DIR}/lib/fuji/
    ${CMAKE_SOURCE_DIR}/lib/modem-sniffer/
    ${CMAKE_SOURCE_DIR}/lib/telnet/
    ${CMAKE_SOURCE_DIR}/lib/device/NDevice/
    ${CMAKE_SOURCE_DIR}/lib/device/fujiDevice/
    ${CMAKE_SOURCE_DIR}/lib/hardware/
    ${CMAKE_SOURCE_DIR}/lib/tcpip/
    ${CMAKE_SOURCE_DIR}/lib/compat/
    ${CMAKE_SOURCE_DIR}/lib/network-protocol/
    ${CMAKE_SOURCE_DIR}/lib/utils/
    ${CMAKE_SOURCE_DIR}/lib/ftp/
    ${CMAKE_SOURCE_DIR}/lib/webdav/
    ${CMAKE_SOURCE_DIR}/lib/http/
    ${CMAKE_SOURCE_DIR}/lib/TNFSlib/
    ${CMAKE_SOURCE_DIR}/lib/fnjson/
    ${CMAKE_SOURCE_DIR}/lib/fnsgml/
    ${CMAKE_SOURCE_DIR}/components_pc/
    ${CMAKE_SOURCE_DIR}/components_pc/libssh/include/
    ${CMAKE_SOURCE_DIR}/build/components_pc/libssh/include/
    ${CMAKE_SOURCE_DIR}/components_pc/cJSON/
    ${CMAKE_SOURCE_DIR}/components_pc/mongoose/
    ${CMAKE_SOURCE_DIR}/components/gumbo-query/
    )
  target_link_libraries(sio_dstats_tests PRIVATE)
  add_test(NAME sio_dstats_tests COMMAND sio_dstats_tests)
endif()
