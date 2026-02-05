cmake_minimum_required(VERSION 3.9)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
project(soem3
  DESCRIPTION "Simple Open EtherCAT Master3"
  VERSION 1.4.0
  LANGUAGES C
)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  # Default to installing in SOEM3 source directory
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install)
endif()

set(SOEM3_INCLUDE_INSTALL_DIR include)
set(SOEM3_LIB_INSTALL_DIR lib)

if(WIN32)
  set(OS "win32")
  include_directories(oshw3/win32/wpcap/Include)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw3/win32/wpcap/Lib/x64)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw3/win32/wpcap/Lib)
  endif()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /WX")
  set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX AND NOT APPLE)
  set(OS "linux")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(OS_LIBS pthread rt)
elseif(APPLE)
  # This must come *before* linux or MacOSX will identify as Unix.
  set(OS "macosx")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(OS_LIBS pthread pcap)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
  set(OS "rtk")
  message(STATUS "ARCH is ${ARCH}")
  message(STATUS "BSP is ${BSP}")
  include_directories(oshw3/${OS}/${ARCH})
  file(GLOB OSHW3_EXTRA_SOURCES oshw3/${OS}/${ARCH}/*.c)
  set(OSHW3_SOURCES "${OS_HW_SOURCES} ${OSHW3_ARCHSOURCES}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
  set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
  message(STATUS "Building for RTEMS")
  set(OS "rtems")
  set(SOEM3_LIB_INSTALL_DIR ${LIB_DIR})
  set(BUILD_TESTS FALSE)
endif()

message(STATUS "OS is ${OS}")

file(GLOB SOEM3_SOURCES soem3/*.c)
file(GLOB OSAL3_SOURCES osal3/${OS}/*.c)
file(GLOB OSHW3_SOURCES oshw3/${OS}/*.c)

file(GLOB SOEM3_HEADERS soem3/*.h)
file(GLOB OSAL3_HEADERS osal3/osal.h osal3/${OS}/*.h)
file(GLOB OSHW3_HEADERS oshw3/${OS}/*.h)

add_library(soem3 STATIC
  ${SOEM3_SOURCES}
  ${OSAL3_SOURCES}
  ${OSHW3_SOURCES}
  ${OSHW3_EXTRA_SOURCES}
)
target_compile_options(soem3 PRIVATE -fPIC)

target_link_libraries(soem3 ${OS_LIBS})

target_include_directories(soem3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem3>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(soem3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal3>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(soem3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal3/${OS}>
  $<INSTALL_INTERFACE:include>
)


target_include_directories(soem3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/oshw3/${OS}>
  $<INSTALL_INTERFACE:include>
)

message(STATUS "LIB_DIR: ${SOEM3_LIB_INSTALL_DIR}")

# Export to lower stream files
ament_export_targets(soem3_EXPORT HAS_LIBRARY_TARGET)

install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Registering headers
install(FILES
  ${SOEM3_HEADERS}
  ${OSAL3_HEADERS}
  ${OSHW3_HEADERS}
  DESTINATION ${SOEM3_INCLUDE_INSTALL_DIR}
)

install(TARGETS soem3
  EXPORT soem3_EXPORT
  LIBRARY DESTINATION ${SOEM3_LIB_INSTALL_DIR}/${PROJECT_NAME}
  ARCHIVE DESTINATION ${SOEM3_LIB_INSTALL_DIR}
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${SOEM3_INCLUDE_INSTALL_DIR}
)

ament_package()
