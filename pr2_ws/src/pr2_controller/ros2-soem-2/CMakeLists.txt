cmake_minimum_required(VERSION 3.9)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
project(soem2
  DESCRIPTION "Simple Open EtherCAT Master2"
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
  # Default to installing in SOEM2 source directory
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install)
endif()

set(SOEM2_INCLUDE_INSTALL_DIR include)
set(SOEM2_LIB_INSTALL_DIR lib)

if(WIN32)
  set(OS "win32")
  include_directories(oshw2/win32/wpcap/Include)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw2/win32/wpcap/Lib/x64)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    link_directories(${CMAKE_CURRENT_LIST_DIR}/oshw2/win32/wpcap/Lib)
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
  include_directories(oshw2/${OS}/${ARCH})
  file(GLOB OSHW2_EXTRA_SOURCES oshw2/${OS}/${ARCH}/*.c)
  set(OSHW2_SOURCES "${OS_HW_SOURCES} ${OSHW2_ARCHSOURCES}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
  set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
  message(STATUS "Building for RTEMS")
  set(OS "rtems")
  set(SOEM2_LIB_INSTALL_DIR ${LIB_DIR})
  set(BUILD_TESTS FALSE)
endif()

message(STATUS "OS is ${OS}")

file(GLOB SOEM2_SOURCES soem2/*.c)
file(GLOB OSAL2_SOURCES osal2/${OS}/*.c)
file(GLOB OSHW2_SOURCES oshw2/${OS}/*.c)

file(GLOB SOEM2_HEADERS soem2/*.h)
file(GLOB OSAL2_HEADERS osal2/osal.h osal2/${OS}/*.h)
file(GLOB OSHW2_HEADERS oshw2/${OS}/*.h)

add_library(soem2 STATIC
  ${SOEM2_SOURCES}
  ${OSAL2_SOURCES}
  ${OSHW2_SOURCES}
  ${OSHW2_EXTRA_SOURCES}
)
target_compile_options(soem2 PRIVATE -fPIC)

target_link_libraries(soem2 ${OS_LIBS})

target_include_directories(soem2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem2>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(soem2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal2>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(soem2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/osal2/${OS}>
  $<INSTALL_INTERFACE:include>
)


target_include_directories(soem2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/oshw2/${OS}>
  $<INSTALL_INTERFACE:include>
)

message(STATUS "LIB_DIR: ${SOEM2_LIB_INSTALL_DIR}")

# Export to lower stream files
ament_export_targets(soem2_EXPORT HAS_LIBRARY_TARGET)

install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Registering headers
install(FILES
  ${SOEM2_HEADERS}
  ${OSAL2_HEADERS}
  ${OSHW2_HEADERS}
  DESTINATION ${SOEM2_INCLUDE_INSTALL_DIR}
)

install(TARGETS soem2
  EXPORT soem2_EXPORT
  LIBRARY DESTINATION ${SOEM2_LIB_INSTALL_DIR}/${PROJECT_NAME}
  ARCHIVE DESTINATION ${SOEM2_LIB_INSTALL_DIR}
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION ${SOEM2_INCLUDE_INSTALL_DIR}
)

ament_package()
