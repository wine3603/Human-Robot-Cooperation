cmake_minimum_required(VERSION 3.5)
project(serial)

# Find ament packages and libraries
find_package(ament_cmake REQUIRED)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

## Sources
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if(APPLE)
    # If OSX
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(${PROJECT_NAME} SHARED ${serial_SRCS})

if(APPLE)
    # Link necessary libraries for macOS
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    # Link rt and pthread directly for UNIX systems
    target_link_libraries(${PROJECT_NAME} rt pthread)
else()
    # Link setupapi for Windows
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

# Export the library for ament
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

## Add executable example
add_executable(serial_example examples/serial_example.cc)
target_link_libraries(serial_example ${PROJECT_NAME})

## Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

## Install the library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

## Install headers
install(DIRECTORY include/
  DESTINATION include/
)

## Install the executable
install(TARGETS serial_example
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
