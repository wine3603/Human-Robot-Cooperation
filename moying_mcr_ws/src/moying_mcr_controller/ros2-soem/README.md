# ROS2-SOEM
![Static Badge](https://img.shields.io/badge/license-GPLv2-red)
![Static Badge](https://img.shields.io/badge/Test_on_-ros2_humble_in_Ubuntu22.04-blue)


This package is down-stream of SOEM project, you can view the origin project [HERE](https://github.com/OpenEtherCATsociety/SOEM). This project have removed all the examples, if you want to build the example program, please download the up-stream package.

## Install
### From DEB package
```bash
sudo dpkg -i ros-humble-soem_1.4.0-0jammy_amd64.deb
```

### From Source
ROS2-SOEM is a ros2 package, to install and use it in your project, please running the following steps in your terminal:

```bash
$ cd ${your_worksapce_path}/src
$ git clone https://github.com/UoN-Hari/ROS2-SOEM soem
$ cd ..
$ colcon build
```

## Usage
To use SOEM in your project, please add following lines in your `package.xml`,

```XML
<build_depend>soem</build_depend>
<exec_depend>soem</exec_depend>
```

And in your `CMakeLists.txt`, add the corressponding dependency,

```cmake
# Add soem package here
find_package(soem REQUIRED)

# No need to derive
include_directories(
  include
)

# Your executable generation command, no need to derive
add_executable(${executable_name} SOURCES)

# Add soem package here
ament_target_dependencies(${executable_name}
  soem ...
)

# No need to derive
install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)
```

## TODO
* Multi-platform test
* Build binary package
