# baichuan_fieldbus_ethercat

## 介绍
基于SOEM的EtherCAT主站。

SOEM可以参考 https://openethercatsociety.github.io/doc/soem/tutorial_8txt.html 。

## 依赖项
1.  [soem](https://github.com/OpenEtherCATsociety/SOEM): 版本1.4.0，已编译放在当前目录下。

## 使用说明
1.  配置运行环境 Ubuntu22.04 + ROS2-Humble
2.  打Ubuntu实时补丁（测试非必须）
3.  下载上述依赖项并编译

## 测试说明
本package包含一个测试程序baichuan_fieldbus_ethercat_test_node，测试时需要root权限，硬件上需要连接大族机械臂。

打开终端，进入root用户，然后运行以下命令启动测试节点。
``` 
export ROS_DOMAIN_ID=9
source {ROS2_WS_PATH}/install/setup.bash
ros2 run baichuan_fieldbus_ethercat baichuan_fieldbus_ethercat_test_node --ros-args -p if_name:=enp7s0 -p period_ms:=1000
```
其中，参数if_name为网卡名称，必须指定；period_ms为查询PDO数据的周期，默认为1000, 可以忽略。

程序运行后，会周期性显示PDO数据。
