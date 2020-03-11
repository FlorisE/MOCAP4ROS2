# Qualisys driver for ROS 2
This package provides a driver for using the Qualisys motion capture system in ROS 2.

## Installing on Linux
First obtain the Qualisys C++ SDK from [here](https://www.github.com/qualisys/qualisys_cpp_sdk) and place the files in the include/qualisys folder, then follow the building instructions. Afterwards, copy `qualisys_cpp_sdk.a` to `/usr/lib`.

Use the following command to build the workspace, excluding the vicon driver:
`colcon build --packages-skip vicon2_driver`
