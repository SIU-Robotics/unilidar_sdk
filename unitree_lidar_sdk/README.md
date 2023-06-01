# Unitree Lidar SDK

## Introduction
This package is a cmake package, which is specially used for running `Unitree LiDAR L1`.

The functions that this package can provide includes:
- Parse the raw data transmitted from the lidar hardware, and convert it into pointcloud and IMU data
- Get the pointcloud data
- Get the IMU data

The output pointcloud defaultly uses a self-defined data type so that this SDK doesn't rely too much on external dependencies. In other case,
- if you are used to use [Point Cloud Library](https://pointclouds.org/), you can use the header `unitree_lidar_sdk_pcl.h` to transform our pointcloud to PCL format;
- if you wish to directly use a [ROS](https://www.ros.org/) package, you are also able to utilize our ROS pacakge for this lidar.

## Dependency
We have verified that this package can successfully run under this environment:
- `Ubuntu 20.04` 

This SDK hardly depends on any external dependencies.
But if you want to use PCL cloud format, you need to install one. 

## Configure

Connect your lidar to your computer with a USB cable, then confirm your serial port name for lidar:
```
$ ls /dev/ttyUSB*
/dev/ttyUSB0
```

The default serial port name is `/dev/ttyUSB0`.
If it is not the default one, you need to modify the configuration parameter in `example_lidar.cpp`.

## Build

You can build this program as a cmake project:
```
cd unitree_lidar_sdk

mkdir build

cd build

cmake .. && make -j2
```

## Run
Directly run the example:
```
../bin/example_lidar
```

The output is like this:
```
$ ../bin/example_lidar 
lidar firmware version = 0.3.2+230511
lidar sdk version = 1.0.3

Dirty Percentage = 5.145833 %
Dirty Percentage = 4.166667 %
Dirty Percentage = 4.166667 %

Turn on all the LED lights ...
Turn off all the LED lights ...
Set LED mode to: FORWARD_SLOW ...
Set LED mode to: REVERSE_SLOW ...
Set LED mode to: SIXSTAGE_BREATHING ...

Set Lidar working mode to: NORMAL_MODE ... 

An IMU msg is parsed!
	stamp = 1683874160.559222, id = 729
	quaternion (x, y, z, w) = [0.0131, -0.0091, 0.6888, -0.7225]

An IMU msg is parsed!
	stamp = 1683874160.564979, id = 121
	quaternion (x, y, z, w) = [0.0102, -0.0093, 0.7099, -0.7018]

An IMU msg is parsed!
	stamp = 1683874160.568425, id = 122
	quaternion (x, y, z, w) = [0.0118, -0.0096, 0.7099, -0.7018]

An IMU msg is parsed!
	stamp = 1683874160.573472, id = 123
	quaternion (x, y, z, w) = [0.0126, -0.0093, 0.7098, -0.7018]

An IMU msg is parsed!
	stamp = 1683874160.577348, id = 124
	quaternion (x, y, z, w) = [0.0128, -0.0093, 0.7099, -0.7018]

A Cloud msg is parsed! 
	stamp = 1683874145.535888, id = 1
	cloud size  = 278
	first 10 points (x,y,z,intensity,time,ring) = 
	  (-0.029885, -0.136897, 0.000448, 88.000000, 0.000000, 0)
	  (-0.035384, -0.171399, 0.005140, 91.000000, 0.000023, 0)
	  (-0.043000, -0.219542, 0.012437, 127.000000, 0.000046, 0)
	  (-0.054879, -0.294965, 0.024572, 132.000000, 0.000069, 0)
	  (-0.055288, -0.301204, 0.033170, 106.000000, 0.000093, 0)
	  (-0.054542, -0.300348, 0.041173, 101.000000, 0.000116, 0)
	  (-0.053764, -0.299285, 0.049148, 99.000000, 0.000139, 0)
	  (-0.056123, -0.318406, 0.060981, 92.000000, 0.000162, 0)
	  (-0.055230, -0.316827, 0.069422, 91.000000, 0.000185, 0)
	  (-0.051257, -0.294854, 0.072849, 128.000000, 0.000208, 0)
	  ...
```

Here, we print the first 10 points of the pointcloud message and the quaternion of the IMU message.

## Version History

### v1.0.0 (2023.05.04)
- Support firmware version: 0.3.1

### v1.0.1 (2023.05.05)
- Support firmware version: 0.3.1
- Add support of setting lidar working mode, e.g. `NORMAL_MODE` and `STANDBY_MODE`
- Add support of LED lights

### v1.0.2 (2023.05.11)
- Support firmware version: 0.3.2

### v1.0.3 (2023.05.12)
- Support firmware version: 0.3.2
- Add support of getting the percentage of removed dirty points

### v1.0.4 (2023.05.30)
- Support firmware version: 1.0.1