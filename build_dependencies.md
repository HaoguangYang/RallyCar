## Additional packages that needs to be built from source

This document provides notes of the caveats when interfacing with a few classical hardware

### Intel RealSense cameras (D435 series and T265 series)

The Linux kernel of the NVIDIA Jetson board is missing a few modules for the official pre-compiled binaries to interface with the camera. Therefore, the `librealsense` library and the ROS2 wrapper need to be compiled from source. To support both D435 series and T265 series, the highest supported driver version is `v2.51.1`.

To compile and install the `librealsense` library, use a non-ROS2 folder, e.g. `/tmp`. Perform the following steps:
```sh
# first remove all binaries installed from apt
sudo apt purge librealsense*

git clone -b v2.51.1 https://github.com/IntelRealSense/librealsense.git --depth=1
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
mkdir build && cd build

cmake ../ -DFORCE_LIBUVC=true \
  -DCMAKE_BUILD_TYPE=release \
  -DBUILD_WITH_CUDA=true \
  -DCMAKE_CUDA_ARCHITECTURES=87
# 87 is the compute capability of Jetson Orin Nano
# Look up your NVIDIA hardware at: https://developer.nvidia.com/cuda-gpus
```

Before building, you need to delete a code segment that contains extended characters. Open `librealsense/src/libusb/libusb.h`, and **DELETE** the `#if 0` ... `#endif` block. Now you can proceed to build:

```sh
make -j6        # 8G memory is sufficient to run 6 parallel threads
sudo make install
```

After installation, run `ldconfig -p | grep realsense` to make sure the self-built libraries are indexed (non-empty output). If the libraries are found, proceed with building the ROS2 wrapper.

In your ROS2 workspace, clone the `realsense-ros` metapackage. The highest supported version is `4.51.1` for this `librealsense` driver library:
```sh
cd ros2_ws/src
git clone -b 4.51.1 https://github.com/IntelRealSense/realsense-ros.git --depth=1

cd ..
source /opt/ros/humble/setup.bash
colcon build
```

- **Known issues**: The T265 camera may be powered on at boot and enter sleep mode before the OS takes over. The symptom is the camera showing up as a USB 2.0 device (`480M`) instead of a USB 3.0 device (`5000M`). You can check the device bus ID with `lsusb`, and then look up its detected bandwidth with `lsusb -t`. If you encounter this issue, you need to unplug and re-plug the T265. Alternatively, you may use the `uhubctl` utility (https://github.com/mvp/uhubctl) to cycle the power supply of the connected port.

### MicroROS (agent)

In case the peripheral hardware utilizes MicroROS (e.g. in a Jetson/RecoNode or Jetson/Zynq7010 setup), we need to include MicroROS agent in our workspace to enable native communication with ROS messages. To build MicroROS agent, we adopt the following two-step build process.

The first step pulls and builds the `micro_ros_setup` metapackage, which guides the actual build of the agent. This step only needs to be performed once. In a pre-populated workspace, we don't need the `micro_ros_setup` metapackage.
```sh
# install dependent software
sudo apt install python3-vcstool python3-rosdep
# initialize rosdep if you have not done so
sudo rosdep init
rosdep update

cd ros2_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git --depth=1
cd ..
source /opt/ros/humble/setup.bash
colcon build
source ./install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh

# if you have accidentally installed ros-humble-librealsense2, uninstall it (as we are building from source)
sudo apt purge ros-humble-librealsense2
sudo apt autoremove
```

The second step builds everything alltogether, and makes `micro_ros_agent` available.
```sh
colcon build
source ./install/setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyTHS0 -b ${BAUDRATE}    # supply your own baudrate
```

