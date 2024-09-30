# gd_theta_driver

## Getting started

### with docker

```bash
git clone --recursive https://github.com/GuideDog-ETRI/gd_theta_driver.git
cd gd_theta_driver
docker build -t theta_driver .
docker run --rm -it --net=host --privileged theta_driver
ros2 run theta_driver theta_driver_node
```

### without docker

You need to install the libuvc-theta and gstreamer dependencies before using this package.

1. install gd_libuvc-theta

```bash
git clone https://github.com/GuideDog-ETRI/gd_libuvc-theta.git
cd gd_libuvc-theta
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make && sudo make install
cd ..
rm -rf gd_libuvc-theta
```

2. install gstreamer dependencies

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
     gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
     gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
     gstreamer1.0-qt5 gstreamer1.0-pulseaudio

# check gstreamer version
gst-launch-1.0 --gst-version
```

To install the package:

```bash
source /opt/ros/humble/setup.bash
mkdir -p theta_driver_ws/src
cd theta_driver_ws
git clone https://github.com/GuideDog-ETRI/gd_theta_driver.git src/theta_driver
git clone https://github.com/stella-cv/libuvc-theta-sample.git src/theta_driver/3rd/libuvc-theta-sample
colcon build
```

Try it with:

```bash
source install/setup.bash
ros2 run theta_driver theta_driver_node 
```

And then you can use image_view package, rqt or rviz2 to see the published image.

## Optimize theta driver

Latency of theta_driver can be reduced by using the **NVIDIA DeepStream** GStreamer plugin. Refer to https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Installation.html

1. Install the Ubuntu 22.04.5 LTS or later

2. Update the Ubuntu system to the latest  (GUI updater or `sudo apt update && sudo apt upgrade`)

3. Install DeepStream following the instructions in `install_deepstream.sh` script

4. Modify the theta_driver pipeline to use the NVIDIA decoder

theta_driver_node.cpp (line number 113 ~ 114)
```bash
//pipeline_ = "appsrc name=ap ! queue ! h264parse ! queue ! decodebin ! queue ! videoconvert n_threads=8 ! queue ! video/x-raw,format=RGB ! appsink name=appsink emit-signals=true";
pipeline_ = "appsrc name=ap ! queue ! h264parse ! queue ! nvh264dec ! queue ! gldownload ! queue ! nvvideoconvert n_threads=8 ! queue ! video/x-raw,format=RGB ! appsink name=appsink qos=false sync=false emit-signals=true";
```
