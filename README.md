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

1. Install libusb

```
sudo apt install libusb-1.0-0-dev
```

2. install gd_libuvc-theta

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

3. install gstreamer dependencies

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
