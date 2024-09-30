# Update the Ubuntu system to be up to date
sudo apt update
sudo apt upgrade

# install ros2 humble
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=100" >> ~/.bashrc
echo "export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification" >> ~/.bashrc

# install gstreamer
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
     gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
     gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
     gstreamer1.0-qt5 gstreamer1.0-pulseaudio

# make sure the version of gstreamer is 1.20.3 (if not, installation of deepstream will be fail)
gst-launch-1.0 --gst-version

# upgrade glib version to 2.76.6
sudo apt install meson
sudo apt install ninja-build
git clone https://github.com/GNOME/glib.git
cd glib
git checkout 2.76.6
meson build --prefix=/usr
ninja -C build/
cd build/
ninja install
cd ../../
rm -rf glib
pkg-config --modversion glib-2.0

# Install Deep Stream Dependencies
sudo apt install \
libssl3 \
libssl-dev \
libgles2-mesa-dev \
libgstreamer1.0-0 \
gstreamer1.0-tools \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
gstreamer1.0-libav \
libgstreamer-plugins-base1.0-dev \
libgstrtspserver-1.0-0 \
libjansson4 \
libyaml-cpp-dev \
libjsoncpp-dev \
protobuf-compiler \
make \
git \
python3

# Upgrade the GCC version to match the version used to build the Ubuntu kernel
# (It solves build error problem when installing NVIDIA driver 535 on the Ubuntu 22.04 LTS)
sudo apt install --reinstall gcc-12
sudo ln -s -f /usr/bin/gcc-12 /usr/bin/gcc
gcc --version

# Install NVIDIA driver 535 version
sudo ubuntu-drivers devices
sudo ubuntu-drivers install nvidia:535

# Install TensorRT 8.6.1.6
sudo apt-get install --no-install-recommends libnvinfer-lean8=8.6.1.6-1+cuda12.0 libnvinfer-vc-plugin8=8.6.1.6-1+cuda12.0 \
libnvinfer-headers-dev=8.6.1.6-1+cuda12.0 libnvinfer-dev=8.6.1.6-1+cuda12.0 libnvinfer-headers-plugin-dev=8.6.1.6-1+cuda12.0 \
libnvinfer-plugin-dev=8.6.1.6-1+cuda12.0 libnvonnxparsers-dev=8.6.1.6-1+cuda12.0 libnvinfer-lean-dev=8.6.1.6-1+cuda12.0 \
libnvparsers-dev=8.6.1.6-1+cuda12.0 python3-libnvinfer-lean=8.6.1.6-1+cuda12.0 python3-libnvinfer-dispatch=8.6.1.6-1+cuda12.0 \
uff-converter-tf=8.6.1.6-1+cuda12.0 onnx-graphsurgeon=8.6.1.6-1+cuda12.0 libnvinfer-bin=8.6.1.6-1+cuda12.0 \
libnvinfer-dispatch-dev=8.6.1.6-1+cuda12.0 libnvinfer-dispatch8=8.6.1.6-1+cuda12.0 libnvonnxparsers-dev=8.6.1.6-1+cuda12.0 \
libnvonnxparsers8=8.6.1.6-1+cuda12.0 libnvinfer-vc-plugin-dev=8.6.1.6-1+cuda12.0 libnvinfer-samples=8.6.1.6-1+cuda12.0

# Install librdkafka (to enable Kafka protocol adaptor for message broker)
git clone https://github.com/confluentinc/librdkafka.git
cd librdkafka
git checkout tags/v2.2.0
./configure --enable-ssl
make
sudo make install
sudo mkdir -p /opt/nvidia/deepstream/deepstream/lib
sudo cp /usr/local/lib/librdkafka* /opt/nvidia/deepstream/deepstream-7.0/lib
sudo ldconfig
cd ..
rm -rf librdkafka

# Install the DeepStream SDK
# download deepstream-7.0_7.0.0-1_amd64.deb from https://catalog.ngc.nvidia.com/orgs/nvidia/resources/deepstream
sudo apt-get install ./deepstream-7.0_7.0.0-1_amd64.deb

# Check if deepstream plug-ins are successfuly installed in gstreamerr
gst-inspect-1.0 | grep "^nv"

# Test the deepstream-app (the reference application)
cd /opt/nvidia/deepstream/deepstream-7.0/samples/configs/deepstream-app
deepstream-app -c source30_1080p_dec_infer-resnet_tiled_display_int8.txt