# gcc
sudo apt install --reinstall gcc-12
sudo ln -s -f /usr/bin/gcc-12 /usr/bin/gcc
gcc --version

# git settings
git config --global user.name roricljy
git config --global user.email roricljy@gmail.com

# install ros2 humble
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

# check gstreamer version
gst-launch-1.0 --gst-version

# install gd_libuvc-theta
git clone https://github.com/GuideDog-ETRI/gd_libuvc-theta.git
cd gd_libuvc-theta
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install

# https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Installation.html
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
gcc \
make \
git \
python3

# Install CUDA Toolkit 12.2
# https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"
sudo apt-get update
sudo apt-get install cuda-toolkit-12-2

# Install NVIDIA driver 535.161.08
# https://www.nvidia.cn/Download/driverResults.aspx/222416/en-us/
#Ensure gdm, lightdm or Xorg service is stopped while installing nvidia driver
#chmod 755 NVIDIA-Linux-x86_64-535.161.08.run
#sudo service gdm stop
#sudo service lightdm stop
#sudo pkill -9 Xorg
#sudo ./NVIDIA-Linux-x86_64-535.161.08.run --no-cc-version-check
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
# https://catalog.ngc.nvidia.com/orgs/nvidia/resources/deepstream
sudo apt-get install ./deepstream-7.0_7.0.0-1_amd64.deb
gst-inspect-1.0 | grep "^nv"

# Build gstramer 1.20.3 in case deepstream fails
# https://gstreamer.freedesktop.org/documentation/installing/building-from-source-using-meson.html?gi-language=c
sudo apt-get install flex bison
git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git
cd gstreamer
git checkout 1.20.3
meson build --prefix=/usr --buildtype release
meson compile -C build
meson install -C build
cd ..
rm -rf gstreamer
gst-launch-1.0 --gst-version

# Run the deepstream-app (the reference application)
cd /opt/nvidia/deepstream/deepstream-7.0/samples/configs/deepstream-app
deepstream-app -c source30_1080p_dec_infer-resnet_tiled_display_int8.txt