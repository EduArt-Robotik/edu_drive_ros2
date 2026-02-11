# Setting up a Raspberry Pi 5 using Ubuntu 23.10 Server

>**Note:** This guide is a legacy documentation. Currently (July 2024), Ubuntu Server 24.04 LTS in combination with ROS 2 Jazzy is recommended for a Raspberry Pi 5.

Software installed in this guide: 
- Ubuntu 23.10
- ROS 2 Humble

### 1. Flash microSD card
To flash the microSD card we recommend using the [RPi Imager](https://www.raspberrypi.com/software/) which can be installed on Linux, Windows and macOS. On Ubuntu, use the command `sudo apt install rpi-imager`. After starting the software, choose your device, Ubuntu 23.10 Server as the operating system and microSD card. After pressing *Next* you can change some additional settings like your passwords or username. We recommend to setup at least your WiFi and ssh connection.

### 2. Connect to your Raspberry Pi
After inserting the microSD card and powering up the Raspberry Pi, connect to the Raspberry via ssh. The command follows the layout `ssh <username>@<ip-address>`. To find out the IP, have a look into your router or use the `nmap` tool to list all available devices in your network.

### 3. Update and install packages
```bash
sudo apt update
sudo apt upgrade
sudo apt install can-utils build-essential git
```
You might need to reboot the Pi with the command `sudo reboot`

### 4. Configure the firmware for the CAN interfaces
For Ubuntu 23.10. the file /boot/firmware/network-config had to be renamed manually (/boot/firmware/network-config.bak) on a Raspberry Pi 5. As long as this file exists, /boot/firmware/config.txt is not read.

Add the configuration of all three can interfaces to the /boot/firmware/config.txt file. This can be done with the following command:
```bash
sudo bash -c "echo -e '\ndtoverlay=spi1-2cs\ndtoverlay=mcp251xfd,spi0-0,oscillator=40000000,interrupt=25\ndtoverlay=mcp251xfd,spi0-1,oscillator=40000000,interrupt=13\ndtoverlay=mcp251xfd,spi1-0,oscillator=40000000,interrupt=24' >> /boot/firmware/config.txt"
```
### 5. Add udev rules and services for CAN interfaces
The extension board for the Raspberry Pi provides three CANFD interfaces. To ensure that the naming of the interfaces is the same after each boot process, a udev rule must be created in the /etc/udev/rules.d directory. Create the file /etc/udev/rules.d/42-mcp251xfd.rules with the following content:
```bash
KERNELS=="spi0.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="CAN0", TAG+="systemd", ENV{SYSTEMD_WANTS}="can0-attach.service"
KERNELS=="spi0.1", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="CAN1", TAG+="systemd", ENV{SYSTEMD_WANTS}="can1-attach.service"
KERNELS=="spi1.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="CAN2", TAG+="systemd", ENV{SYSTEMD_WANTS}="can2-attach.service"
```
In this way, the CAN interface for the motor controllers is always named CAN2. The CAN0 and CAN1 interfaces can be accessed via the sockets on the expansion board (see labeling on the board) and are intended for connecting the flexible sensor ring from EduArt. Please note that these entries require the definition of three systemd services. Create the file /etc/systemd/system/can0-attach.service with the following content.
```bash
[Service]
Type=oneshot
ExecStart=ip link set CAN0 up type can bitrate 1000000 dbitrate 1000000 fd on
```
... then, the file /etc/systemd/system/can1-attach.service
```bash
[Service]
Type=oneshot
ExecStart=ip link set CAN1 up type can bitrate 1000000 dbitrate 1000000 fd on
```

... and finally the file /etc/systemd/system/can2-attach.service
```bash
[Service]
Type=oneshot
ExecStart=ip link set CAN2 up type can bitrate 500000
```

### 6. Install ROS
For Ubuntu Server 23.10 install [ROS Humble](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html).
Read the official documentation for reference on [building from source](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) and [prebuilt packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for more detail.\
The following commands prepare the installation of ROS:
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Unfortunately, there were no complete packages for Ubuntu 23.10 when this documentation was created, so the installer has to be fooled into using an Ubuntu Jammy version:
```bash
sudo apt install -y python3-flake8-docstrings python3-pip python3-pytest-cov python3-rosinstall-generator colcon
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
rosinstall_generator ros_base --format repos --rosdistro humble --deps > base.repos
vcs import --input base.repos src
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-catkin-pkg-modules python3-rosdistro-modules" --os=ubuntu:jammy
colcon build --symlink-install
```

### 7. Add ROS Distribution to .bashrc
To avoid having to re-source the setup.bash file of the ros-distribution in every new terminal, it can be added to the ~/.bashrc file.
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 8. Optional: Static IP address
By default, the configuration of the Ubuntu 22.04. server edition is set to DHCP. If you would like to set a static IP address, you can do this by making the following adjustment:
```bash
sudo bash -c "echo 'network: {config: disabled}' > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg"
```
This switches off the automatism with which the network configuration file is generated. The network configuration file /etc/netplan/50-cloud-init.yaml must now be adapted:
```yaml
network:
    renderer: networkd
    ethernets:
        eth0:
          addresses:
            - 192.168.178.111/24
          nameservers:
            addresses: [4.2.2.2, 8.8.8.8]
          routes:
            - to: default
              via: 192.168.178.1
    version: 2
```
Replace the IP addresses above with your desired configuration. Then reboot your system.

### 9. Get and build the edu_drive_ros2 software
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/EduArt-Robotik/edu_drive_ros2.git
cd ..
colcon build --packages-select edu_drive_ros2 --symlink-install
source install/setup.bash
```

Now, the launch file should be started. Please adjust the parameters in edu_drive.yaml before.
```bash
ros2 launch edu_drive_ros2 edu_drive.launch.py
```