# edu_drive_ros2
This package comprises a ROS2 interface for EduArt's generic drive concept. It covers several kinematic concepts: Differential drive, Mecanum steering and Skid steering. All three can be used in dependency of the mounted wheels and the configuration of YAML parameters.

<p align="center">
  <img src="/images/Prototype_yellow.jpg" width="90%"/>
  <br><em>Fig.: Robot prototype using the Free Kinematic Kit.</em>
</p>

## Launching the Robot
In order to run the robot, you need to launch the appropriate launch file. In the launch folder, there is a prepared template.
Configure the correct kinematic concept and motor parameters. A description of the YAML file can be found below.
```bash
ros2 launch edu_drive_ros2 edu_drive.launch.py
```
When everthing is initialized well, one should see the following output:
```bash
# Listener start
CAN Interface: CAN2
[ INFO] [1651663592.994224328]: Instanciated robot with vMax: 0.366519 m/s and omegaMax: 0.733038 rad/s
...
```
After startup, the drive system is in the deactivated state.

Please notice also, that the ROS variable ROS_DOMAIN_ID should be set properly.

## YAML file parameters

| tag    | description      |
| ------ |:--|
| usingPowerManagementBoard | Define if a Power Management Board is used, default is "true"
| verbosity      | Display status messages for debugging, default is "false"
| canInterface   | SocketCAN interface, e.g. CAN2       |
| frequencyScale | Divider of PWM frequency, Base frequency: 500kHz (Divider=1)     |
| inputWeight    | Low pass filtering, setpoint = inputWeight*setpoint_new + (1-inputWeight)*setpoint_old  |
| maxPulseWidth  | Limitation of pulse width, meaningful values [0; 100]      |
| timeout        | Dead time monitoring of communication (in milliseconds) |
| kp, ki, kd     | Controller coefficients    |
| antiWindup     | Enable anti windup monitoring of closed-loop controller     |
| invertEnc      | Invert encoder signal |
| responseMode   | Activate transmission of RPM or position measurements of motor controller |
| controllers    | Number of motor controllers used |
| canID          | Motorcontroller ID (set by DIP switch on motor controller PCB) |
| gearRatio      | Gear ratio of connected geared motors |
| encoderRatio   | Encoder pulses per motor revolution (rising and falling edge evaluation) |
| rpmMax         | Maximum revolutions per minute of geared motor pinion |
| channel        | Used channel of motor controller. There are single-channel motorshields and dual-channel motorshields. Meaningful values are 0 or 1. |
| kinematics     | Three-dimensional vector describing the conversion from Twist messages in motor revolutions. See explanation below. |

## Calculation of the kinematic parameters
The kinematic concept uses a conversion matrix for the conversion of twist parameters and wheel speeds.

<img src="https://latex.codecogs.com/svg.image?\begin{pmatrix}&space;\omega_0&space;\\&space;\omega_1&space;\\&space;\omega_2&space;\\&space;\omega_3\end{pmatrix}&space;=&space;\mathbf{T}\begin{pmatrix}v_x&space;\\v_y&space;\\\omega\end{pmatrix}&space;" title="https://latex.codecogs.com/svg.image?\begin{pmatrix} \omega_0 \\ \omega_1 \\ \omega_2 \\ \omega_3\end{pmatrix} = \mathbf{T}\begin{pmatrix}v_x \\v_y \\\omega\end{pmatrix} " />

, where &omega;<sub>i</sub> are the wheel's angular velocities and v<sub>x</sub>, v<sub>y</sub> and &omega; are Twist values. The matrix **T** can be calculated as follows:
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\begin{pmatrix}&space;kx_0&space;&&space;ky_0&space;&&space;k\omega{}_0\\&space;kx_1&space;&&space;ky_1&space;&&space;k\omega{}_1\\&space;kx_2&space;&&space;ky_2&space;&&space;k\omega{}_2\\&space;kx_3&space;&&space;ky_3&space;&&space;k\omega{}_3\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \begin{pmatrix} kx_0 & ky_0 & k\omega{}_0\\ kx_1 & ky_1 & k\omega{}_1\\ kx_2 & ky_2 & k\omega{}_2\\ kx_3 & ky_3 & k\omega{}_3\end{pmatrix}" />

for a four-wheeled robot. kx<sub>i</sub>, ky<sub>i</sub> and k&omega;<sub>i</sub> are the translation parameters from one space to the other. These parameters include the wheel radius r as well as the robot length l<sub>x</sub> and robot width l<sub>y</sub>.

Depending on the polarity of the motor wiring, the kinematic parameters may have to negated.

### Example for a Differential drive
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\begin{pmatrix}&space;\frac{1}{r}&space;&&space;0&space;&&space;-\frac{l_y}{2&space;\cdot&space;r}\\&space;-\frac{1}{r}&space;&&space;0&space;&&space;-\frac{l_y}{2&space;\cdot&space;r}\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \begin{pmatrix} \frac{1}{r} & 0 & -\frac{l_y}{2 \cdot r}\\ -\frac{1}{r} & 0 & -\frac{l_y}{2 \cdot r}\\\end{pmatrix}" />

### Example for a Mecanum drive
<img src="https://latex.codecogs.com/svg.image?\mathbf{T}&space;=&space;\frac{1}{r}\begin{pmatrix}&space;&space;1&space;&&space;-1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;-1&space;&&space;-1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;&space;1&space;&&space;&space;1&space;&&space;-\frac{l_x&plus;l_y}{2}\\&space;-1&space;&&space;&space;1&space;&&space;-\frac{l_x&plus;l_y}{2}\end{pmatrix}" title="https://latex.codecogs.com/svg.image?\mathbf{T} = \frac{1}{r}\begin{pmatrix} 1 & -1 & -\frac{l_x+l_y}{2}\\ -1 & -1 & -\frac{l_x+l_y}{2}\\ 1 & 1 & -\frac{l_x+l_y}{2}\\ -1 & 1 & -\frac{l_x+l_y}{2}\end{pmatrix}" />

> **Concrete parameter example:** for Faulhaber motors, series 2224U018SR, gear ratio: 44:1, wheel diameter 100mm ($r=0.05m$), robot length $lx=0.25m$, robot width $l_y=0.35m$

$k_{x,n} = \pm\frac{1}{r} = \pm 20$

$k_{y,n} = \pm\frac{1}{r} = \pm 20$ (mecanum wheels) or  $k_{y,n} = 0$ (skid steering)

$k_{w,n} = \pm\frac{1}{r} \cdot \frac{l_x+l_y}{2} = \pm 6$ (mecanum wheels) or $k_{w,n} = \pm\frac{l_y}{2 \cdot r} = \pm 3.5$ (skid steering)

# Setting up a Raspberry PI from scratch
### 1. Choose between Raspberry Pi OS or Ubuntu
Which version of the operating system you use determines which ROS version can be installed later. We also recommend using the Ubuntu Server Edition. Besides Ubuntu it is also possible to use Raspberry Pi OS, but ROS has to be built from source later on. This guide covers the installation of **Ubuntu 24.04 Server** in combination with **ROS 2 Jazzy** on a **Raspberry Pi 5**. For other Ubuntu or Raspberry versions, see the following legacy guides:
   - [Ubuntu 22.04 Server with ROS 2 Humble on Raspberry Pi 4](/doc/legacyInstallationGuides/Ubuntu22.md)
   - [Ubuntu 23.04 Server with ROS 2 Humble on Raspberry Pi 5](/doc/legacyInstallationGuides/Ubuntu23.md)
### 2. Flash microSD card
To flash the microSD card we recommend using the [RPi Imager](https://www.raspberrypi.com/software/) which can be installed on Linux, Windows and macOS. On Ubuntu, use the command `sudo apt install rpi-imager`. After starting the software, choose your device, desired operating system and microSD card. After pressing *Next* you can change some aditional settings like your passwords or username. We recommend to setup your wifi and ssh connection.
### 3. Connect to your Raspberry Pi
After inserting the microSD card and powering up the Raspberry Pi, connect to the Rasperry via ssh. The command follows the layout `ssh <username>@<ip-address>`. To find out the IP, have a look into your router or use the `nmap` tool to list all available devices in your network.

### 4. Update and install packages
```bash
sudo apt update
sudo apt upgrade
sudo apt install can-utils build-essential git
```
You might need to reboot the Pi with the command `sudo reboot`

### 5. Kernel upgrade (Ubuntu 24.04.1 LTS)
Unfortunately, there is a bug in the handling of our CAN module in the kernel version of Ubuntu 24.04.1 LTS used. This bug leads to a segmentation fault during the boot process. However, this bug has already been recognized and is already in the list of proposed fixes for version 24.04.1 LTS. It is also to be expected that this problem will be solved from version 24.04.2 LTS. To install the proposed kernel, these sources must be activated for the package manager. Therefore, in the file `/etc/apt/sources.list.d/ubuntu.sources`, add the entry `noble-proposed` so that the suites are defined as follows:
`Suites: noble noble-updates noble-backports noble-proposed`
The kernel can then be installed:
```bash
sudo apt update
sudo apt install linux-image-6.8.0-1019-raspi/noble-proposed
sudo reboot
```
After booting, you can use the following command to check whether the current kernel is installed:
```bash
uname -r
```

### 6. Configure the firmware for the CAN interfaces
   
Add the configuration of all three can interfaces to the /boot/firmware/config.txt file. This can be done with the following command:
```bash
sudo bash -c "echo -e '\ndtoverlay=spi1-2cs\ndtoverlay=mcp251xfd,spi0-0,oscillator=40000000,interrupt=25\ndtoverlay=mcp251xfd,spi0-1,oscillator=40000000,interrupt=13\ndtoverlay=mcp251xfd,spi1-0,oscillator=40000000,interrupt=24' >> /boot/firmware/config.txt"
```

### 7. Add udev rules and services for CAN interfaces
   
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

### 8. Install ROS
Select the ROS [distribution](https://docs.ros.org/en/rolling/Releases.html) depending on the installed version of your operating system. For Ubuntu Server 24.04 install [ROS Jazzy](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html).
Read the official documentation for reference on [building from source](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html) and [prebuilt packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) for more detail.\
> **Important hint:** It is important that UTF8 is supported. Other language settings, such as those specified in the ROS2 documentation, should work. We have also tested the German settings. For German settings, please replace the strings “en_US” with “de_DE”.
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
Now install ROS 2 Jazzy:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-base
sudo apt install python3-colcon-common-extensions
sudo apt install ros-dev-tools
sudo reboot
```

### 9. Add ROS Distribution to .bashrc
To avoid having to re-source the setup.bash file of the ros-distribution in every new terminal, it can be added to the ~/.bashrc file.
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 10.  Optional: Static IP address
#### Ubuntu 22.04. server edition
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
#### Raspberry Pi OS / Debian version: 12 (bookwork)
The network configuration can easily be done with the command nmtui
```bash
sudo nmtui
```

### 11.  Get and build the edu_drive_ros2 software
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

## Free Kinematics Kit
The Free Kinematics Kit from EduArt gives you all the freedom you need to build your own robot with few restrictions on the mechanical design.

<p align="center">
  <img src="/images/Free_Kinematics_Kit_1920.jpg" width="80%"/>
  <br><em>Fig.: Kinematic Kit in two different variants, with Ethernet Gateway or as plug-in solution for a Raspberry Pi 5.</em>
</p>

It consists of the following components, which can all be plugged together:
- *Adapter Board* either directly pluggable onto a Raspberry Pi 4/5 or as a standalone EduArt Ethernet controller board, with which you can connect any computer with an Ethernet interface. This gives you the freedom to build the computing power into your robot that you need for your desired application. This board is supplied via the socket strips, i.e. a circuit board with the appropriate sockets for a power supply unit or a battery should be used. The possible input voltage V<sub>in</sub> is 12V to 55V. Depending on how many motors are connected, the power supply must be able to supply higher currents. High-current batteries are recommended for operating a robot (min. 8A).
- *Motorcontrollers* can be plugged in directly. This allows you to control the speed of 1 to 8 motors. The motor controllers are available in two versions, single-channel or dual-channel. Up to four motor controllers can be used, i.e. it is possible to use 1 to 4 larger motors (I<sub>RMS</sub> up to 5A) or 1 to 8 smaller motors (I<sub>RMS</sub> up to 2.5A). The dielectric strength of the single-channel motor controllers is 55V. The dielectric strength of the dual-channel motor controllers is 35V.
- The *Power Management Module* takes over the charge control of a 19.2V NiMH battery pack and also offers an on/off logic. Temperature monitoring of the battery uses an integrated 6.8kOhm NTC. Never connect a battery other than the one supplied by EduArt.
- The *Auxiliary Power Supply Module* provides additional voltage levels with which you can supply additional devices. The permissible operating voltage is between 15V and 36V.
- The *Extension Shield* allows you to extend the pin header of the Raspberry Pi. In addition, 8 model servos can be connected to the *Extension Shield*.

<p align="center"  width="100%">
  <img width="80%" src="/images/extend_kinematic_kit.jpg"/>
  <br><em>Fig.: Extend Kinematic Kit with Extension shield</em>
</p>

<p align="center"  width="100%">
  <img src="/images/PowerManagement.jpg" width="80%"/>
  <br><em>Fig.: Top view of power management module. Above the white Control Socket there is a two-pole socket for powering the Extension Shield.</em>
</p>

> **Warning:** The Raspberry single-board computer is supplied directly via the socket connectors. Never connect an external power supply for the Raspberry via USB-C.

### Electrical Interface
Below you can see the electrical interfaces of the Free Kinematics Kit. There are off-the-shelf cables for the __white__ Molex *Control Socket*. Depending on the desired cable length, you can obtain the following part numbers from the usual distributors:
*  50mm: 151360800
* 100mm: 151360801
* 150mm: 151360802
* 300mm: 151360803
* 450mm: 151360805
* 600mm: 151360806

The following off-the-shelf cables are available for the __two-pole__ *Power Connector* of the Extension Shield:
* 150mm: 2147501021
* 300mm: 2147501022
* 600mm: 2147501023

<br>
The signals of the other connectors are shown below.
<p align="center">
  <img src="/images/Free_Kinematics_Kit_Electrical_Interface_Desc_1920.jpg" width="80%"/>
  <br><em>Fig.: Signal assignment of the white *Control Socket* and the Auxiliary Power Supply Module.</em>
</p>

In addition to the interface of the power supply shield the pinout of the motorcontroller boards is described below. Motors can be either connected on the 2x3 box header or on the 1x6 pin header.

<p align="center">
  <img src="/images/Motorcontroller.jpg" width="80%"/>
</p>

When using a generic motor with the EduArt motorcontroller boards the motor has to be wired correctly to match the above pinout description. Below is the motor wiring diagram of a generic motor.The encoder supply voltage +5V and GND are sensitive to reverse voltage. Pay attention when connecting these wires!

<p align="center">
  <img src="/images/example_motor.jpg" width="60%"/>
</p>


Integrated voltage monitoring of the power management module protects your robot in the event of incorrect operation:
* If the voltage is too low (< 17.5 V), the drives are deactivated after 10 seconds.
* If the voltage remains below 17.5 V for longer than 120 seconds, the system switches off automatically. This protects your battery from deep discharge.
* If the voltage is above 24 V, the drives are deactivated immediately. This prevents you from driving off with a connected power supply unit.

The following diagram shows an example of the logic functions.

<p align="center">
  <img src="/images/simulateEnableLogic/simulateEnableLogic.png" width="80%"/>
</p>

> **Warning:** Despite these protective functions, you must always ensure that the device is used as intended. Never charge the device unattended. Also make sure that no persons are in the immediate vicinity when operating your robot and that the robot cannot fall from a height difference.

### Software Interface
The software is structured in three layers. An independent layer enables communication via a CAN bus. Specific commands are defined via the classes of the robot interface layer. The ROS interface is encapsulated in a single class. For most users, it is sufficient to use the EduDrive class, as the included node edu_drive_node.cpp does.

<p align="center">
  <img src="/doc/edu_drive_classdiagram.png" width="80%"/>
</p>

#### Adding additional CAN devices
Suppose you want to develop your own device and read in this data via the CAN bus. In this case, you add a new class that inherits from the SocketCANObserver class. Below is an example of how the implementation might look.
```c++
  YOURCLASS::YOURCLASS(SocketCAN* can, bool verbosity)
  {
    // Remeber a reference, if you also want to send data via CAN (see method send)
    _can = can;
  
    // Set CAN input ID of device. Your class uses this ID to send data to the device.
    can_frame cf;
    cf.can_id = YOUR_DEVICE_INPUT_ID;
  
    // Set CAN ID that we listen to. Your device is using it as output ID.
    canid_t canidOutput = YOUR_DEVICE_OUTPUT_ID;
    setCANId(canidOutput);
  
    can->registerObserver(this);
  }
  
  void YOURCLASS::notify(struct can_frame* frame)
  {
    // This method is called as soon as messages arrive for the receiver YOUR_DEVICE_OUTPUT_ID
    // The CAN listener must be started before. This is usually done only once.
    // Thus, in most cases outside of the user-defined class, if more than one listener is used.
  }
  
  bool YOURCLASS::send()
  {
    canid_t idTmp = _cf.can_id;
    can_frame cf;
    cf.can_id = YOUR_DEVICE_INPUT_ID;
    cf.can_dlc = LENGTH_OF_YOUR_MSG;
    cf.data[0] = BYTE_1;
    ...
    cf.data[N] = BYTE_N;
    return _can->send(&cf);
  }
```
