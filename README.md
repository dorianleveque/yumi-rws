<div align="center">
    <img src="docs/YumiRobot.png" alt="drawing" width="200px;"/>
    <h1>Yumi RWS</h1>
</div>

<br/>
<br/>


# 📃 Description of the project

Project realised in the PRI (Computing project) module at the end of the ENIB programme.
The aim of this project is to achieve the following functionalities:
- Retrieve the positions of each joint/articulation of the robot (start, stop this retrieval)
- Move the robot via the position/speed of each joint
- Activate and deactivate the lead-through mode at will
Initially, the tests will be done via the simulator and then on the real robot. Different libraries and packages will have to be tested in order to determine which one is functional and adapted to the problem.

The robot is developed by the company ABB. It is an industrial robot consisting of 2 arms with 7 axes each. The official simulator is called RobotStudio (version 2019.5). It is only available under Windows. It is already installed on a laptop computer. The robot is programmed using RAPID, a proprietary language.

The lead-through mode allows a user to physically move the robot. The robot's joints are therefore soft. The robot simply compensates for gravity.

To control the robot with an external PC, there are 2 technologies: RWS (Robot Web Service, 5Hz) and EGM (Externally Guided Motion, 250Hz). RWS is a REST web technology. The following features are taken from discussions with ABB.

# 📦 Installation

The project was installed and developed on Windows 10 alone. We use a library
additional abb_librws created by the community to facilitate communication with the robot. To facilitate development and compilation, WSL (Windows Subsytem Linux) was used with Ubuntu 16.04. It is a kind of virtual machine directly integrated into Windows without any firewall worries. 

The installation steps are as follows:

On Windows 10:

1. Install the RobotStudio simulator (latest version)
2. Install the RobotWare plugin: 6.07.01 or higher from RoboStudio
3. 3. Configure the simulator with the same parameters described in EGM.
4. If not installed, install WSL with Ubuntu 16.04. To do so, go to the
Windows Store, search for Ubuntu and download version 16.04. One time
install, open the application, a terminal must appear and Ubuntu must be installed.
During the installation, Ubuntu will ask you to fill in a login and
password to define the Root's identifiers (the same identifier to be used by the Root).
(Please contact us for more information when ordering sudo).
Or install a VM as for EGM.

On Linux:
1. Install ROS Kinetic. Follow the installation procedure on the following site
http://wiki.ros.org/kinetic/Installation/Ubuntu.
2. Create a catkin directory, to finish installing ROS
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share
```
3. Install POCO the dependency of abb_librws
```bash
git clone https://github.com/pocoproject/poco
cd poco
mkdir cmake-build
cd cmake-build
cmake ..
make
make install
```
4. Installer la librairie abb_librws

⚠ Methods missing from the library were added during the course of the
development of the project. Check if the setLeadThroughOn methods,
setLeadThroughOff, isLeadThroughOn, loadProgramIntoTask and
loadModuleIntoTask exist in the file rws_interface.cpp of the library
abb_librws. If this is the case, execute the following commands:
```bash
cd /catkin_ws/src
git clone https://github.com/ros-industrial/abb_librws
```
Otherwise, execute them: (fork github)
```bash
cd /catkin_ws/src
git clone https://github.com/dorianleveque/abb_librws
```
After either option, execute the following commands:
```bash
cd /catkin_ws
catkin_make
```
If no errors occur, the installation of the abb_librws library is successful.

Now you can test the code made in the directory of your choice:
```bash
git clone https://github.com/dorianleveque/yumi-rws
cd yumi-rws
```
To create the project makefile
```bash
mkdir build
cd build
cmake .
```

# 📜 Running

To compile and execute the main.cpp:
In the yumi/RWS file
```bash
./run.sh
```
