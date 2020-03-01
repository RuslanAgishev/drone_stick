# DroneStick

Flight code to control a swarm of [Crazyflie](https://www.bitcraze.io/crazyflie-2/)
drones via externally tracked controllers.

## Example usage
The flight software from this repository requires external positioning system for drones to fly.
It could be a motion capture system, for example Vicon.
The external controller could be any object with known position and orientation, for example,
you can use a VR controller (its pose is estimated by HTC Vive base stations) or even another drone, tracked by motion capture system.

### Vicon motion capture
In case if you use a motion capture system for crazyflie localization, install a ROS driver,
[crazyflie_ros](https://github.com/whoenig/crazyflie_ros), and [vicon_bridge](https://github.com/ethz-asl/vicon_bridge.git)
follow the instructions bellow:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/vicon_bridge.git
git clone https://github.com/whoenig/crazyflie_ros.git
cd crazyflie_ros
git submodule init
git submodule update
```
Build package inside the workspace.
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```
Clone the repository.
```bash
cd ~/Desktop/
git clone https://github.com/RuslanAgishev/drone_stick
```
Adjust drone URI you use in the launch file [here](https://github.com/RuslanAgishev/drone_stick/tree/master/flight_code/launch/connect1.launch#L3).

And use our repository alongside with crazyflie_ros driver.
```
cp -r ~/Desktop/drone_stick/flight_code/scripts/interactive_control_vr ~/catkin_ws/src/crazyflie_ros/crazyflie_demo/scripts/
cp ~/Desktop/drone_stick/flight_code/launch/connect1.launch ~/catkin_ws/src/crazyflie_ros/crazyflie_demo/launch
```

### Interactive control example

Here we map a [VR controller](https://www.vive.com/ru/accessory/controller/) movements to drone position commands.
In order to obtain VR controller pose information, follow the next steps:

1. Install HTC Vive SteamVR.
2. ```pip2 install openvr``` OR [download](https://github.com/cmbruns/pyopenvr/releases) the installer.
3. Setup VR environment. You can follow an [example](https://wiki.bitcraze.io/doc:lighthouse:setup) from Bitcraze.

Connect to crazyflie:
```
roslaunch crazyflie_demo connect1.launch
```
And run the flight node:
```
rosrun crazyflie_demo interactive_control_vr.py
```
