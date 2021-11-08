# rotors_quadrotor_control

#### Verified Setup On:
* Operating System: Ubuntu 18.04.6 LTS
* ROS Version: Melodic Morenia

#### Additional Packages:
* catkin-tools [for installation](https://catkin-tools.readthedocs.io/en/latest/installing.html)

#### Installation
0. Assuming required setup is done.
  ```
    $ sudo apt-get install ros-melodic-desktop-full ros-melodic-octomap-ros
    $ sudo apt-get install protobuf-compiler libprotobuf-dev libgoogle-glog-dev
  ```
1. Create a ros workspace, if doesn't exist.
  ```
    $ mkdir -p ~/catkin_ws/src
  ```
2. Get the required code.
  ```
    $ cd ~/catkin_ws/src
    ~/catkin_ws/src $ git clone git@github.com:ethz-asl/rotors_simulator.git
    ~/catkin_ws/src $ git clone git@github.com:ethz-asl/mav_comm.git
    ~/catkin_ws/src $ git clone git@github.com:catkin/catkin_simple.git
    ~/catkin_ws/src $ git clone git@github.com:ethz-asl/eigen_catkin.git
  ```
3. Build the ros workspace.
  ```
  
    ~/catkin_ws/src $ cd ~/catkin_ws/
    ~/catkin_ws $ source /opt/ros/melodic/setup.bash
    ~/catkin_ws $ catkin build
  ```
4. 
