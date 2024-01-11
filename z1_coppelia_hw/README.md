# Universal Robots ROS2 CoppeliaSim
A ros package that implements an hardware interface for using ros controllers in CoppeliaSim.
## Installation
1. Clone this repo in your workspace src folder:  
    ``git clone https://github.com/Hydran00/Universal_Robot_ROS2_CoppeliaSim.git``
2. Build workspace:  
   ``colcon build``
3. Install required messages in CoppeliaSim for ROS2.  
  - Go to your CoppeliaSim install folder (usually in home directory)  
    ``cd``  
  - cd to meta folder of the ros2 interface  
    ``cd programming/ros2_packages/sim_ros2_interface/meta``  
  - Append required message definition to the file:  
    ``geometry_msgs/msg/Wrench``  
    ``geometry_msgs/msg/WrenchStamped``  
    ``std_msgs/msg/MultiArrayDimension``  
    ``std_msgs/msg/MultiArrayLayout``  
    ``std_msgs/msg/Float64MultiArray``  
    ``sensor_msgs/msg/JointState``  
    ``rosgraph_msgs/msg/Clock``  
  - Before building the interface you need to install the following packages:  
  	``sudo apt install xsltproc``  
	``pip install xmlschem``  
  - Now you can build the package:  
      ``cd ..``  
      ``export COPPELIASIM_ROOT_DIR=~/CoppeliaSim``  
      ``colcon build --symlink-install``
## Running the simulation  
1. Open CoppeliaSim and load ``coppelia_world.ttt`` from ``File->Open_Scene`` then click Play. 
2. Run the hardware interface + controllers:  
  ``ros2 launch z1_coppelia_hw ur_coppelia_HWInterface.launch.py``
3. Run ``joint_trajectory_controller``:  
  ``ros2 control switch_controllers --activate joint_trajectory_controller``
4. Test the interface sending trajectory:  
  ``ros2 launch z1_coppelia_hw test_joint_trajectory_controller.launch.py``


