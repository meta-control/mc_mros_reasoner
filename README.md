# mros_reasoner
A meta-controller implementation for ROS1

## Installation
- First you need to have a ROS Melodic version with Python3 support, for example following these [instructions](https://answers.ros.org/question/237613/how-to-define-ros-kinetic-to-use-python3-instead-of-python27/?answer=331009#post-id-331009)
_Notes:_
  - For me it worked with `Melodic` and `python3.6-venv`
  - I created `$HOME/rospy3_melodic` for the ROS Melodic version with Python3 support

- **Note:** For regular usage you'll need to make sure to have the virtual environment activated, or things will break.
- you need to install owlready2:
```
pip3 install owlready2
```
- We recommend you now create a workspace only for `mros1_reasoner` (and any other packages requiring Python3 in your project), using [workspace overlaying](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying), for example:
```
mkdir -p abb_metacontrol_ws/src
cd abb_metacontrol_ws
```
- Do not forget to source your ROS Melodic workspace before building your `MROS1_REASONER_WS`
```
source $HOME/rospy3_melodic/devel/setup.bash
catkin init
catkin config --extend $HOME/ros_kinetic_py3/devel
catkin b
```
- We also advice you to create a separate workspace for "everything else", and only keep the reasoner package in the `abb_metacontrol_ws` workspace.
- **IMP Note:** For `abb_metacontrol_ws` you cannot use apt or rosdep install .. (as that will try to use apt and it will install the regular Kinetic Python 2 packages, which are invisible to `abb_metacontrol_ws`)
- For the non-Python 3 workspace and its packages you can use apt and `rosdep install ..`
- Since `mros1_reasoner` depends on `cheops_system_state_msgs` and `cheops_graph_manipulation_msgs`, you will need to copy these packages into the Python 3 workspace (ie: `abb_metacontrol_ws`), so they now exists in both the Python 2 as well as the Python 3 workspaces.

### Known issues
If you run into an issue with:
`sudo apt install python3-venv libpython3-dev python-catkin-tools`
try the following:
```
sudo apt clean
sudo apt update
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt clean && sudo apt update
```

## Execution
Activate your virtual environment, source your ws and launch the reasoner:
```
source venv3.5_ros/bin/activate
source $YOUR_MROS1_REASONER_WS/devel/setup.bash
roslaunch mros1_reasoner run.launch
```

## Testing
The `mros1_reasoner` can be tested with the cheops_mock_diagnostics. In another terminal run:
```
source $YOUR_CHEOPS_WS/devel/setup.bash
roslaunch cheops_mock_diagnostics mock_metacontrol_monitoring_input.launch
```

and then you can simulate failures in some components of the system by running for example (in yet another terminal):
```
rosservice call /mock_camera/is_in_error "data: true"
```
