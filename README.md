# mros_reasoner
A meta-controller implementation for ROS1

## Installation

### Previous steps

- First you need to have a ROS Melodic version with Python3 support, for example following these [instructions](https://answers.ros.org/question/237613/how-to-define-ros-kinetic-to-use-python3-instead-of-python27/?answer=331009#post-id-331009)
_Notes:_
  - For me it worked with `Melodic` and `python3.6-venv`
  - I created `$HOME/rospy3_melodic` for the ROS Melodic version with Python3 support

- **Note:** For regular usage you'll need to make sure to have the virtual environment activated, or things will break.

- you need to install owlready2:
```
pip3 install owlready2
```
- You also need a `java jre` you can install it with:
```
sudo apt-get install openjdk-11-jre
```
### Create reasoner_ws

- We recommend you now create a workspace only for `mros1_reasoner` (and any other packages requiring Python3 in your project), using [workspace overlaying](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying), for example:
```
mkdir -p ~/mros1_reasoner_ws/src
cd mros1_reasoner_ws
```

### Get mros1_reasoner and dependencies using wstool

- You need to copy the `mros1_reasoner` and its dependencies into the Python 3 reasoner workspace (ie: `mros1_reasoner_ws`), so they now exists in both the Python 2 as well as the Python 3 workspaces.

```
$ cd ~/mros1_reasoner_ws
$ wstool init src https://raw.githubusercontent.com/rosin-project/mros1_reasoner/mvp/mros1_reasoner/mros1_reasoner.rosinstall
```

- **IMP Note:** For `mros1_reasoner_ws` you cannot use apt or rosdep install .. (as that will try to use apt and it will install the regular Melodic Python 2 packages, which are invisible to `mros1_reasoner_ws`)
- For the non-Python 3 workspace and its packages you can use apt and `rosdep install ..`

## Build the code

- Once you have the workspace setup, you can build the workspace
- Do not forget to source your Python3 ROS Melodic workspace before building your `mros1_reasoner_ws`
```
source $HOME/rospy3_melodic/devel/setup.bash
catkin init
catkin config --extend $HOME/rospy3_melodic/devel
catkin b
```
- We also advice you to create a separate workspace for "everything else", and only keep the reasoner package in the `mros1_reasoner_ws` workspace.


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
source mros1_reasoner_ws/devel/setup.bash
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
