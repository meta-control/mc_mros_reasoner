# mros_reasoner

A meta-controller implementation for ROS1

## Installation

### Previous steps

- you need to install a `java jre` you can install it with:

```console
sudo apt-get install openjdk-11-jre
```

### Create reasoner_ws

- We recommend you to create a workspace only for `mros1_reasoner`, for example:

```console
  mkdir -p ~/mros1_reasoner_ws/src
  cd mros1_reasoner_ws
```

### Get mros1_reasoner and dependencies using wstool

- You need to copy the `mros1_reasoner` and its dependencies into the reasoner workspace (ie: `mros1_reasoner_ws`).

```console
  cd ~/mros1_reasoner_ws
  wstool init ~/mros1_reasoner_ws/src https://raw.githubusercontent.com/tud-cor/mc_mros_reasoner/master/mros1_reasoner/mros1_reasoner.rosinstall
  rosdep install --from-paths ~/mros1_reasoner_ws/src -y -i -r --skip-keys="abb_rws_interface"
```

**Note** The above `rosdep install` uses the `-r` argument in order to ignore possible errors. Please check the console output to make sure all dependencies are installed correctly.

### Build the code

- Once you have the workspace setup, you can build the workspace
- Do not forget to source ROS Melodic workspace before building your `mros1_reasoner_ws`

```console
source /opt/ros/melodic/setup.bash
catkin build
```

### Known issues

If you run into an issue with:

`sudo apt install python3-venv libpython3-dev python-catkin-tools`

try the following:

```console
sudo apt clean
sudo apt update
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt clean && sudo apt update
```

## User instructions

The mros1_reasoner node needs the following elements:

- an OWL model of the system to be metacontrolled. This can be manually created directly in OWL, e.g. using Protege, or it can be generated automatically from other models of the system ([rosin-experiments](https://github.com/rosin-project/rosin-experiments) provides a way to develop the model according to RosModel, and the script [`rosmodel2owl.py`](https://github.com/tud-cor/mc_mros_reasoner/blob/master/mros1_reasoner/scripts/rosmodel2owl.py) provides a way to transform it to OWL)
- monitoring and reconfiguration infrastructure for the given platform, e.g. for ROS1 reconfiguration capabilities are provided by [`ros_manipulator.py`](https://github.com/rosin-project/metacontrol_sim/blob/master/scripts/rosgraph_manipulator.py) and by the launchfiles in [metacontrol_move_base_configurations](https://github.com/rosin-project/metacontrol_move_base_configurations)

### Execution

Source your ws and launch the reasoner:

```console
source mros1_reasoner_ws/devel/setup.bash
roslaunch mros1_reasoner run.launch onto:=kb.owl
```

### Testing

#### _To Do_
