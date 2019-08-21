# mros_reasoner
A meta-controller implementation for ROS1

## Installation

mros_reasoner requires Python 3. For this we need to build some ROS core libraries from source with Python 3. You can do this following the instructions below (**Note:** you need a running ROS installation already in your system):
```
$ mkdir $HOME/ros_kinetic_py3
$ cd $HOME/ros_kinetic_py3
$ sudo rosdep init
$ rosdep update
$ rosdep check --from-paths src/ -i

$ apt install python3-venv libpython3-dev python-catkin-tools
```

Create and activate your python virtual environment to work without affecting your standard Python installation:
```
$ python3.5 -m venv venv3.5_ros
$ source venv3.5_ros/bin/activate
```

Build ros_base libraries:
```
$ pip3 install wheel
$ pip3 install -U rosdep rosinstall_generator wstool rosinstall

$ rosinstall_generator --deps --tar --rosdistro=kinetic ros_base > ros_base.rosinstall
$ wstool init src -j8 ros_base.rosinstall

$ catkin build -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3.5
```

### Known issues
If you run into an issue with:
`$ sudo apt install python3-venv libpython3-dev python-catkin-tools`
try the following:
```
$ sudo apt clean
$ sudo apt update
$ sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt clean && sudo apt update
```

## Execution
```
$ source venv3.5_ros/bin/activate
$ source $YOUR_WS/devel/setup.bash
$ roslaunch mros1_reasoner run.launch
