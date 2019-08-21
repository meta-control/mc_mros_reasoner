# mros_reasoner
A meta-controller implementation for ROS1

## Installation
First you need to have a ROS Kinetic version with Python3 support, for example following these [instructions](https://answers.ros.org/question/237613/how-to-define-ros-kinetic-to-use-python3-instead-of-python27/?answer=331009#post-id-331009)

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
Activate your virtual environment, source your ws and launch the reasoner:
```
$ source venv3.5_ros/bin/activate
$ source $YOUR_WS/devel/setup.bash
$ roslaunch mros1_reasoner run.launch
```
