# mros_reasoner

A meta-controller implementation for ROS2

### Note:

**This branch is highly experimental and unstable**

## Installation

### Create mros_reasoner workspace   

- We recommend you to create a workspace only for `mros_reasoner`, for example:

```console
  mkdir -p ~/mros_reasoner_ws/src
  cd mros_reasoner_ws
```

### Get mros2_reasoner

- You need to clone the `mc_mros_reasoner` into the reasoner workspace (ie: `mros_reasoner_ws`).
- Make sure to change to the branch `humble`

```console
  cd ~/mros_reasoner_ws/src
  git clone --branch humble https://github.com/meta-control/mc_mros_reasoner.git
```

Install deps:
```console
  cd ~/mros_reasoner_ws/
  rosdep install --from-paths src --ignore-src -r -y
```

### Build MROS2 workspace

- Once you have the workspace setup, you can build the workspace
- Do not forget to source ROS humble workspace before building your `mros_reasoner_ws`

```console
cd ~/mros_reasoner_ws/src
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Example - Pilot URJC

- To test the code, get the Turtlebot3 Pilot from https://github.com/MROS-RobMoSys-ITP/Pilot-URJC
- **Make sure you check the** metacontrol-test-integration **branch**
- Put the packages in the same ws (i.e. `~/mros_reasoner_ws/`)
- Follow the instructions on how to build the packages there

**Disclaimer:** The example might not work for all ros2 versions supported by MROS. Check the example page to find out which versions are supported.

### Build workspace with example

Make sure to rebuild the workspace after you get the example code.

```console
cd ~/mros_reasoner_ws/src
source /opt/ros/hubmle/setup.bash
colcon build --symlink-install
```

### Execution - with simulated turtlebot3


1. **Launch turtlebot3 world in gazebo sim**

    ```console
      export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:[ros2_ws]/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
      export TURTLEBOT3_MODEL=${TB3_MODEL}
      ros2 launch pilot_urjc_bringup tb3_sim_launch.py
    ```
  - After the last command, the Gazebo simulator is running in background. Don't worry if no window is opened.

2. **Turtlebot3 Navigation launcher**

    This launcher includes rviz, nav2, amcl, map-server, **[system-modes](https://github.com/micro-ROS/system_modes)**, etc.
    The **system_modes mode_manager** takes the modes description from `params/pilot_modes.yaml`.


    ```console
      export TURTLEBOT3_MODEL=${TB3_MODEL}
      ros2 launch pilot_urjc_bringup nav2_turtlebot3_launch.py
    ```
  - RVIz opens, and the navigation system is waiting for the activation of the laser_driver. It is not necessary to set an initial robot position with the 2D Pose Estimate tool. When the laser_driver is up, the pose will be set automatically.


3. **Launch the mros2 metacontroller**

    This step launches the `mros2_metacontroller`, it launches by default the `kb.owl` ontology and connects to the
    system_modes created by the pilot_urjc
    - The names of the modes there have been changed to match the `fd` names of the `kb.owl` ontology.


    ```console
      ros2 launch mros2_reasoner launch_reasoner.launch.py
    ```

By default it sets the `f3_v3_r1` mode which corresponds to the NORMAL mode.

With all the above, we will have enough to test some navigation actions and experiment changing the current mode and seeing how this change affects the navigation.


4. **Publish energy values**

- To get a reconfiguration, we need to Publish some fake qa_values. This node sends a [`DiagnosticArray`](http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticArray.html) message, with increasing energy value.


    ```console
      ros2 run mros2_reasoner mros2_publish_qa_node
    ```
    ```  
