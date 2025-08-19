Troubleshooting
===============

This section provides an example workflow using `PiPER (AgileX Robotics) <https://global.agilex.ai/products/piper>`_.

.. _system-calibration:

System Calibration
------------------

Currently, there is an unresolved issue with the PiPER hardware.  
If the robot remains disconnected for an extended period, it may accumulate a small positional drift (~millimeters).  
This can cause inaccuracies in experiments that require high-precision manipulation.  

To recalibrate the RoboChemist system (e.g., AUTOHIAM), follow the steps below.

Starting the Robot
~~~~~~~~~~~~~~~~~~

To start the robot:

#. **Enable the Bash environment**

   Open a new terminal and source the PiPER workspace:

   .. code-block:: bash
      
      cd ~/piper_ros_ws &&
      source install/setup.bash

#. **Activate the CAN interface**

   Run the following script to bring up the CAN bus:

   .. code-block:: bash
      
      cd ~/piper_ros_ws/src/piper_ros &&
      bash can_activate.sh can0 1000000

#. **Launch the enable node**

   Run the node to power on and enable the robot:

   .. code-block:: bash
      
      ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2

#. **Start the ROS 2 system**

   In a separate terminal, launch the ROS 2 stack for the robot:

   .. code-block:: bash
      
      cd ~/piper_ros_ws &&
      source install/setup.bash &&
      ros2 launch piper_with_gripper_moveit demo.launch.py

#. **Start the curobo collision-free motion planner**

   In a new terminal, source the python virtual environment and run the cumotion service:

   .. code-block:: bash

      source ~/directory_env/curobo_env/bin/activate &&
      cd ~/autohiam_ws && source install/setup.bash && 
      ros2 run hiam_gen2 cumotion

Starting the Camera
~~~~~~~~~~~~~~~~~~

To start the camera, first run the node to power on the camera ROS 2 stack:

    .. code-block:: bash

       ros2 run realsense2_camera realsense2_camera_node   --ros-args   -p enable_color:=true   -p rgb_camera.color_profile:=1920x1080x30

Then in a seperate terminal, launch the ROS 2 node for marker detection:

    .. code-block:: bash

       source ~/directory_env/handeye_env/bin/activate &&
       cd ~/autohiam_ws &&
       source install/setup.bash &&
       ros2 run marker_detection gripper_estimate_marker_pose

Calibrating the Robot Goal Positions
~~~~~~~~~~~~~~~~~~

To calibrate the goal positions (e.g., for precipitation, washing, and drying), manually control the robot to pick up the sample container. 
Move it to each goal position and adjust the X and Y parameters. 
The goal is to center the container within the beaker and ensure it can move straight up and down without colliding with the beaker walls.

#. **Position the camera to verify the container**

   In a new terminal,
   
   .. code-block:: bash

      cd ~/autohiam_ws &&
      source install/setup.bash &&
      ros2 run hiam_gen2 start_pose

#. **Open the gripper**

   In a new terminal,
   
   .. code-block:: bash

      cd ~/autohiam_ws &&
      source install/setup.bash &&
      ros2 run hiam_gen2 open_gripper

#. **Pick up the container**

   In a new terminal,
   
   .. code-block:: bash

      source ~/directory_env/curobo_env/bin/activate &&
      cd ~/autohiam_ws && source install/setup.bash &&
      ros2 run hiam_gen2 pickup

#. **Move up the container**

   In a new terminal,
   
   .. code-block:: bash

      source ~/directory_env/curobo_env/bin/activate &&
      cd ~/autohiam_ws && source install/setup.bash &&
      ros2 run hiam_gen2 cartesian_control_moveit --ros-args -p z_offset:=0.15

#. **Calibrate the precipitation position**

   In a new terminal, run the following command and adjust the parameters ``-p x:=0.375 -p y:=0.180``:
   
   .. code-block:: bash

      source ~/directory_env/curobo_env/bin/activate &&
      cd ~/autohiam_ws && source install/setup.bash &&
      ros2 run hiam_gen2 moveto --ros-args -p x:=0.375 -p y:=0.180

#. **Calibrate the washing position**

   In a new terminal, run the following command and adjust the parameters ``-p x:=0.375 -p y:=0.075``:
   
   .. code-block:: bash

      source ~/directory_env/curobo_env/bin/activate &&
      cd ~/autohiam_ws && source install/setup.bash &&
      ros2 run hiam_gen2 moveto --ros-args -p x:=0.375 -p y:=0.075

#. **Calibrate the drying position**

   In a new terminal, run the following command and adjust the parameters ``-p x:=0.488 -p y:=0.080``:
   
   .. code-block:: bash

      source ~/directory_env/curobo_env/bin/activate &&
      cd ~/autohiam_ws && source install/setup.bash &&
      ros2 run hiam_gen2 moveto --ros-args -p x:=0.488 -p y:=0.080

#. **Calibrate the drying infusion position**

   In a new terminal, run the following command and adjust the parameters ``-p x:=0.253 -p y:=-0.143 -p z:=0.280``:
   
   .. code-block:: bash

      source ~/directory_env/curobo_env/bin/activate &&
      cd ~/autohiam_ws && source install/setup.bash &&
      ros2 run hiam_gen2 moveto --ros-args -p x:=0.253 -p y:=-0.143 -p z:=0.280