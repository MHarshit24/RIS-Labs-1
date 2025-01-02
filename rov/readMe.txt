---FOLDERS AND FILES---

bagfiles: This folder is here for all recorded bags to be stored. As for the base state of this package, it is empty.

cgf: This folder includes configuration files for the auv.py script which launch the auv_master_control node.
- AccelerationControl.cfg: Config. for the acceleration section of the auv_master_control node.
- VelocityControl.cfg: Config. for the velocity section of the auv_master_control node.

launch: This folder includes all relevant launch files.
- auv.launch: This launch file launches the auv_master_control node for task 3. It uses the script auv.py.
- auv_rov.launch: This launch file launches the auv_master_control node for task 4. It uses the script auv_rov.py. The difference with the file from task 3 is a namespace change from "rexrov" to "rov".
- launch_auv.launch: This is the main launch file for task 3.
- launch_rexrov.launch: This is the main launch file for task 2.
- ROV_launcher.launch: This is the main launch file for task 4.

plugins: Holds the plugin for the thrusters
- ThursterPlugin.cc: Thruster plugin.

scripts: This folder holds all relevant scripts.
- auv.py: This script describes the auv_master_control node for task 3, it works by combining all the control chain into one node, it publishes messages for itself to process, and finally it publishes to "/rexrov/thruster_manager/input".
- teleop_rov.py: This script is used as an alternative to the auv.py script to publish force/torque messages for task 4
- thrusters.py: This script describes the node in charge of processing the wrench messages (forces / torques) and publishing them as thruster commands.

urdf: This folder holds the robot's urdf file.
- ROV.urdf: Robot's URDF file.

setup.py: This file is included so that all scripts can be compiled properly with the required dependencies.



---MAIN COMMANDS---

TASK 1:
$ roslaunch uuv_gazebo rexrov_default.launch

TASK 2:
$ roslaunch rov launch_rexrov.launch

TASK 3:
$ roslaunch rov launch_auv.launch

TASK 4:
$ roslaunch rov ROV_launcher.launch



---COMMENT---

When running tasks 2 and 3, after opening both the gazebo and the rviz the terminal starts to spam a warning, do not mind it, as it is expected behavior. ROS handles the conflict internally and that is why it only sends a warning, it is nothing to worry about.
