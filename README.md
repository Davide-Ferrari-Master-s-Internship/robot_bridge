# robot_bridge

Simple explanation of your package

## Getting Started

Node that interface directly with robot.

There is an upper level node that split a custom trajectory message (manipulator + mobile base) in a trajectory message for maipulator and a velocity message for mobile base.

The other two nodes use thoose message to control directly the manipulator and the mobile base.

If the maximum joint acceleration limit error is displayed, you need to set the acceleration limits for each joint in the configuration file in the prbt_support package by editing the file:

```prbt_support/config/manipulator_controller.yaml```

## Running the tests

Run PRBT6 Bridge:

```roslaunch robot_bridge prbt_bridge.launch```

Run MPO_500 Bridge:

```roslaunch robot_bridge mpo_500_bridge.launch```

Run PRBT6 + MPO 500 Bridge:

```roslaunch robot_bridge prbt+mpo_bridge.launch```

## Version

* **ROS:** Melodic+

## Authors

* **Davide Ferrari**
