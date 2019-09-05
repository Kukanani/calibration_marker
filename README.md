# calibration_marker
A ROS interactive marker that quickly generates static_transform_publisher transformations.

I use this package to generate quick-and-dirty transformations, usually between a robot and its environment.

## Dependencies
 - ROS (tested on Kinetic, but should work on Indigo and Melodic at least)
 - RViz
 - interactive_markers (i.e. `sudo apt install ros-kinetic-interactive-markers`)

## Installation
- Clone into your catkin workspace source folder.
- Build with `catkin_make` or `catkin build`.

## Usage
Before starting, run `roscore` and RViz. If you're trying to calibrate between a robot and a camera, you
should also launch the camera driver, add a point cloud in RViz, run MoveIt for your robot, and add MotionPlanning
in RViz. This will allow you to (roughly) align the viewed point cloud with the known robot position.

To start, run the calibration marker:
```
rosrun calibration_marker calibration_marker.py _parent_frame:=base_link _child_frame:=camera_link
```

Launch file version:

    <node
      pkg="calibration_marker"
      type="calibration_marker.py"
      name="calibration_marker"
    />
      <arg name="parent_frame" value="j2s7s300_link_base" />
      <arg name="child_frame" value="camera_link" />
    </node>


Add an interactive marker in RViz and connect it to the `/calibration_marker/update` topic. You should now be able
to drag the marker around (and see the camera points move with it if you're doing camera calibration)

Once you're done moving the marker to align your frame how you want it, right click the marker and use the Print
menu items to print out a `rosrun` command or a `<node>` tag for your roslaunch file.

You can also reset the marker from the context (right-click) menu.
