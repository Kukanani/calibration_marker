#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import tf
from geometry_msgs.msg import Point, Quaternion, Pose

from random import random
from math import sin

import transformations

class CalibrationMarker:
    def __init__(self):
        rospy.init_node("calibration_marker", anonymous=True)
        self.parent_frame = rospy.get_param("~parent_frame", "base_link")
        self.child_frame = rospy.get_param("~child_frame", "your_new_frame")

        self.listener = tf.TransformListener()

        self.start_position = Point(0, 0, 0)
        self.start_orientation = Quaternion(0, 0, 0, 1)
        self.pose = Pose()
        self.pose.orientation.w = 1

        # while self.start_position is None and not rospy.is_shutdown():
        #     rospy.logwarn_throttle(5, "waiting for transformation between {} and {}...".format(self.parent_frame, self.child_frame))
        #     try:
        #         (pos, ori) = self.listener.lookupTransform(self.child_frame, self.parent_frame, rospy.Time(0))
        #         self.start_position = Point(*pos)
        #         self.start_orientation = Quaternion(*ori)
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         continue
        # if rospy.is_shutdown():
        #     return

        self.br = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.01), self.cb_publish_tf)

        self.server = InteractiveMarkerServer("calibration_marker")
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Print roslaunch string", callback=self.cb_print_launch)
        self.menu_handler.insert("Print rosrun string", callback=self.cb_print_run)
        self.menu_handler.insert("Reset", callback=self.cb_reset)

        self.marker = self.make_6dof_marker(copy.deepcopy(self.start_position), copy.deepcopy(self.start_orientation))

        self.server.applyChanges()
        rospy.spin()

    #####################################################################
    # Callbacks

    def cb_process_feedback(self, feedback ):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose = feedback.pose
        self.server.applyChanges()

    def cb_print_launch(self, feedback):
        self.print_launch_string()

    def cb_print_run(self, feedback):
        self.print_run_string()

    def cb_reset(self, feedback):
        self.reset_pose()

    def cb_publish_tf(self, msg ):
        self.br.sendTransform((self.pose.position.x,
                               self.pose.position.y,
                               self.pose.position.z),
                              (self.pose.orientation.x,
                               self.pose.orientation.y,
                               self.pose.orientation.z,
                               self.pose.orientation.w),
                              rospy.Time.now(),
                              self.child_frame,
                              self.parent_frame )

    #####################################################################
    # Functionality for menu

    def get_node_args(self):
        r, p, y = transformations.euler_from_quaternion(
            [self.pose.orientation.x,
             self.pose.orientation.y,
             self.pose.orientation.z,
             self.pose.orientation.w])
        return "{0.x} {0.y} {0.z} " \
               "{1} {2} {3} {4} {5} 100" \
               .format(self.pose.position, y, p, r, self.parent_frame, self.child_frame)

    def get_node_name(self):
        return "{}_static_transform_publisher".format(self.child_frame)

    def print_run_string(self):
        print("The following code can be run directly from the terminal to publish this transform:")
        print(
            '\n' +
            'rosrun tf static_transform_publisher __name:={} {}'.format(self.get_node_name(), self.get_node_args()) +
            '\n')

    def print_launch_string(self):
        print("The following code can be added to a roslaunch file (inside the <launch> tag):")
        print(
            '\n' +
            '<node\n' +
            '   pkg="tf"\n' +
            '   type="static_transform_publisher"\n' +
            '   name="{}"\n'.format(self.get_node_name()) +
            '   args="{}"\n'.format(self.get_node_args()) +
            '/>'
            '\n')

    def reset_pose(self):
        # not sure why we have to do this twice. The first two setters reset the tf frame.
        self.marker.pose.position = self.start_position
        self.marker.pose.orientation = self.start_orientation
        # The second set resets the actual RViz marker.
        pose = Pose(position=self.start_position, orientation=self.start_orientation)
        self.server.setPose(self.marker.name, pose)
        self.server.applyChanges()


    #####################################################################
    # Marker Creation

    def make_box(self, msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def make_box_control(self, msg):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.make_box(msg) )
        return control

    def make_6dof_marker(self, position, orientation):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.parent_frame
        int_marker.pose.position = position
        int_marker.pose.orientation = orientation
        int_marker.scale = 1

        int_marker.name = "simple_6dof"
        # int_marker.description = ""

        # insert a box
        control = self.make_box_control(int_marker)
        control.interaction_mode = InteractiveMarkerControl.MENU
        int_marker.controls.append(control)
        # if interaction_mode != InteractiveMarkerControl.NONE:
        #     int_marker.description = "3D Control"
        #     int_marker.description += " + 6-DOF controls"

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.cb_process_feedback)
        self.menu_handler.apply( self.server, int_marker.name )

        return int_marker


def main():
    cm = CalibrationMarker()


if __name__=="__main__":
    main()
