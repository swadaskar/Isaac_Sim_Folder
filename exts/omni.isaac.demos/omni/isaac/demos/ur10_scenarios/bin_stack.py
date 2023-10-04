# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import random
import numpy as np

from pxr import Gf, UsdGeom
from enum import Enum
import omni
import carb
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.utils._isaac_utils import math as math_utils
from omni.isaac.demos.utils.world import World
from omni.isaac.demos.utils.ur10 import UR10, default_config
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper_Properties

from .scenario import set_translate, set_rotate, create_ur10, Scenario, create_background, create_objects, setup_physics
from copy import copy

import omni.physx
from collections import deque


class SM_events(Enum):
    START = 0
    WAYPOINT_REACHED = 1
    GOAL_REACHED = 2
    ATTACHED = 3
    DETACHED = 4
    TIMEOUT = 5
    STOP = 6
    BROKENGRIP = 7

    NONE = 8  # no event ocurred, just clocks


class SM_states(Enum):
    STANDBY = 0  # Default state, does nothing unless enters with event START

    PICKING = 1
    ATTACH = 2
    PLACING = 3
    DETACH = 4
    FLIPPING = 5


statedic = {0: "orig", 1: "axis_x", 2: "axis_y", 3: "axis_z"}


class PickAndPlaceStateMachine(object):
    """
    Self-contained state machine class for Robot Behavior. Each machine state may react to different events,
    and the handlers are defined as in-class functions
    """

    def __init__(self, stage, robot, ee_prim, target_bodies, default_position, bin_holder_object):
        self.robot = robot
        self.dc = robot.dc
        self.end_effector = ee_prim
        self.end_effector_handle = None
        self._stage = stage
        self.current = None
        self.target_bodies = target_bodies

        self.start_time = 0.0
        self.start = False
        self._time = 0.0
        self.default_timeout = 0.5
        self.bin_holder_object = bin_holder_object
        self.default_position = copy(default_position)
        self.target_position = default_position
        self.reset = False
        self.waypoints = deque()
        self.thresh = {}
        # Threshold to clear waypoints/goal
        # (any waypoint that is not final will be cleared with the least precision)
        self.precision_thresh = [
            [0.0005, 0.0025, 0.0025, 0.0025],
            [0.0005, 0.005, 0.005, 0.005],
            [0.05, 0.2, 0.2, 0.2],
            [0.08, 0.4, 0.4, 0.4],
            [0.18, 0.6, 0.6, 0.6],
        ]
        self.add_bin = None

        # Event management variables

        # Used to verify if the goal was reached due to robot moving or it had never left previous target
        self._is_moving = False
        self._attached = False  # Used to flag the Attached/Detached events on a change of state from the end effector
        self._detached = False
        self._upright = False  # Used to indicate if the bin is being picked facing up, so the proper state is called
        self._flipped = False
        self._closed = False

        # Constants for lifting, fliping, and intermediary goals
        self.upside_goal = _dynamic_control.Transform()
        self.upside_offset = _dynamic_control.Transform()
        self.upside_flip = _dynamic_control.Transform()

        self.upside_goal.p = (0.80808, 0.70978, -0.03)
        self.upside_goal.r = (0.12380744620354063, -0.510572739737588, -0.1093972128961598, 0.8438124456962951)

        self.upside_offset.p = [0.51855, 0.9221, 0.1370]
        self.upside_offset.r = self.upside_goal.r

        self.upside_flip.p = [0.51855, 0.9221, 0.1370]
        self.upside_flip.r = self.upside_goal.r
        # flip 180 degrees along y axis

        self.pick_count = 0
        # Define the state machine handling functions
        self.sm = {}
        # Make empty state machine for all events and states
        for s in SM_states:
            self.sm[s] = {}
            for e in SM_events:
                self.sm[s][e] = self._empty
                self.thresh[s] = 0

        # Use a same event handler for broken grip on all events
        for s in SM_states:
            self.sm[s][SM_events.BROKENGRIP] = self._all_broken_grip

        # Fill in the functions to handle each event for each status
        self.sm[SM_states.STANDBY][SM_events.START] = self._standby_start
        self.sm[SM_states.STANDBY][SM_events.GOAL_REACHED] = self._standby_goal_reached
        self.thresh[SM_states.STANDBY] = 3

        self.sm[SM_states.PICKING][SM_events.GOAL_REACHED] = self._picking_goal_reached
        self.sm[SM_states.PICKING][SM_events.NONE] = self._picking_no_event
        self.thresh[SM_states.PICKING] = 1

        self.sm[SM_states.FLIPPING][SM_events.GOAL_REACHED] = self._flipping_goal_reached
        self.thresh[SM_states.FLIPPING] = 2

        self.sm[SM_states.PLACING][SM_events.GOAL_REACHED] = self._placing_goal_reached
        self.thresh[SM_states.PLACING] = 0

        self.sm[SM_states.ATTACH][SM_events.GOAL_REACHED] = self._attach_goal_reached
        self.sm[SM_states.ATTACH][SM_events.ATTACHED] = self._attach_attached
        self.thresh[SM_states.ATTACH] = 0

        self.sm[SM_states.DETACH][SM_events.GOAL_REACHED] = self._detach_goal_reached
        self.sm[SM_states.DETACH][SM_events.DETACHED] = self._detach_detached
        self.thresh[SM_states.DETACH] = 0

        self.current_state = SM_states.STANDBY
        self.previous_state = -1
        self._physx_query_interface = omni.physx.get_physx_scene_query_interface()

        x = [1.00, 0.79, 0.58]
        y = [-0.62, -0.31, 0]
        self.stack_coordinates = np.array(
            [
                [x[0], y[0]],
                [x[1], y[0]],
                [x[0], y[1]],
                [x[2], y[0]],
                [x[1], y[1]],
                [x[0], y[2]],
                [x[2], y[1]],
                [x[1], y[2]],
                [x[2], y[2]],
            ]
        )
        self.stack_size = np.zeros([9, 1])
        self.stack_transition = ((3, 6, 8), (1, 3, 6))
        self.current_stack_list = [0, 0]
        self.current_stack = 0

        self.total_bins = 0

    # Auxiliary functions

    def get_current_place_pose(self):
        """
        Gets the (x,y) coordinates for the current stack placement
        """
        while self.stack_size[self.current_stack_list[self.current_stack]] > 3:
            self.advance_stack()
        return self.stack_coordinates[self.current_stack_list[self.current_stack]]

    def advance_stack(self):
        """
        Moves to next stack, prioritizing filling the left-bottom-most stack, but maintaining
        a single bin ahead of the other stacks
        """
        self.current_stack_list[self.current_stack] = (self.current_stack_list[self.current_stack] + 1) % 9
        if self.current_stack_list[self.current_stack] in self.stack_transition[self.current_stack]:
            self.current_stack = (self.current_stack + 1) % 2

    def _empty(self, *args):
        """
        Empty function to use on states that do not react to some specific event
        """
        pass

    def change_state(self, new_state):
        """
        Function called every time a event handling changes current state
        """
        self.current_state = new_state
        self.start_time = self._time
        carb.log_warn(str(new_state))

    def goalReached(self):
        """
        Checks if the robot has reached a certain waypoint in the trajectory
        """
        if self._is_moving:
            state = self.robot.end_effector.status.current_frame
            target = self.robot.end_effector.status.current_target
            error = 0
            for i in [0, 2, 3]:
                k = statedic[i]
                state_v = state[k]
                target_v = target[k]
                error = np.linalg.norm(state_v - target_v)
                # General Threshold is the least strict
                thresh = self.precision_thresh[-1][i]
                # if the target is a goal point, use the defined threshold for the current state
                if len(self.waypoints) == 0:
                    thresh = self.precision_thresh[self.thresh[self.current_state]][i]

                if error > thresh:
                    return False
            self._is_moving = False
            return True
        return False

    def get_current_state_tr(self):
        """
        Gets current End Effector Transform, converted from Motion position and Rotation matrix
        """
        # Gets end effector frame
        state = self.robot.end_effector.status.current_frame

        orig = state["orig"]

        mat = Gf.Matrix3f(
            *state["axis_x"].astype(float), *state["axis_y"].astype(float), *state["axis_z"].astype(float)
        )
        q = mat.ExtractRotation().GetQuaternion()
        (q_x, q_y, q_z) = q.GetImaginary()
        q = [q_x, q_y, q_z, q.GetReal()]
        tr = _dynamic_control.Transform()
        tr.p = list(orig)
        tr.r = q
        return tr

    def ray_cast(self, x_offset=0.0015, y_offset=0.03, z_offset=0.0):
        """
        Projects a raycast forward from the end effector, with an offset in end effector space defined by (x_offset, y_offset, z_offset)
        if a hit is found on a distance of 100 centimiters, returns the object usd path and its distance
        """
        tr = self.get_current_state_tr()
        offset = _dynamic_control.Transform()
        offset.p = (x_offset, y_offset, z_offset)
        raycast_tf = math_utils.mul(tr, offset)
        origin = raycast_tf.p
        rayDir = math_utils.get_basis_vector_x(raycast_tf.r)
        hit = self._physx_query_interface.raycast_closest(origin, rayDir, 1.0)
        if hit["hit"]:
            usdGeom = UsdGeom.Mesh.Get(self._stage, hit["rigidBody"])
            distance = hit["distance"]
            return usdGeom.GetPath().pathString, distance
        return None, 10000.0

    def get_target(self):
        """
        Indicates if there is any bin at the bottom of the conveyor belt, if there is, the current target object is set
        with the found value
        """
        origin = (-0.360, 0.440, -0.500)
        rayDir = (1, 0, 0)
        hit = self._physx_query_interface.raycast_closest(origin, rayDir, 1.0)
        if hit["hit"]:
            self.current = hit["rigidBody"]
            return True
        self.current = None
        return False

    def lerp_to_pose(self, pose, n_waypoints=1):
        """
        adds spherical linear interpolated waypoints from last pose in the waypoint list to the provided pose
        if the waypoit list is empty, use current pose
        """
        if len(self.waypoints) == 0:
            start = self.get_current_state_tr()
            # start.p = math_utils.mul(start.p, 0.01)
        else:
            start = self.waypoints[-1]

        if n_waypoints > 1:
            for i in range(n_waypoints):
                self.waypoints.append(math_utils.slerp(start, pose, (i + 1.0) / n_waypoints))
        else:
            self.waypoints.append(pose)

    def move_to_zero(self):
        """
        clears the robot target, so it returns to its rest pose
        """
        self._is_moving = False
        self.robot.end_effector.go_local(
            orig=[], axis_x=[], axis_y=[], axis_z=[], use_default_config=True, wait_for_target=False, wait_time=5.0
        )

    def move_to_target(self):
        """
        moves the end effector to the current target pose
        """
        xform_attr = self.target_position
        self._is_moving = True

        orig = np.array([xform_attr.p.x, xform_attr.p.y, xform_attr.p.z])
        axis_y = np.array(math_utils.get_basis_vector_y(xform_attr.r))
        axis_z = np.array(math_utils.get_basis_vector_z(xform_attr.r))
        self.robot.end_effector.go_local(
            orig=orig,
            axis_x=[],
            axis_y=axis_y,
            axis_z=axis_z,
            use_default_config=True,
            wait_for_target=False,
            wait_time=5.0,
        )

    def get_target_to_object(self, offset_up=0.25, offset_down=0.25):
        """
        Gets target pose to end effector on a given target, with an offset on the end effector actuator direction given
        by [offset_up, offset_down]
        """
        offset = _dynamic_control.Transform()
        offset.p.z = offset_up
        offset.r = (0, 0.7071, 0, 0.7071)
        body_handle = self.dc.get_rigid_body(self.current)
        obj_pose = self.dc.get_rigid_body_pose(body_handle)
        offset_1 = _dynamic_control.Transform()
        tr = self.get_current_state_tr()
        rx = math_utils.dot(math_utils.get_basis_vector_y(obj_pose.r), math_utils.get_basis_vector_y(tr.r))
        if math_utils.get_basis_vector_z(obj_pose.r).z > 0:
            self._upright = True
            if rx < 0:  # rotate target by 180 degrees on z axis
                offset_1.r = (0, 0, 1, 0)
        else:  # If bin is upside down, pick by bottom
            offset.p.z = offset_down
            if rx < 0:  # rotate target by 180 degrees on z axis
                offset_1.r = (1, 0, 0, 0)
            else:
                offset_1.r = (0, -1, 0, 0)
            self._upright = False
        target_position = math_utils.mul(math_utils.mul(obj_pose, offset_1), offset)
        # target_position.p = math_utils.mul(target_position.p, 0.01)
        return target_position

    def set_target_to_object(self, offset_up=0.25, offset_down=0.25, n_waypoints=1, clear_waypoints=True):
        """
        Clears waypoints list, and sets a new waypoint list towards the target pose for an object.
        """
        target_position = self.get_target_to_object(offset_up, offset_down)
        # linear interpolate to target pose
        if clear_waypoints:
            self.waypoints.clear()
        self.lerp_to_pose(target_position, n_waypoints=n_waypoints)
        # Get first waypoint target
        self.target_position = self.waypoints.popleft()

    def step(self, timestamp, start=False, reset=False):
        """
            Steps the State machine, handling which event to call
        """
        if self.current_state != self.previous_state:
            self.previous_state = self.current_state
        if not self.start:
            self.start = start
        # Process events
        if reset:
            self.current_state = SM_states.STANDBY
            self.robot.end_effector.gripper.open()
            self._closed = False
            self.start = False
            self._upright = False
            self.waypoints.clear()
            self.target_position = self.default_position
            self.move_to_target()
            self.current_stack_list = [-1, 0]
            self.current_stack = 0
            self.total_bins = 0
            self.stack_size *= 0
            self.current = None
        elif self._closed and not self.robot.end_effector.gripper.is_closed():
            self.sm[self.current_state][SM_events.BROKENGRIP]()
        elif self.goalReached():
            if len(self.waypoints) == 0:
                self.sm[self.current_state][SM_events.GOAL_REACHED]()
            else:
                self.target_position = self.waypoints.popleft()
                self.move_to_target()
                self.start_time = self._time
        elif self.current_state == SM_states.STANDBY and self.start and self.get_target():
            self.sm[self.current_state][SM_events.START]()
        elif self._attached:
            self._attached = False
            self.sm[self.current_state][SM_events.ATTACHED]()
        elif self._detached:
            self._detached = False
            self.sm[self.current_state][SM_events.DETACHED]()
        elif self._time - self.start_time > self.default_timeout:
            self.sm[self.current_state][SM_events.TIMEOUT]()
        else:
            self.sm[self.current_state][SM_events.NONE]()

    # Event handling functions. Each state has its own event handler function depending on which event happened

    def _standby_start(self, *args):
        """
        Handles the start event when in standby mode.
        Proceeds to pick up the next bin on the queue, and set the arm
        to move towards the bin from current  position.
        switches to picking state.
        """
        if self.total_bins < 36:
            # Tell motion planner controller to ignore current object as an obstacle
            self.target_bodies[self.current].suppress()
            self.pick_count = 0
            self.lerp_to_pose(self.default_position, 1)
            self.lerp_to_pose(self.default_position, 60)
            # set target above the current bin with offset of 20 cm
            self.set_target_to_object(0.12, 0.20, clear_waypoints=False)
            # start arm movement
            self.move_to_target()
            # Move to next state
            self.change_state(SM_states.PICKING)

            self._flipped = False

    def _standby_goal_reached(self, *args):
        """
        Finished processing a bin, moves up the stack position for next bin placement
        """
        self.move_to_zero()
        self.advance_stack()
        self.start = True

    def _flipping_goal_reached(self, *args):
        """
        Reached the goal base pose for flipping.
        sets the robot to flip the arm to the bin upside pose, and towards the placement goal in the platform.
        Sets the next state as Detach.
        """

        self.lerp_to_pose(self.upside_flip, n_waypoints=1)
        target_position = self.upside_offset
        self.lerp_to_pose(target_position, 1)
        self.lerp_to_pose(self.upside_goal, 80)
        self.lerp_to_pose(self.upside_goal, 30)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.change_state(SM_states.DETACH)
        self._flipped = True
        if self.add_bin is not None:
            self.add_bin()

    def _attach_goal_reached(self, *args):
        """
        Handles a state machine step when the target goal is reached, and the machine is on attach state
        """
        self.robot.end_effector.gripper.close()
        self._closed = True
        self.lerp_to_pose(self.target_position, 60)  # Wait 1 second in place for attachment
        if self.robot.end_effector.gripper.is_closed():
            self._attached = True
        else:  # Failed to attach so return grasp to try again
            # move up 25 centimiters and return to picking state
            offset = _dynamic_control.Transform()
            offset.p = (-0.25, 0.0, 0.0)
            self.target_position = math_utils.mul(self.target_position, offset)
            self.move_to_target()
            self.change_state(SM_states.PICKING)

    def _attach_attached(self, *args):
        """
        Handles a state machine step when the target goal is reached, and the machine is on attach state
        """
        offset = _dynamic_control.Transform()
        offset.p = (-0.20, 0.0, 0.0)
        target = math_utils.mul(self.target_position, offset)
        self.lerp_to_pose(target, 10)
        if not self._upright:
            target.p.z = 0.15
            target.r = [0, 0.7071, 0, 0.7071]
            self.lerp_to_pose(target, 10)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.pick_count += 1
        if self._upright:
            # Add additional waypoints to hit before flipping
            base_flip = _dynamic_control.Transform()
            base_flip.p = [0, 0.7, 0]
            base_flip.r = [0, 0.7071, 0, 0.70701]
            self.lerp_to_pose(base_flip, 25)
            self.change_state(SM_states.FLIPPING)
        else:
            target = _dynamic_control.Transform()
            x, y = self.get_current_place_pose()
            target.r = [0, 0.35836791862248063, 0, 0.9335804383673595]
            target.p.x = x
            target.p.y = y
            target.p.z = 0.15
            # If bin was picked from the bottom directly from the conveyor belt, send it to an intermediary waypoint
            # before sending it to its target so it clears the robot base
            if self.pick_count == 1:
                target.p.x = 0.5
                target.p.y = 0.5
                self.lerp_to_pose(target, 25)
                target.p.y = 0
                self.lerp_to_pose(target, 25)
                target.p.x = x
                target.p.y = y
                if self.add_bin is not None:
                    self.add_bin()

            self.lerp_to_pose(target, 50)
            self.lerp_to_pose(target, 60)
            target.r = [0, 0.7071, 0, 0.7071]
            self.lerp_to_pose(target, 1)
            self.lerp_to_pose(target, 30)
            self.change_state(SM_states.PLACING)

    def _detach_goal_reached(self, *args):
        """
        Handles the goal reached event while in detach state.
        If the bin is still fixed to the robot, opens the gripper and waits in place for it to open.
        Then sets the next state which may be picking the bin again from the bottom, if it was upright, otherwise
        sets it to the standby pose and state
        """
        if self.robot.end_effector.gripper.is_closed():
            self.robot.end_effector.gripper.open()
            self._closed = False
            # Lerp to its same pose to wait a few timesteps before entering this event again.
            self.lerp_to_pose(self.target_position, n_waypoints=4)
            self._detached = True
            self.thresh[SM_states.DETACH] = 3
        else:
            if self._upright:
                self.bin_holder_object.unsuppress()
                offset = _dynamic_control.Transform()
                offset.p = (0.5, 0.517, 0.10)
                offset.r = (0.14102078714018185, 0.6810133229625428, 0.059975620045362985, 0.7160565037356013)
                self.lerp_to_pose(copy(offset), 10)
                offset.p = [0.9912, 0.517, 0.2900]
                self.lerp_to_pose(offset, 20)
                self.lerp_to_pose(offset, 60)  # wait in place for 1 second
                self.target_position = self.waypoints.popleft()
                self.move_to_target()
                target_to_obj = self.get_target_to_object(0.18, 0.20)
                offset2 = _dynamic_control.Transform()
                offset2.p = (0.0, 0.0, 0.20)
                offset2.r = (0, 0, 0, 1)
                pre_target = math_utils.mul(offset2, target_to_obj)
                self.lerp_to_pose(copy(pre_target), 20)
                offset2.p.x = 0.0
                offset2.p.z = 0.0
                self.lerp_to_pose(math_utils.mul(offset2, target_to_obj), 25)
                self.change_state(SM_states.PICKING)
            else:
                # Sends the arm to an intermediary pose that will help clear the robot base more easily
                target = copy(self.default_position)
                target.p = [0.8, 0.4, 0.4]
                target.r = [-0.271, 0.650, 0.271, 0.651]
                self.lerp_to_pose(target, n_waypoints=1)
                self.target_position = self.waypoints.popleft()
                self.move_to_target()
                self.lerp_to_pose(self.default_position, n_waypoints=1)
                self.lerp_to_pose(self.default_position, n_waypoints=60)
                self.start = False
                self.total_bins += 1
                self.stack_size[self.current_stack_list[self.current_stack]] += 1
                self.change_state(SM_states.STANDBY)
            self.thresh[SM_states.DETACH] = 0

    def _detach_detached(self, *args):
        """
        Event that happens right after the arm succesfully detaches from the object it was holding.
        """
        offset = _dynamic_control.Transform()
        if self._upright:
            offset.p = (-0.28, 0.0, -0.05)
            # Quickly push the arm down 15 cm to clear the bin and let it fall on the platform
            pose = math_utils.mul(self.target_position, offset)
            self.lerp_to_pose(pose, n_waypoints=1)
            # Wait in place for a while
            self.lerp_to_pose(pose, n_waypoints=5)
            # Then move back
            offset.p = (-0.20, 0.00, 0.30)
            pose = math_utils.mul(self.target_position, offset)
            self.lerp_to_pose(pose, n_waypoints=15)
            # Reorient gripper
            pose.r = math_utils.mul(pose.r, (0, 0.7071, 0, 0.7071))
            self.lerp_to_pose(pose, n_waypoints=30)
        else:

            offset.p = (-0.10, 0.0, 0.0)
            # Move the arm up slowly 10 cm
            self.lerp_to_pose(math_utils.mul(self.target_position, offset), n_waypoints=30)
            offset.p = (-0.30, 0.0, 0.0)
            # Move the arm further up 20 extra cm, but faster
            self.lerp_to_pose(math_utils.mul(self.target_position, offset), n_waypoints=1)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()

    def _picking_goal_reached(self, *args):
        """
        Handles a state machine step when goal was reached event happens, while on picking state
        ensures the bin obstacle is suppressed for the planner, Updates the target position
        to where the bin surface is, and send the robot to move towards it. No change of state happens
        """
        if self._flipped:
            self._flipped = False
            return
        obj, distance = self.ray_cast()
        if obj is not None:
            # Set target towards surface of the bin
            tr = self.get_current_state_tr()
            offset = _dynamic_control.Transform()
            offset.p = (distance + 0.0015, 0, 0)

            target = math_utils.mul(tr, offset)
            # target.p = math_utils.mul(target.p, 0.01)
            offset.p.x = -0.05
            pre_target = math_utils.mul(target, offset)
            self.lerp_to_pose(pre_target, n_waypoints=90)
            self.lerp_to_pose(target, n_waypoints=60)
            self.lerp_to_pose(target, n_waypoints=30)
            self.target_position = self.waypoints.popleft()
            self.move_to_target()
            # Check if bin is upright or not
            if self._upright:
                # Ignore the bin holder obstacle to allow to enter the crevice to drop the bin
                self.bin_holder_object.suppress()
            # Move to attach state
            self.change_state(SM_states.ATTACH)

    def _picking_no_event(self, *args):
        """
        Handles a state machine step when no event happened, while on picking state
        ensures the bin obstacle is suppressed for the planner, Updates the target position
        to where the bin is, and send the robot to move towards it. No change of state happens
        """
        if not self._flipped:
            # set target above the current bin with offset of 25 cm
            self.set_target_to_object(0.20, 0.16, 3)
            self.thresh[SM_states.PICKING] = 0
            # start arm movement
            self.move_to_target()

    def _placing_goal_reached(self, *args):
        """
        robot reached the placing pose. If it's placing on the final destination,.identifies how farther down it needs to go,
        and places the bin either on top of another bin, or on a predefined grid pose.
        """
        if self._upright:
            # If the bin is upright, places it on the platform for picking from the bottom
            self.target_position = self.upside_goal
            self.move_to_target()
        else:
            x_off = 0.250  # Offset to clear the bin it's currently holding
            target = copy(self.target_position)
            obj, distance = self.ray_cast(x_off)
            if obj is not None:
                if "bin" in obj:  # if result is a bin, override current pose to be on top of the bin below.
                    rb = self.dc.get_rigid_body(obj)
                    tr = self.dc.get_rigid_body_pose(rb)
                    # target.p.x = tr.p.x * 0.01
                    # target.p.y = tr.p.y * 0.01
                    r = (1, 0, 0, 0)
                    rx = math_utils.get_basis_vector_x(tr.r).x
                    if rx < 0:  # rotate target by 180 degrees on z axis
                        r = (0, -1, 0, 0)
                    target.r = math_utils.mul(math_utils.mul(tr.r, r), (0, 0.7071, 0, 0.7071))
                    target.p.z -= ((distance + x_off)) - 0.15
                else:
                    target.r = [0, 0.7071, 0, 0.7071]
                    target.p.z -= ((distance + x_off)) - 0.22

            else:
                self.move_to_target()  # trigger the  goal_reached event so it tries again
                return
            target.p.z += 0.05
            self.lerp_to_pose(target, int(distance) * 2)  # moves down the pose minus 5cm at a 30 cm/s
            target.p.z -= 0.05
            self.lerp_to_pose(target, 50)  # Moves down the last 5cm at ~1cm/s
            self.lerp_to_pose(target, 75)  # Waits in place for arm to stabilize for about one second
            self.target_position = self.waypoints.popleft()
            self.move_to_target()

        self.change_state(SM_states.DETACH)

    def _all_broken_grip(self, *args):
        self._closed = False
        tr = self.get_current_state_tr()
        self.waypoints.clear()
        self.lerp_to_pose(tr, 60)
        self.lerp_to_pose(self.default_position, 90)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.current_state = SM_states.STANDBY


class BinStack(Scenario):
    """
    Defines an obstacle avoidance scenario
    Scenarios define the life cycle within kit and handle init, startup, shutdown etc.
    """

    def __init__(self, dc, mp):
        super().__init__(dc, mp)
        self._paused = True
        self._start = False
        self._reset = False
        self._time = 0
        self.pick_and_place = None
        self._pending_disable = False

        self.max_bins = 36
        self.current_bin = 0
        self.unpicked_bins = 0
        self._bins = {}
        self.add_bin_timeout = -1
        self.bin_added_timeout = 0

        self._waypoints_backup = None
        self.stopped = True
        self._pending_stop = False

    def on_startup(self):
        super().on_startup()

    def step(self, step):
        if self._timeline.is_playing():
            if self._pending_stop:
                self.stop_tasks()
                return
            # Disable requires a one simulation step after they have been moved
            # from their previous location to work.
            if self._pending_disable:
                self.disable_bins()

            # Updates current references and locations for the robot.
            self.world.update()
            self.ur10_solid.update()

            if self.bin_added_timeout > 0:
                self.bin_added_timeout -= 1
                self._add_bin_enabled = self.bin_added_timeout == 0 and self.unpicked_bins < 3

            target = self._stage.GetPrimAtPath("/environments/env/target")
            xform_attr = target.GetAttribute("xformOp:transform")
            if self._reset:
                self._paused = False
            if not self._paused:
                self._time += step
                if self.add_bin_timeout > 0:
                    self.add_bin_timeout -= 1
                    if self.add_bin_timeout == 0:
                        self.create_new_bin()
                if self._start and self.current_bin == 0:
                    self.create_new_bin()
                self.pick_and_place.step(self._time, self._start, self._reset)
                if self._reset:
                    self._paused = True
                    self._time = 0
                    p = self.default_position.p
                    r = self.default_position.r
                    set_translate(target, Gf.Vec3d(p.x, p.y, p.z))
                    set_rotate(target, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))

                else:
                    state = self.ur10_solid.end_effector.status.current_target
                    state_1 = self.pick_and_place.target_position
                    tr = state["orig"]
                    set_translate(target, Gf.Vec3d(tr[0], tr[1], tr[2]))
                    set_rotate(target, Gf.Matrix3d(Gf.Quatd(state_1.r.w, state_1.r.x, state_1.r.y, state_1.r.z)))
                self._start = False
                self._reset = False

            if self._paused:
                translate_attr = xform_attr.Get().GetRow3(3)
                rotate_x = xform_attr.Get().GetRow3(0)
                rotate_y = xform_attr.Get().GetRow3(1)
                rotate_z = xform_attr.Get().GetRow3(2)

                orig = np.array(translate_attr)
                axis_x = np.array(rotate_x)
                axis_y = np.array(rotate_y)
                axis_z = np.array(rotate_z)
                self.ur10_solid.end_effector.go_local(
                    orig=orig,
                    axis_x=axis_x,
                    axis_y=axis_y,
                    axis_z=axis_z,
                    use_default_config=True,
                    wait_for_target=False,
                    wait_time=5.0,
                )

    def create_UR10(self, background=True):
        super().create_UR10()
        if self.assets_root_path is None:
            return
        self.ur10_table_usd = (
            self.assets_root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_stacking_short_suction.usd"
        )

        # Load robot environment and set its transform
        self.env_path = "/environments/env"
        create_ur10(self._stage, self.env_path, self.ur10_table_usd, Gf.Vec3d(0, 0, 0))

        # Set robot end effector
        orig = [-0.0645, 0.7214, 0.495]
        self.default_position = _dynamic_control.Transform()
        self.default_position.p = orig
        self.default_position.r = [-0.33417784954541885, 0.33389792551856345, 0.6230546169232118, 0.6234102056738156]

        GoalPrim = self._stage.DefinePrim(self.env_path + "/target", "Xform")
        p = self.default_position.p
        r = self.default_position.r
        set_translate(GoalPrim, Gf.Vec3d(p.x, p.y, p.z))
        set_rotate(GoalPrim, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))

        a = [self.small_klt_usd for i in range(self.max_bins)]
        b = [self.env_path + "/bins/bin_{}".format(i) for i in range(self.max_bins)]
        c = [Gf.Vec3d(-50000 - 50 * i, 150, 0) for i in range(self.max_bins)]
        create_objects(self._stage, a, b, c)

        if background:
            create_background(self._stage, self.background_usd, [10.00, 2.00, -1.18180], Gf.Quatd(0.7071, 0, 0, 0.7071))

        # Setup physics simulation
        setup_physics(self._stage)

    def add_bin(self, *args):
        self.create_new_bin(args)

    def create_new_bin(self, *args):
        if self.current_bin < self.max_bins and self.unpicked_bins < 3 and self.bin_added_timeout == 0:
            i = self.current_bin
            self._dc.set_rigid_body_disable_simulation(self.bin_handles[i], False)

            tf = _dynamic_control.Transform()
            tf.p = [-random.random() * 0.15 - 0.05, 1.50, -0.15]
            z = random.random() * 0.02 - 0.01
            w = random.random() * 0.02 - 0.01
            norm = np.sqrt(z ** 2 + w ** 2)
            tf.r = [0, 0, z / norm, w / norm]
            if random.random() > 0.5:
                tf.r = math_utils.mul(tf.r, [0, 1, 0, 0])
            self._dc.set_rigid_body_pose(self.bin_handles[i], tf)
            self._dc.set_rigid_body_linear_velocity(self.bin_handles[i], [0, -0.30, 0])
            self._bin_objects[self.bin_paths[i]].unsuppress()
            self.current_bin += 1
            self.unpicked_bins += 1
            self.bin_added_timeout = 100
            self._add_bin_enabled = False

    def disable_bins(self, *args):
        for i in range(self.max_bins):
            self._dc.set_rigid_body_disable_simulation(self.bin_handles[i], True)
        self._pending_disable = False
        self.unpicked_bins = 0
        self.bin_added_timeout = 0
        self._add_bin_enabled = True

    def add_new_bin(self):
        if self.current_bin < self.max_bins:
            self.unpicked_bins -= 1
            self.add_bin_timeout = int(random.random() * 200) + 100

    def register_assets(self, *args):

        # Prim path of two blocks and their handles
        prim = self._stage.GetPrimAtPath(self.env_path)
        self.bin_paths = [self.env_path + "/bins/bin_{}".format(i) for i in range(self.max_bins)]
        self.bin_handles = [self._dc.get_rigid_body(i) for i in self.bin_paths]

        # Create world and robot object
        ur10_path = str(prim.GetPath()) + "/ur10"
        self.world = World(self._dc, self._mp)
        sgp = Surface_Gripper_Properties()
        sgp.parentPath = ur10_path + "/ee_link"
        sgp.d6JointPath = sgp.parentPath + "/d6FixedJoint"
        sgp.gripThreshold = 0.01
        sgp.forceLimit = 5.0e3
        sgp.torqueLimit = 10.0e3
        sgp.bendAngle = np.pi / 24  # 7.5 degrees
        sgp.stiffness = 1.0e3
        sgp.damping = 1.0e2
        sgp.disableGravity = True
        tr = _dynamic_control.Transform()
        tr.p.x = 0.162
        sgp.offset = tr
        sgp.retryClose = False

        self.ur10_solid = UR10(
            self._stage,
            self._stage.GetPrimAtPath(ur10_path),
            self._dc,
            self._mp,
            self.world,
            default_config,
            sgp=sgp,
            urdf="/ur10/ur10_robot_suction.urdf",
        )

        i = 0
        for p in self._stage.GetPrimAtPath(str(prim.GetPath()) + "/sortbot_housing/RMPObstacle").GetChildren():
            self.world.register_object(0, p.GetPath().pathString, "housing_" + str(i))
            self.world.make_obstacle("housing_" + str(i), 3, p.GetAttribute("xformOp:scale").Get())
            i += 1

        i = 0
        for p in self._stage.GetPrimAtPath(str(prim.GetPath()) + "/pallet_holder/RMPObstacle").GetChildren():
            self.world.register_object(0, p.GetPath().pathString, "holder_" + str(i))
            self.world.make_obstacle("holder_" + str(i), 3, p.GetAttribute("xformOp:scale").Get())
            self.world.get_object_from_name("holder_" + str(i)).suppress()
            i += 1
        self._bin_holder_obstacle = self.world.get_object_from_name("holder_0")

        self._bin_objects = {}
        for i, (bin_handle, bin_path) in enumerate(zip(self.bin_handles, self.bin_paths)):
            self.world.register_object(bin_handle, bin_path, "{}_bin".format(i))
            self.world.make_obstacle("{}_bin".format(i), 3, np.asarray(self.small_bin_scale))
            obj = self.world.get_object_from_name("{}_bin".format(i))
            self._dc.set_rigid_body_disable_simulation(bin_handle, True)
            obj.suppress()
            self._bin_objects[bin_path] = obj
            self._obstacles.append(obj)
        i = 0

        self.pick_and_place = PickAndPlaceStateMachine(
            self._stage,
            self.ur10_solid,
            self._stage.GetPrimAtPath(self.env_path + "/ur10/ee_link"),
            self._bin_objects,
            self.default_position,
            self._bin_holder_obstacle,
        )
        self.pick_and_place.add_bin = self.add_new_bin

    def perform_tasks(self, *args):
        self._start = True
        self._paused = False
        return False

    def stop_tasks(self, *args):
        if self.pick_and_place is not None:
            if self._timeline.is_playing():
                self.ur10_solid.stop()
                self._reset = True
                self.current_bin = 0
                self.add_bin_timeout = -1
                self._pending_disable = True
                for i in range(self.max_bins):
                    tf = _dynamic_control.Transform()
                    tf.p = [-50000 - 50 * i, 150, 0]
                    self._dc.set_rigid_body_pose(self.bin_handles[i], tf)
                    self._dc.set_rigid_body_linear_velocity(self.bin_handles[i], [0, 0, 0])
                    self._dc.set_rigid_body_angular_velocity(self.bin_handles[i], [0, 0, 0])
                self._pending_stop = False
            else:
                self._pending_stop = True

    def pause_tasks(self, *args):
        self._paused = not self._paused
        if self._paused:
            selection = omni.usd.get_context().get_selection()
            selection.set_selected_prim_paths(["/environments/env/target"], False)
            target = self._stage.GetPrimAtPath("/environments/env/target")
            xform_attr = target.GetAttribute("xformOp:transform")
            translate_attr = np.array(xform_attr.Get().GetRow3(3))
            if np.linalg.norm(translate_attr) < 0.01:
                p = self.default_position.p
                r = self.default_position.r
                set_translate(target, Gf.Vec3d(p.x, p.y, p.z))
                set_rotate(target, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))

        return self._paused

    def open_gripper(self):
        if self.ur10_solid.end_effector.gripper.is_closed():
            self.ur10_solid.end_effector.gripper.open()
            self.pick_and_place._closed = False
        else:
            self.ur10_solid.end_effector.gripper.close()
            self.pick_and_place._closed = True
