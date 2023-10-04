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

from .scenario import set_translate, set_rotate, create_ur10, Scenario, create_objects, setup_physics
from copy import copy

import omni.physx
from collections import deque
import asyncio


def normalize(a):
    norm = np.linalg.norm(a)
    return a / norm


class SM_events(Enum):
    START = 0
    WAYPOINT_REACHED = 1
    GOAL_REACHED = 2
    ATTACHED = 3
    DETACHED = 4
    TIMEOUT = 5
    STOP = 6

    NONE = 7  # no event ocurred, just clocks


class SM_states(Enum):
    STANDBY = 0  # Default state, does nothing unless enters with event START

    PICKING = 1
    ATTACH = 2
    HOLDING = 3


statedic = {0: "orig", 1: "axis_x", 2: "axis_y", 3: "axis_z"}


class PickAndPlaceStateMachine(object):
    """
    Self-contained state machine class for Robot Behavior. Each machine state may react to different events,
    and the handlers are defined as in-class functions
    """

    def __init__(self, stage, robot, ee_prim, target_body, default_position):
        self.robot = robot
        self.dc = robot.dc
        self.end_effector = ee_prim
        self.end_effector_handle = None
        self._stage = stage
        self.current = target_body
        self.retry = True

        self.start_time = 0.0
        self.start = False
        self._time = 0.0
        self.default_timeout = 0.5
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

        self.is_closed = False
        self.pick_count = 0
        # Define the state machine handling functions
        self.sm = {}
        # Make empty state machine for all events and states
        for s in SM_states:
            self.sm[s] = {}
            for e in SM_events:
                self.sm[s][e] = self._empty
                self.thresh[s] = 0

        # Fill in the functions to handle each event for each status
        self.sm[SM_states.STANDBY][SM_events.START] = self._standby_start
        self.sm[SM_states.STANDBY][SM_events.GOAL_REACHED] = self._standby_goal_reached
        self.thresh[SM_states.STANDBY] = 3

        self.sm[SM_states.PICKING][SM_events.GOAL_REACHED] = self._picking_goal_reached
        self.sm[SM_states.PICKING][SM_events.NONE] = self._picking_no_event
        self.thresh[SM_states.PICKING] = 1

        self.sm[SM_states.ATTACH][SM_events.GOAL_REACHED] = self._attach_goal_reached
        self.sm[SM_states.ATTACH][SM_events.ATTACHED] = self._attach_attached

        self.sm[SM_states.HOLDING][SM_events.GOAL_REACHED] = self._holding_goal_reached
        self.thresh[SM_states.HOLDING] = 3
        for s in SM_states:
            self.sm[s][SM_events.DETACHED] = self._all_detached

        self.current_state = SM_states.STANDBY
        self.previous_state = -1
        self._physx_query_interface = omni.physx.get_physx_scene_query_interface()

    # Auxiliary functions

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

    def ray_cast(self, x_offset=0.15, y_offset=0.03, z_offset=0.0):
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

    def lerp_to_pose(self, pose, n_waypoints=1):
        """
        adds spherical linear interpolated waypoints from last pose in the waypoint list to the provided pose
        if the waypoit list is empty, use current pose
        """
        if len(self.waypoints) == 0:
            start = self.get_current_state_tr()
            # start.p = math_utils.mul(start.p, 0)
        else:
            start = self.waypoints[-1]

        if n_waypoints > 1:
            for i in range(n_waypoints):
                self.waypoints.append(math_utils.slerp(start, pose, (i + 1.0) / n_waypoints))
        else:
            self.waypoints.append(pose)

    def move_to_zero(self):
        self._is_moving = False
        self.robot.end_effector.go_local(
            orig=[], axis_x=[], axis_y=[], axis_z=[], use_default_config=True, wait_for_target=False, wait_time=5.0
        )

    def move_to_target(self):
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
        offset.p.x = -offset_up

        offset.p.z = 0.03
        offset.r = (0, 0, 0, 1)
        body_handle = self.dc.get_rigid_body(self.current)
        obj_pose = self.dc.get_rigid_body_pose(body_handle)
        offset_1 = _dynamic_control.Transform()
        # tr = self.get_current_state_tr()
        if math_utils.get_basis_vector_z(obj_pose.r).z > 0:
            self._upright = True
            offset_1.r = (0, -1, 0, 0)
            offset.p.z = -0.03
        else:  # If bin is upside down, pick by bottom
            offset_1.r = (1, 0, 0, 0)
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

        if self.is_closed and not self.robot.end_effector.gripper.is_closed():
            self._detached = True
            self.is_closed = False

        # Process events
        if reset:
            self.current_state = SM_states.STANDBY
            self.robot.end_effector.gripper.open()
            self.start = False
            self._upright = False
            self.waypoints.clear()
            self.target_position = self.default_position
            self.move_to_target()
        elif self._detached:
            self._detached = False
            self.sm[self.current_state][SM_events.DETACHED]()
        elif self.goalReached():
            if len(self.waypoints) == 0:
                self.sm[self.current_state][SM_events.GOAL_REACHED]()
            else:
                self.target_position = self.waypoints.popleft()
                self.move_to_target()
                self.start_time = self._time
        elif self.current_state == SM_states.STANDBY and self.start:
            self.sm[self.current_state][SM_events.START]()
        elif self._attached:
            self._attached = False
            self.sm[self.current_state][SM_events.ATTACHED]()
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
        # Tell motion planner controller to ignore current object as an obstacle
        self.pick_count = 0
        self.lerp_to_pose(self.default_position, 1)
        self.lerp_to_pose(self.default_position, 90)
        # set target above the current bin with offset of 20 cm
        self.set_target_to_object(0.25, 0.25, 6, clear_waypoints=False)
        # start arm movement
        self.move_to_target()
        # Move to next state
        self.change_state(SM_states.PICKING)

    def _standby_goal_reached(self, *args):
        """
        Finished processing a bin, moves up the stack position for next bin placement
        """
        self.move_to_zero()
        self.start = True

    def _attach_goal_reached(self, *args):
        """
        Handles a state machine step when the target goal is reached, and the machine is on attach state
        """
        self.robot.end_effector.gripper.close()
        if self.robot.end_effector.gripper.is_closed():
            self.lerp_to_pose(self.target_position, 20)
            self._attached = True
            self.is_closed = True
            self.retry = True
        elif self.retry:
            self.retry = False
            self.lerp_to_pose(self.target_position, 20)
            self.move_to_target()
        else:
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
        self.waypoints.clear()
        target_position = _dynamic_control.Transform()
        target_position.p = [0.0, 0.81, 0.58]
        target_position.r = [0, -1, 0, 0]
        # print(target_position.r)
        self.lerp_to_pose(target_position, 360)
        self.target_position = self.waypoints.popleft()
        self.move_to_target()
        self.change_state(SM_states.HOLDING)

    def _picking_goal_reached(self, *args):
        """
        Handles a state machine step when goal was reached event happens, while on picking state
        ensures the bin obstacle is suppressed for the planner, Updates the target position
        to where the bin surface is, and send the robot to move towards it. No change of state happens
        """
        obj, distance = self.ray_cast()
        if obj is not None:
            # Set target towards surface of the bin
            tr = self.get_current_state_tr()
            offset = _dynamic_control.Transform()
            offset.p = (distance + 0.15, 0, 0)

            target = math_utils.mul(tr, offset)
            # target.p = math_utils.mul(target.p, 0.01)
            offset.p.x = -0.05

            pre_target = math_utils.mul(target, offset)
            self.lerp_to_pose(pre_target, n_waypoints=40)
            self.lerp_to_pose(target, n_waypoints=30)
            self.lerp_to_pose(target, n_waypoints=30)
            self.target_position = self.waypoints.popleft()
            self.move_to_target()
            self.retry = True
            self.change_state(SM_states.ATTACH)

    def _picking_no_event(self, *args):
        """
        Handles a state machine step when no event happened, while on picking state
        ensures the bin obstacle is suppressed for the planner, Updates the target position
        to where the bin is, and send the robot to move towards it. No change of state happens
        """
        self.set_target_to_object(0.25, 0.25, 1, True)
        self.move_to_target()

    def _holding_goal_reached(self, *args):

        if self.add_bin is not None:
            self.add_bin()
        self.lerp_to_pose(self.target_position, 20)
        self.move_to_target()

    def _all_detached(self, *args):
        self.current_state = SM_states.STANDBY
        self.start = False
        self._upright = False
        self.waypoints.clear()
        self.lerp_to_pose(self.target_position, 60)
        self.lerp_to_pose(self.default_position, 10)
        self.lerp_to_pose(self.default_position, 60)
        self.move_to_target()


class FillBin(Scenario):
    """Defines an obstacle avoidance scenario

    Scenarios define the life cycle within kit and handle init, startup, shutdown etc.
    """

    def __init__(self, dc, mp):
        super().__init__(dc, mp)
        self._paused = True
        self._start = False
        self._reset = False
        self._time = 0
        self._start_time = 0
        self.current_state = SM_states.STANDBY
        self.timeout_max = 8.0
        self.pick_and_place = None
        self._pending_disable = False
        self._pending_stop = False

        self.max_bins = 36

        self.current_obj = 0
        self.max_objs = 100
        self.num_objs = 3

        self._bins = {}

        self.add_objects_timeout = -1
        self.ur10_solid = None

    def __del__(self):
        if self.ur10_solid:
            self.ur10_solid.end_effector.gripper = None
        super().__del__()

    def on_startup(self):
        super().on_startup()

    def step(self, step):
        if self._timeline.is_playing():
            if self._pending_stop:
                self.stop_tasks()
                return
            # Updates current references and locations for the robot.
            self.world.update()
            self.ur10_solid.update()

            target = self._stage.GetPrimAtPath("/environments/env/target")
            xform_attr = target.GetAttribute("xformOp:transform")
            if self._reset:
                self._paused = False
            if not self._paused:
                self._time += 1.0 / 60.0
                # if self.pick_and_place.is_closed and not self.ur10_solid.end_effector.gripper.is_closed():
                #     self.pick_and_place.step(self._time, False, False)
                #     self.stop_tasks(reset=False)
                #     return

                self.pick_and_place.step(self._time, self._start, self._reset)
                if self._reset:
                    self._paused = (self._time - self._start_time) < self.timeout_max
                    self._time = 0
                    self._start_time = 0
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
                if (
                    self.pick_and_place.current_state == self.current_state
                    and self.current_state in [SM_states.PICKING, SM_states.ATTACH]
                    and (self._time - self._start_time) > self.timeout_max
                ):
                    self.stop_tasks()
                elif self.pick_and_place.current_state != self.current_state:
                    self._start_time = self._time
                    # print(self._time)
                    self.current_state = self.pick_and_place.current_state
                if self.add_objects_timeout > 0:
                    self.add_objects_timeout -= 1
                    if self.add_objects_timeout == 0:
                        self.create_new_objects()

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

    def create_UR10(self, *args):
        super().create_UR10()
        if self.assets_root_path is None:
            return
        self.ur10_table_usd = self.assets_root_path + "/Isaac/Samples/Leonardo/Stage/ur10_bin_filling.usd"

        self.objects = [
            self.assets_root_path + "/Isaac/Props/Flip_Stack/large_corner_bracket_physics.usd",
            self.assets_root_path + "/Isaac/Props/Flip_Stack/screw_95_physics.usd",
            self.assets_root_path + "/Isaac/Props/Flip_Stack/screw_99_physics.usd",
            self.assets_root_path + "/Isaac/Props/Flip_Stack/small_corner_bracket_physics.usd",
            self.assets_root_path + "/Isaac/Props/Flip_Stack/t_connector_physics.usd",
        ]

        # Load robot environment and set its transform
        self.env_path = "/environments/env"
        create_ur10(self._stage, self.env_path, self.ur10_table_usd, Gf.Vec3d(0, 0, 0))

        # Set robot end effector Target
        orig = [0, 0.75, 0.42]
        self.default_position = _dynamic_control.Transform()
        self.default_position.p = orig
        self.default_position.r = [-0.33417784954541885, 0.33389792551856345, 0.6230546169232118, 0.6234102056738156]

        GoalPrim = self._stage.DefinePrim(self.env_path + "/target", "Xform")
        p = self.default_position.p
        r = self.default_position.r
        set_translate(GoalPrim, Gf.Vec3d(p.x, p.y, p.z))
        set_rotate(GoalPrim, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))

        num_objs = self.max_objs
        a = [self.objects[random.randint(0, len(self.objects) - 1)] for i in range(num_objs)]
        b = [self.env_path + "/objects/object_{}".format(self.current_obj + i) for i in range(num_objs)]
        c = [Gf.Vec3d(-50000, 0, -50000 + 5 * i) for i in range(num_objs)]
        create_objects(self._stage, a, b, c)
        self.current_obj = 0

        # Setup physics simulation
        setup_physics(self._stage)

    def add_bin(self, *args):
        self.create_new_objects(args)

    def create_new_objects(self, *args):
        for i in range(self.current_obj, min(self.current_obj + self.num_objs, self.max_objs)):
            self._dc.set_rigid_body_disable_simulation(self.objects_handles[i], False)
            tf = _dynamic_control.Transform()
            tf.p = [
                random.randint(-15, -5) * 0.01,
                random.randint(-3, 3) * 0.01 + 0.81,
                1.10 + 0.05 * (i - self.current_obj),
            ]
            tf.r = normalize(
                [random.random() * 2 - 1, random.random() * 2 - 1, random.random() * 2 - 1, random.random() * 2 - 1]
            )
            self._dc.set_rigid_body_pose(self.objects_handles[i], tf)
            self._dc.set_rigid_body_linear_velocity(self.objects_handles[i], [0, 0.0, 0])
            self._dc.set_rigid_body_angular_velocity(
                self.objects_handles[i], [random.random(), random.random(), random.random()]
            )
        self.current_obj = (self.current_obj + self.num_objs) % self.max_objs

    def disable_bins(self, *args):
        for i in range(self.max_objs):
            self._dc.set_rigid_body_disable_simulation(self.objects_handles[i], True)
        self._pending_disable = False

    def add_new_objects(self):
        self.add_objects_timeout = 10

    def register_assets(self, *args):

        # Prim path of two blocks and their handles
        prim = self._stage.GetPrimAtPath(self.env_path)
        self.bin_path = self.env_path + "/bin"
        self.bin_handle = self._dc.get_rigid_body(self.bin_path)

        # Create world and robot object
        ur10_path = str(prim.GetPath()) + "/ur10"
        self.world = World(self._dc, self._mp)
        sgp = Surface_Gripper_Properties()
        sgp.parentPath = ur10_path + "/ee_link"
        sgp.d6JointPath = sgp.parentPath + "/d6FixedJoint"
        sgp.gripThreshold = 0.01
        sgp.forceLimit = 1.0e2
        sgp.torqueLimit = 1.0e2
        sgp.bendAngle = np.pi / 24  # 7.5 degrees
        sgp.stiffness = 1.0e2
        sgp.damping = 1.0e4
        sgp.disableGravity = True
        tr = _dynamic_control.Transform()
        tr.p.x = 0.162
        sgp.offset = tr

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

        # self.world.register_object(0, self.env_path + "/gridroom_curved/staticPlaneActor", "ground")
        # self.world.make_obstacle("ground", 3, (10, 10, 0.1))

        body_count = self._dc.get_articulation_body_count(self.ur10_solid.ar)
        for bodyIdx in range(body_count):
            body = self._dc.get_articulation_body(self.ur10_solid.ar, bodyIdx)
            self._dc.set_rigid_body_disable_gravity(body, True)

        self.pick_and_place = PickAndPlaceStateMachine(
            self._stage,
            self.ur10_solid,
            self._stage.GetPrimAtPath(self.env_path + "/ur10/ee_link"),
            self.bin_path,
            self.default_position,
        )
        self.pick_and_place.add_bin = self.add_new_objects

        self.objects_handles = []
        for prim in self._stage.GetPrimAtPath(self.env_path + "/objects").GetChildren():
            for o in [o for o in prim.GetChildren() if "Looks" not in o.GetPath().pathString]:
                obj = self._dc.get_rigid_body(o.GetPath().pathString)
                self.objects_handles.append(obj)
                self._dc.set_rigid_body_disable_simulation(obj, True)

    def perform_tasks(self, *args):
        self._start = True
        self._paused = False
        return False

    def stop_tasks(self, reset=True):
        asyncio.ensure_future(self.stop_tasks_async(reset))

    async def stop_tasks_async(self, reset=True):
        if self.pick_and_place is not None:
            if self._timeline.is_playing():
                self.add_objects_timeout = 0
                self.ur10_solid.stop()
                self._reset = reset
                self._pending_disable = True
                if self._timeline.is_playing():
                    self.ur10_solid.end_effector.gripper.open()
                for i in range(self.max_objs):
                    tf = self._dc.get_rigid_body_pose(self.objects_handles[i])
                    tf.p = [-500000 - 50 * i, 150, 0]
                    tf.r = [0, 0, 0, 1]
                    self._dc.set_rigid_body_pose(self.objects_handles[i], tf)

                await omni.kit.app.get_app().next_update_async()
                for i in range(self.max_objs):
                    self._dc.set_rigid_body_disable_simulation(self.objects_handles[i], True)
                    # self._dc.set_rigid_body_linear_velocity(self.objects_handles[i], [0, 0, 0])
                    # self._dc.set_rigid_body_angular_velocity(self.objects_handles[i], [0, 0, 0])
                bin = self._dc.get_rigid_body(self.bin_path)
                tf = self._dc.get_rigid_body_pose(bin)
                tf.p = [0, 0.81, -0.430]
                tf.r = [0, 0, 0, 1]
                self._dc.set_rigid_body_pose(bin, tf)
                # self._dc.set_rigid_body_linear_velocity(bin, [0, 0, 0])
                # self._dc.set_rigid_body_angular_velocity(bin, [0, 0, 0])
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
            if np.linalg.norm(translate_attr) < 0.0001:
                p = self.default_position.p
                r = self.default_position.r
                set_translate(target, Gf.Vec3d(p.x, p.y, p.z))
                set_rotate(target, Gf.Matrix3d(Gf.Quatd(r.w, r.x, r.y, r.z)))
        return self._paused

    def open_gripper(self):
        if self.ur10_solid.end_effector.gripper.is_closed():
            self.ur10_solid.end_effector.gripper.open()
        else:
            self.ur10_solid.end_effector.gripper.close()
