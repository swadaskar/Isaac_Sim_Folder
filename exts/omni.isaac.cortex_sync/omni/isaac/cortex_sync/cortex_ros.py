# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

""" Cortex sync tools using ROS as the communication backend.

For a simple example usage, see

    standalone_examples/api/omni.isaac.cortex/example_cortex_sync_main.py

The main differences between this standalone python app and a standard Cortex standalone python app
are the creation of these objects:

    cortex_sim = CortexSimRobotRos(sim_robot)
    cortex_sim_objects_ros = CortexSimObjectsRos(sim_objects)
    cortex_control = CortexControlRos(robot)
    cortex_objects_ros = CortexObjectsRos(cortex_objects, auto_sync_objects=args.auto_sync_objects)

These objects register PhysX callbacks and handle all communication processing in the background. 

The breakdown is:
1. We collect up the sim objects into a sim_objects dict and pass it into a CortexSimRobotRos object
   which handles publishing ground truth poses from those objects.
2. We wrap belief objects as CortexObject(obj) and add it to a cortex_objects dict. That dict of
   relevant objects is passed into a CortexObjectsRos object which handles receiving those poses and
   making them available through the CortexObject objects as the measured poses.
3. We create a CortexSimRobotRos object to handle mimicking the ROS interface from the physical
   robot. It publishes joint state information for the robot.
4. We make a CortexControlRos object which handles synchronizing the belief robot with the sim (or
   physical) robot as well as publishing joint-level commands to external controllers.
"""

import json
import numpy as np
import time
from typing import Any, Dict, Optional, Sequence, Tuple, Union

import rospy

from cortex_control.msg import JointPosVelAccCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String, Bool
import tf2_ros

from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationSubset
from omni.isaac.core.prims import XFormPrim
import omni.isaac.cortex.math_util as math_util
from omni.isaac.cortex.commander import Commander
from omni.isaac.cortex.cortex_object import CortexObject, CortexMeasuredPose
from omni.isaac.cortex.motion_commander import MotionCommander
from omni.isaac.cortex.robot import CortexRobot, CortexGripper, DirectSubsetCommander
import omni.isaac.cortex_sync.ros_tf_util as ros_tf_util
from omni.isaac.cortex_sync.synchronized_time import SynchronizedTime


class PosVel:
    """ Convenient paring of a position and velocity. Provides a string conversion method which
    gives it semantics of configuration q: <pos>\n qd: <vel>.

    The class can be used to either store the position and velocity values of the single joint
    (float) or the position and velocity vectors or multiple joints (np.ndarray).

    Args:
        pos: The position vector.
        vel: The velocity vector of the same dimensionality.
    """

    def __init__(self, pos: Union[float, np.ndarray], vel: Union[float, np.ndarray]):
        self.pos = pos
        self.vel = vel

    def __str__(self) -> str:
        """ Format the information as 
        
            q: <vec>
            qd: <vec>

        Returns: The formatted string.
        """
        return "\nq: %s\nqd: %s" % (str(self.pos), str(self.vel))


class StampedValue:
    """ Pairs a time stamp with a corresponding value of any type.

    Args:
        stamp: The timestamp
        value: The object being timestamped
    """

    def __init__(self, stamp: rospy.Time, value: Any):
        self.stamp = stamp
        self.value = value


class PackedAndPrunedJointMsgs:
    """ Helper class for dealing with joint states messages.
    
    Joint state messages for a given articulation may arrive in parts. These parts need to be
    combined into a single view, and at times a given set of joints may go stale and need to be
    pruned off. 
    """

    def __init__(self):
        self._stamped_name_value_pairs = {}

    def add_stamped_value(self, stamp: rospy.Time, name: str, value: Any) -> None:
        """ Add a stamped value with the given key name.

        Args:
            stamp: The time stamp to use.
            name: The key used for referencing the (stamp, value) pair.
            value: The value being added.
        """
        self._stamped_name_value_pairs[name] = StampedValue(stamp, value)

    def add_stamped_joint_values(self, stamp: rospy.Time, name: str, pos: float, vel: float) -> None:
        """ Add a stamped joint position and velocity.

        Adds the stamped (position, velocity) pair as a PosVel object.

        Args:
            stamp: The stamp to use.
            name: The name of the joint.
            pos: The position value.
            vel: The velocity value.
        """
        self.add_stamped_value(stamp, name, PosVel(pos, vel))

    def prune_by_stamp(self, prune_thresh_stamp: rospy.Time) -> None:
        """ Prune any stored joint (position, velocity) pair that's older than the provided time
        threshold.

        Args:
            prune_thresh_stamp: The time threshold. Stored joint states older than this will be
                pruned.
        """
        names_to_prune = []
        for name, stamped_value in self._stamped_name_value_pairs.items():
            if stamped_value.stamp < prune_thresh_stamp:
                names_to_prune.append(name)

        for name in names_to_prune:
            del self._stamped_name_value_pairs[name]

    def get_joint_states(self, joint_names: Sequence[str]) -> Union[PosVel, None]:
        """ Returns the vector of position and velocity joint values for the named joints.

        If any of the named joints is not available, returns None. Otherwise, returns a PosVel
        object containing the joint values for the named joints.

        Args:
            joint_names: The names of the joints information is wanted for.

        Returns: A PosVel object containing position and velocity vectors for the named joints. Or
            None if any of the named joints aren't available.
        """
        try:
            pos = [self._stamped_name_value_pairs[name].value.pos for name in joint_names]
            vel = [self._stamped_name_value_pairs[name].value.vel for name in joint_names]
        except KeyError as e:
            # If one of the keys isn't yet available, return None
            # print("Joint state not available:",  e.args[0])
            return None
        return PosVel(pos, vel)


class GripperCommandSerializer:
    """ A tool for converting gripper commands to JSON messages with the same information.

    Args:
        gripper_command: The gripper command which will be serialized.
    """

    def __init__(self, gripper_command: CortexGripper.Command):
        self.gripper_command = gripper_command

    def is_different(self, other: "GripperCommandSerializer") -> bool:
        """ Report whether this gripper command is different from the one represented in other.

        Args:
            other: The other gripper command to compare to.

        Returns: True if they're the same, and False otherwise.
        """
        if self.command != other.command:
            return True

        if self.command == "move_to" and (
            self.gripper_command.width != other.gripper_command.width
            or self.gripper_command.speed != other.gripper_command.speed
        ):
            return True

        return False

    def to_msg_dict(self) -> dict:
        """ Convert this gripper command to a dict format.
        """
        msg = {}
        msg["width"] = self.gripper_command.width
        msg["speed"] = self.gripper_command.speed
        msg["force"] = self.gripper_command.force
        return msg

    def to_msg(self) -> Union[String, None]:
        """ Convert this command to a message encoding a serialized JSON string.

        The JSON string contains the information in the dict format returned by to_msg_dict().

        Returns: The String ROS message containing the serialized JSON message. If there is a JSON
            parse error, returns None.
        """
        try:
            msg_dict = self.to_msg_dict()
            s = json.dumps(msg_dict)
            msg = String(s)
        except ValueError as ve:
            print("Json parse error:\n", msg_dict)
            print("ERROR:", ve)
            return None
        return msg


def cortex_init_ros_node(node_name: Optional[str] = "cortex") -> None:
    """ A helper method to call rospy.init_node(...) with common defaults that operate nicely with
    Isaac Sim standalone Python apps.

    Args:
        node_name: The name to give the ROS node on initialization.
    """
    rospy.init_node(node_name, log_level=rospy.ERROR, anonymous=False, disable_signals=True)


def make_motion_command_ros_pub(topic: str) -> rospy.Publisher:
    """ Constructs a publisher for publishing JointPosVelAccCommand messages on the provided topic.

    Args:
        topic: The topic to publish on.

    Returns: The constructed publisher.
    """
    return rospy.Publisher(topic, JointPosVelAccCommand, queue_size=10)


def pack_motion_command_ros_msg(
    motion_commander: MotionCommander, msg_id: int, stamp: rospy.Time, period: rospy.Duration
) -> JointPosVelAccCommand:
    """ Pack the latest action from the articulation subset of the provided motion commander into a
    joint position, velocity, acceleration command message. 

    Args:
        motion_commander: The motion commander with the relevant latest action.
        msg_id: An ID for this latest message.
        stamp: The time stamp to add to this message's header.
        period: The period information indicating the time duration between joint command messages.

    Returns: The JointPosVelAccCommand message packed with the provided information.
    """
    action = motion_commander.latest_action
    joint_names = motion_commander.articulation_subset.joint_names

    msg = JointPosVelAccCommand()
    msg.period = period

    msg.id = msg_id
    msg.header = Header()
    msg.header.seq = msg_id
    msg.header.stamp = stamp

    msg.q = action.joint_positions
    msg.qd = action.joint_velocities
    msg.qdd = []  # Note, change to np.zeros(len(qd)) if using older builds of cortex_control_franka
    msg.names = joint_names
    msg.t = stamp

    return msg


def make_gripper_command_ros_pub(topic: str) -> rospy.Publisher:
    """ Constructs a publisher for publishing gripper commands as serialized JSON string messages
    on the provided topic.

    Args:
        topic: The topic to publish on.

    Returns: The constructed publisher.
    """
    return rospy.Publisher(topic, String, queue_size=10)


def make_ros_pub(commander: Commander) -> rospy.Publisher:
    """ Creates a ROS publisher for publishing commands from the provided commander.

    Depending on the commander type, this method will choose what type of message to publish and
    what topic to publish it on.

    Args:
        commander: The commander to create the publisher for.

    Returns: The publisher for that commander.
    
    Raises: RuntimeError if the commander type is unrecognized.
    """
    if isinstance(commander, MotionCommander):
        return make_motion_command_ros_pub("/cortex/arm/command")
    elif isinstance(commander, CortexGripper):
        return make_gripper_command_ros_pub("/cortex/gripper/command")
    else:
        raise RuntimeError("Could not pack ros message for commander: {}".format(commander))


def pack_ros_msg(
    commander: Commander, msg_id: int, stamp: rospy.Time, period: rospy.Duration
) -> Union[JointPosVelAccCommand, String]:
    """ Pack information about the latest command in the commander into a ROS message, along with
    any required meta data.

    Not all meta information will necessarily be used for every commander.

    Args:
        commander: The commander with the latest command information.
        msg_id: The ID of this message. The ID increments with every cycle.
        stamp: A time stamp for this message.
        period: The time duration between messages (cycle period).

    Returns: The packed ROS message.

    Raises: RuntimeError if the commander type is unrecognized.
    """
    if isinstance(commander, MotionCommander):
        return pack_motion_command_ros_msg(commander, msg_id, stamp, period)
    elif isinstance(commander, CortexGripper):
        if commander.latest_command is None:
            return None
        return GripperCommandSerializer(commander.latest_command).to_msg()
    else:
        raise RuntimeError("Could not pack ros message for commander: {}".format(commander))


class CortexControlRos:
    """ A tool to handle publishing commands from all a given (belief) robot's commanders and
    synchronizing the robot with either a real-world physical robot or a simulated version of that.
    We consider both of these cases as the "outside world", external to Cortex, either simulated or
    physical.

    Listens to joint states messages and arm commander suppression messages on topics:
        Joint state messages [JointState]: /robot/joint_state
        Suppression messages [Bool]: /cortex/arm/command/suppress

    Each commander in the robot has an associated command publisher which this object publishes
    commands on. For instance, the Franka robot has a MotionCommander and a FrankaGripper. Its
    messages are defined by make_ros_pub() above:
        Arm command messages [JointPosVelAccCommand]: /cortex/arm/command
        Gripper command messages [String]: /cortex/gripper/command

    Args:
        robot: The Cortex robot this object synchronizes with the outside world.
    """

    def __init__(self, robot: CortexRobot):
        self.robot = robot
        self._verbose = False

        self._num_cycles = 0

        # We'll add this as a physics callback because physics callbacks are called after the physics step.
        world = World.instance()  # Get the singleton.
        world.add_physics_callback("cortex_control_cb", self._on_simulation_step)

        self._physics_call_count = 0
        self._start_time = None
        self._synced_time = SynchronizedTime()
        self._states_from_suppress = None

        # Some fixed properties. TODO: These should be configurable.
        self._js_msg_stale_thresh = rospy.Duration(0.5)
        self._suppress_msg_timeout = 0.2

        # Members filled in by ROS subscriber callbacks.
        self._is_suppressed = False
        self._packed_joint_msg_values = PackedAndPrunedJointMsgs()
        self._joint_states = None
        self._suppress_msg_stamp = None

        # Members filled in each physics step from _on_simulation_step
        self._next_msg_id = 0
        self._prev_gripper_command = None

        self._suppression_sub = rospy.Subscriber("/cortex/arm/command/suppress", Bool, self._suppression_callback)
        self._joint_state_sub = rospy.Subscriber("/robot/joint_state", JointState, self._joint_state_callback)

        # Setup publishers
        self._joint_command_pubs = {}
        for _, commander in self.robot.commanders.items():
            self._joint_command_pubs[commander] = make_ros_pub(commander)

    def _joint_state_callback(self, msg: JointState) -> None:
        """ Process an incoming JointState message. Packs the data into a PackedAndPrunedJointMsgs
        object for easy merging.

        Args:
            msg: The incoming JointState message.
        """
        try:
            stamp = rospy.Time.now()
            for name, (pos, vel) in zip(msg.name, zip(msg.position, msg.velocity)):
                self._packed_joint_msg_values.add_stamped_joint_values(stamp, name, pos, vel)
            self._packed_joint_msg_values.prune_by_stamp(stamp - self._js_msg_stale_thresh)
            if self.robot.handles_initialized:
                self._joint_states = self._packed_joint_msg_values.get_joint_states(self.robot.dof_names)
        except Exception as e:
            print("\nProblem processing joint state message.")
            import traceback

            traceback.print_exc()

    def _suppression_callback(self, msg: Bool) -> None:
        """ Suppression messages are simply stored in an self._is_suppressed member along with their
        incoming time stamp.
        """
        self._is_suppressed = msg.data
        self._suppress_msg_stamp = rospy.Time.now()

    def _step_msg_meta_data(self) -> Tuple[int, rospy.Time, rospy.Duration]:
        """ Step the message meta data: message ID, current time, adaptive period duration

        This method should be called once per cycle.

        Returns: latest (msg ID, time, period)
        """
        try:
            adaptive_cycle = self._synced_time.next_adaptive_cycle_time()
            cmd_time = adaptive_cycle.time
            if adaptive_cycle.is_period_available:
                adaptive_cycle_dt = adaptive_cycle.period.to_sec()
                self.robot.set_commanders_step_dt(adaptive_cycle_dt)
                period = cmd_time - self._prev_cmd_time
            else:
                period = rospy.Duration(0)

            return self._next_msg_id, cmd_time, period

        except Exception as e:
            print("\nProblem packing command message.")
            import traceback

            traceback.print_exc()
        finally:
            # Always increment and safe off time at the end.
            self._next_msg_id += 1
            self._prev_cmd_time = cmd_time
            self._num_cycles += 1

    def _on_simulation_step(self, step: float) -> None:
        """ On each simulation step, we process incoming and outgoing messages.

        If it's suppresed, then we don't process anything. Note that the communication protocol with
        the controller requires the controller to explicitly unsuppress Cortex when necessary. We
        also handle the case where the controller dies while suppressing Cortex and unsuppress
        Cortex automatically when the time delta since the last suppression message is exceeds a
        threshold.

        During suppression, the latest joint states are stored off so that on the first step once
        Cortex is unsuppressed, we can synchronize the robot with the outside robot by setting its
        joint states to the measured values. This only happens once on the first cycle after being
        suppressed.

        Every cycle while not suppressed, all commanders are processed. Their latest commands are
        packed into ROS messages and published.

        Args:
            step: The dt between steps. Required API for a physics callback.
        """
        self._physics_call_count += 1
        if self._verbose:
            print("cortex_ros:", self._physics_call_count, "t:", time.time())

        if self._is_suppressed:
            print("<cortex suppressed by for synchronization>")
            self._synced_time.reset()
            self._states_from_suppress = self._joint_states

            now = rospy.Time.now()
            delta_secs = (now - self._suppress_msg_stamp).to_sec()
            if delta_secs > self._suppress_msg_timeout:
                self._is_suppressed = False
            return
        else:
            if self._states_from_suppress is not None:
                print("Setting robot to measured joint states:", self._states_from_suppress)
                self.robot.set_joint_positions(self._states_from_suppress.pos)
                self.robot.set_joint_velocities(self._states_from_suppress.vel)
                self._states_from_suppress = None

                print("Resetting cortex pipeline")
                self.robot.arm.soft_reset()

            msg_id, stamp, period = self._step_msg_meta_data()
            for _, commander in self.robot.commanders.items():
                pub = self._joint_command_pubs[commander]
                msg = pack_ros_msg(commander, msg_id, stamp, period)
                if msg is not None:
                    pub.publish(msg)
            return


class StampedMsg:
    """ Pairs a message with a time stamp and an expiration duration.
    
    Args:
        stamp: The time stamp.
        msg: The message object.
        expiration_duration: An optional expiration duration. Defaults to .25 seconds.
    """

    def __init__(
        self, stamp: rospy.Time, msg: Any, expiration_duration: Optional[rospy.Duration] = rospy.Duration(0.25)
    ):
        self.stamp = stamp
        self.msg = msg
        self.expiration_duration = expiration_duration

    def has_expired(self, now: rospy.Time) -> bool:
        """ Reports whether this stamped message has expired.

        Args:
            now: The current time stamp.

        Returns: True if it's been greater than the expiration duration since the stamp, and False
            otherwise.
        """
        return (now - self.stamp) > self.expiration_duration


class CortexSimRobotRos:
    """ A tool to create a ROS interface to a simulated CortexRobot that mimics the required
    interface to a physical robot.

    The robot should contain commanders abstracting the command interface of a physical robot. For
    instance, if the robot has an arm commandable by joint position / velocity commands, it should
    contain a DirectSubsetCommander (see robot.py) govering those joints. Often that commander will
    be commanded by commands coming from a MotionCommander on the Cortex belief robot.
    
    Likewise, if the robot has a standard parallel gripper, the robot should have a gripper
    commander providing the standard interface. In this case, both the belief and sim robots will
    process the higher level gripper commands in the same way (rather than the sim robot processing
    joint level gripper commands).

    Note: Currently this class is hard-coded to expect incoming arm and gripper commands of the type
    listed above.

    The incoming commands are expected on the following topics:

        Arm joint commands [JointPosVelAccCommand]: /cortex/arm/command/interpolated
        Gripper commands [String]: /cortex/gripper/command

    Note that often the gripper commands will come directly from the CortexControlRos object while
    the arm's joint commands will come from an separate controller node emulating the real time
    controller for controlling the physical robot. A simulated version of that controller can be run
    using the command:

        rosrun cortex_control sim_controller

    This class also handles publishing joint state objects for each commander's articulation subset
    on the topic:

        Joint state messages [JointState] /robot/joint_state

    Each commander's subset is published as a separate message (which is common for physical robots).

    Args:
        robot: The robot object with commanders mimicking the physical robot's command interface.
    """

    def __init__(self, robot: CortexRobot):
        self.robot = robot
        self._verbose = False

        world = World.instance()  # Get the singleton.
        world.add_physics_callback("cortex_sim_cb", self._on_simulation_step)
        self._physics_call_count = 0

        self._latest_stamped_command_msg = None
        self._latest_stamped_gripper_command_msg = None

        self._joint_state_pub = rospy.Publisher("/robot/joint_state", JointState, queue_size=10)

        self._interpolated_joint_command_sub = rospy.Subscriber(
            "/cortex/arm/command/interpolated", JointPosVelAccCommand, self._interpolated_joint_command_callback
        )
        self._gripper_command_sub = rospy.Subscriber("/cortex/gripper/command", String, self._gripper_command_callback)

    def _interpolated_joint_command_callback(self, msg: JointPosVelAccCommand) -> None:
        """ Each incoming joint command message is stored off in an internal member for later
        processing as a StampedMsg() to track its age.
        """
        self._latest_stamped_command_msg = StampedMsg(rospy.Time.now(), msg)

    def _gripper_command_callback(self, msg: String) -> None:
        """ Each incoming gripper command message is stored off in an internal member for later
        processing as a StampedMsg() to track its age.
        """
        self._latest_stamped_gripper_command_msg = StampedMsg(rospy.Time.now(), msg)

    def _publish_joint_state_subset(self, articulation_subset: ArticulationSubset) -> None:
        """ Publish the subset of joint states governed by the provided articulation subset.

        Args:
            articulation_subset: The articulation subset to publish from.
        """
        names = articulation_subset.joint_names
        joint_state = articulation_subset.get_joints_state()

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.name = names
        msg.position = joint_state.positions
        msg.velocity = joint_state.velocities
        msg.effort = []
        self._joint_state_pub.publish(msg)

    def _on_simulation_step(self, step: float) -> None:
        """ The physics callback which processes all messages.

        Processes each command by transfering the command information to the associated commander on
        the sim robot.

        Also handles publishing joint state information for articulation subset associated with each
        commander.

        Args:
            step: The time delta between calls. This is part of the physics callback signature.
        """
        self._physics_call_count += 1
        if self._verbose:
            print("cortex_sim:", self._physics_call_count, "t:", time.time())

        now = rospy.Time.now()
        stamped_msg = self._latest_stamped_command_msg
        if stamped_msg is not None and not stamped_msg.has_expired(now):
            q = stamped_msg.msg.q
            qd = stamped_msg.msg.qd
            self.robot.arm.send(DirectSubsetCommander.Command(q, qd))

            stamped_gripper_msg = self._latest_stamped_gripper_command_msg
            if stamped_gripper_msg is not None and not stamped_gripper_msg.has_expired(now):
                self._latest_stamped_gripper_command_msg = None
                cmd = json.loads(stamped_gripper_msg.msg.data)
                self.robot.gripper.send(CortexGripper.Command(cmd["width"], speed=cmd["speed"], force=cmd["force"]))

        for _, commander in self.robot.commanders.items():
            self._publish_joint_state_subset(commander.articulation_subset)


class CortexObjectsRos:
    """ A tool for handling receiving measured poses for a collection of objects.

    The measured object poses are communicated over /tf using the standard ROS transform tree
    protocol. Each cycle, pose information for each object is written into the corresponding
    CortexObject as a CortexMeasuredPose. These CortexObject objects should be the same objects
    made available to the behaviors (decider networks) so those behaviors can decide what to do
    with the measured information (e.g. whether and how frequently to synchronize the object pose
    with the measured). 

    Incoming measured poses are automatically put into a ROS tf buffer, then processed from a
    physics callback.

    Args:
        objects: The collection of objects as a dict mapping name (used as the child_frame_id tf
            lookup) to object.
        auto_sync_objects: If True, the physics callback will automatically synchronize the measured
            poses to the belief poses via a call to cortex_object.sync_to_measured_pose().
        world_frame: The name of the world frame to query the object poses relative to.
    """

    def __init__(
        self,
        objects: Dict[str, CortexObject],
        auto_sync_objects: Optional[bool] = False,
        world_frame: Optional[str] = "world",
    ):
        self.objects = objects
        self.auto_sync_objects = auto_sync_objects
        self.world_frame = world_frame

        world = World.instance()  # Get the singleton.
        world.add_physics_callback("cortex_objects_ros_cb", self._on_simulation_step)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def _on_simulation_step(self, step: float) -> None:
        """ Process the measured poses.

        Queries the measured pose of each object passed on construction and sets the corresponding
        CortexObject's measured pose with that info. If there's a problem looking up the pose
        information, it skips that object and prints the exception to the console.

        If auto_sync_objects is set, automatically syncs the measured pose to the objects.
        (Otherwise, it's the behavior's responsibility to do that.)

        Args:
            step: The dt between steps. Required API for a physics callback.
        """
        in_coords = self.world_frame
        for name, obj in self.objects.items():
            try:
                child_frame_id = name
                transform_stamped = self._tf_buffer.lookup_transform(in_coords, child_frame_id, rospy.Time(0))
                p, q = ros_tf_util.transform_msg_to_pq(transform_stamped.transform)
                obj.set_measured_pose(CortexMeasuredPose(transform_stamped.header.stamp.to_sec(), (p, q), timeout=0.25))

                if self.auto_sync_objects:
                    obj.sync_to_measured_pose()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print("exception:", e)
                continue


class CortexSimObjectsRos:
    """ A tool to handle publishing the tfs for a collection of objects.

    Always publishes on /tf. Uses the ROS provided tf2_ros.TransformBroadcaster().

    Args:
        sim_objects: The objects whose tfs are to be published.
        world_frame: A name for the world frame when publishing tfs.
    """

    def __init__(self, sim_objects: Dict[str, XFormPrim], world_frame: Optional[str] = "world"):
        self.sim_objects = sim_objects
        self.world_frame = world_frame

        world = World.instance()  # Get the singleton.
        world.add_physics_callback("cortex_sim_objects_ros_cb", self._on_simulation_step)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def _on_simulation_step(self, step: float) -> None:
        """ Tfs are published on each simulation step via a physics callback.

        Args:
            step: The dt between steps. Required API for a physics callback.
        """
        self._publish_world_object_tfs()

    def _publish_world_object_tfs(self) -> None:
        """ Publish tfs for each of the objects supplied on construction.
        """
        frame_id = self.world_frame

        stamp = rospy.Time.now()
        for name, obj in self.sim_objects.items():
            T = math_util.pq2T(*obj.get_local_pose())
            child_frame_id = name
            tf = ros_tf_util.pack_transform_stamped(T, child_frame_id, frame_id, stamp)
            self.tf_broadcaster.sendTransform(tf)
