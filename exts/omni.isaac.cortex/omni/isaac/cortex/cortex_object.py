# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import numpy as np
import time
from typing import Optional, Tuple, Sequence

from pxr import Gf, Usd

from omni.isaac.core.utils.rotations import gf_quat_to_np_array
from omni.isaac.core.prims import XFormPrim
import omni.isaac.cortex.math_util as math_util


class CortexMeasuredPose(object):
    """ Contains information about the measured pose of an object.

    This includes the time stamp of the measurement, the pose, and a timeout (time to live) defining
    how long we trust this measurement.

    Args:
        stamp: The timestamp of the measurement.
        pose_pq: A tuple (p,q) containing the position and quaternion of the measurement.
        timeout: How long we trust this measurement (time to live).
    """

    def __init__(self, stamp: float, pose_pq: Tuple[np.ndarray, np.ndarray], timeout: float):
        self.stamp = stamp
        self.pq = pose_pq
        self.timeout = timeout

    def is_valid(self, time: float) -> bool:
        """ Returns whether this measurement is still valid based on the time stamp and its timeout.

        Args:
            time: The current time.

        Returns:
            Whether the measurement has not yet timed out (True if it's valid, False if timeout).
        """
        return time - self.stamp < self.timeout


class CortexObject(object):
    """ A CortexObject is an object (derived from the core API XFormPrim) which may have measurement
    information from perception.
    
    It handles recording that measurement information and providing an API to both access it and
    sync it to the underlying object. Since perception modules differ dramatically in their
    performance characteristics, the specifics of how that measured pose is synchronized to the
    underlying object is left to the user.

    Args:
        obj: The underlying object in the scene, wrapped in a core API class deriving from
            XFormPrim.
        sync_throttle_dt: Prevents synchronization (via sync_to_measured_pose()) within this number
            of seconds of a previous sync. Defaults to None, which means no throttling.
    """

    def __init__(self, obj: XFormPrim, sync_throttle_dt: float = None):
        self.obj = obj
        self.time_at_last_sync = None
        self.sync_throttle_dt = sync_throttle_dt
        self.measured_pose = None
        self.sync_sim = False

    @property
    def name(self) -> str:
        """ The name of the underlying object.
        """
        return self.obj.name

    @property
    def prim(self) -> Usd.Prim:
        """ The underlying USD prim representing this object.
        """
        return self.obj.prim

    def set_world_pose(
        self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """ Set the object's world pose.
        """
        self.obj.set_world_pose(position, orientation)

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """ Get the object's world pose.
        """
        return self.obj.get_world_pose()

    def get_transform(self) -> np.ndarray:
        """ Returns the object's world pose (in meters) as a 4x4 homogeneous matrix.
        """
        position, orientation = self.get_world_pose()
        return math_util.pq2T(position, orientation)

    def get_T(self):
        """ Convenience accessor for get_transform() using T naming convention.
        """
        return self.get_transform()

    def set_measured_pose(self, measured_pose: CortexMeasuredPose) -> None:
        """ Set the measured pose of this object

        Args:
            measured_pose: The measurement information.
        """
        self.measured_pose = measured_pose

    def has_measured_pose(self) -> bool:
        """ Queries whether this object has a valid measured pose.

        A measured pose is valid if it's both available (has been set) and it's valid per the
        CortexMeasuredPose.is_valid() method.

        Returns: The truth value of whether it has a valid measured pose.
        """
        return self.measured_pose is not None and self.measured_pose.is_valid(time.time())

    def get_measured_pq(self) -> Tuple[np.ndarray, np.ndarray]:
        """ Returns the measured pose as a (p,q) tuple in meters.
        
        This method doesn't check whether the measured pose is available. Use has_measured_pose() to
        verify.

        Returns: (p, q) containing the position p and quaternion q of the measured pose.
        """
        return self.measured_pose.pq

    def get_measured_T(self) -> np.array:
        """ Returns the measured pose as a 4x4 homogeneous matrix in units of meters.

        This method doesn't check whether the measured pose is available. Use has_measured_pose() to
        verify.

        Returns: A homogeneous transform matrix T representing the latest measured pose.
        """
        p, q = self.measured_pose.pq
        return math_util.pq2T(p, q)

    def sync_to_measured_pose(self, use_throttle: bool = True) -> None:
        """ Syncs the pose of the underlying USD object to match the measured pose.
        
        If use_throttle is True (default) when this method will prevent two syncs from happening
        within sync_throttle_dt seconds of one another.  i.e. it throttles the rate to <
        1./sync_throttle_dt.

        This method doesn't check whether the measured pose is available. Use has_measured_pose() to
        verify.

        Args:
            use_throttle: Whether or not to use the throttling. Defaults to True. Note that this
                will only throttle, even when True, when sync_throttle_dt is not None.
        """
        current_time = time.time()

        if not self.has_measured_pose():
            # There's nothing to sync to.
            return

        if (
            self.time_at_last_sync is not None
            and use_throttle
            and self.sync_throttle_dt is not None
            and (current_time - self.time_at_last_sync < self.sync_throttle_dt)
        ):
            # Don't sync this cycle.
            return

        # Write the measured pose to the object's USD. The TensorAPI will automatically pull that
        # in. (Note if we just use the core API, that'll directly access the tensor API and the USD
        # won't be updated if the object is asleep (w.r.t. PhysX), so visually the object won't
        # sync until it's moved.
        self._sync_tensor_api_to_usd(*self.get_measured_pq())
        self.time_at_last_sync = current_time

    def _sync_tensor_api_to_usd(self, p: np.ndarray, q: np.ndarray) -> None:
        """ Internal method used to synchronize the tensor API to the USD for this object. The Isaac
        Sim core API goes through the tensor API, but the tensor API is only synced to USD when the
        object is active. If we receive a measured pose, we want to sync to USD regardless of
        whether the object is active so it's visualized correctly.
        """
        p = p.astype(float)
        q = q.astype(float)

        p_attr = self.obj.prim.GetAttribute("xformOp:translate")
        p_attr.Set(Gf.Vec3d(p[0], p[1], p[2]))

        w, x, y, z = q
        q_attr = self.obj.prim.GetAttribute("xformOp:orient")
        q_attr.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))

        verbose = False  # Set to True to get debugging diagnostics.
        if verbose:
            p_gf = p_attr.Get()
            q_gf = q_attr.Get()
            print("[{}] p: {}, p_gf: {} -- q: {}, q_gf: {}".format(self.name, p, p_gf, q, q_gf))
