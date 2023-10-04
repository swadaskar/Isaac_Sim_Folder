# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import numpy as np
import time

import omni.ext
import carb
import omni.usd
from pxr import UsdGeom, Usd, Gf, Sdf
from omni.physx import get_physx_interface, get_physx_scene_query_interface
from omni.usd._impl.utils import get_prim_at_path, get_world_transform_matrix
from typing import List, Dict


class Sensor:
    def __init__(self, parent: Usd.Prim, callback_fns=[None, None, None], exclusions=[]):
        self.parent = parent
        self._callback_fns = callback_fns
        self._exclusions = exclusions

        self._active_zones = []
        self._prev_active_zones = []
        self._entered_zones = []
        self._exited_zones = []

        self._data = {}
        self.name = str(self.parent.GetPath()).rpartition("/")[-1]

        self._is_inside = False
        self.overlapping = False
        self.distance = 0
        self.stage = omni.usd.get_context().get_stage()

    def update(self):

        # get any active zones
        self._prev_active_zones = self._active_zones
        self._active_zones = []  # clear the active zones
        num_hits = self.check_for_overlap()  # update active_zones

        # check if we have entered or exited a zone
        if len(self._active_zones) != len(self._prev_active_zones):
            entered_zones = list(set(self._active_zones) - set(self._prev_active_zones))
            if len(entered_zones) > 0:
                self._entered_zones = entered_zones
                # Pass to external on_enter callback_fn
                if self._callback_fns[0] is not None:
                    self._callback_fns[0](self)
            else:
                self._entered_zones = []
            exited_zones = list(set(self._prev_active_zones) - set(self._active_zones))
            if len(exited_zones) > 0:
                self._exited_zones = exited_zones

                # remove timers from dict on exit
                for zone in self._exited_zones:
                    self._data.pop(zone)

                # Pass to external on_exit callback_fn
                if self._callback_fns[2] is not None:
                    self._callback_fns[2](self)
            else:
                self._exited_zones = []

        is_inside = len(self._active_zones) != 0
        # Check for a state change
        if self._is_inside != is_inside:
            # Reset the Tracker if we've moved outside
            if not is_inside:
                self.reset()
            # Update _is_inside
            self._is_inside = is_inside
            self.overlapping = is_inside

        # Pass to external is_inside callback_fn
        if self._is_inside:

            # update the overlap data
            for key, val in self._data.items():
                # update timer
                val["duration"] = time.time() - val["start_time"]
                # update the distance
                pos_a = get_world_transform_matrix(self.parent).ExtractTranslation()
                prim = get_prim_at_path(Sdf.Path(key))
                pos_b = get_world_transform_matrix(prim).ExtractTranslation()
                a = np.array((pos_a[0], pos_a[1], pos_a[2]))
                b = np.array((pos_b[0], pos_b[1], pos_b[2]))
                val["distance"] = np.linalg.norm(a - b)
            if self._callback_fns[1] is not None:
                self._callback_fns[1](self)

        return

    def report_hit(self, hit):
        path = str(UsdGeom.Mesh.Get(omni.usd.get_context().get_stage(), hit.rigid_body).GetPrim().GetPrimPath())
        # Add to the active zone list
        if path not in self._active_zones and path not in self._exclusions:
            self._active_zones.append(path)

        # Start the timer
        if not path in self._data and not path in self._exclusions:
            self._data[path] = {"start_time": time.time(), "duration": 0}

        return True  # return True to continue the query

    def check_for_overlap(self):
        self.parent_pose = omni.usd.utils.get_world_transform_matrix(self.parent)
        extent = self.parent.GetPropertyAtPath(str(self.parent.GetPath()) + ".xformOp:scale").Get()
        origin = self.parent_pose.ExtractTranslation()
        rot = self.parent_pose.ExtractRotation().GetQuat()

        # Pass 'False' to overlap_box() to indicate an 'overlap multiple' query.
        return get_physx_scene_query_interface().overlap_box(
            carb.Float3(extent[0] * 0.5, extent[1] * 0.5, extent[2] * 0.5),
            carb.Float3(origin[0], origin[1], origin[2]),
            carb.Float4(rot.GetImaginary()[0], rot.GetImaginary()[1], rot.GetImaginary()[2], rot.GetReal()),
            self.report_hit,
            False,
        )

    def status(self):
        return (self.overlapping, self._data)

    def reset(self):
        self._is_inside = False
        self._active_zones = []
        self._entered_zones = []
        self._exited_zones = []
        self._data.clear()

    def get_data(self) -> Dict[str, Dict[str, float]]:
        """
        Returns dictionary of overlapped geometry and respective metadata.
            
            key: prim_path of overlapped geometry
            val: dictionary of metadata:

                "duration": float of time since overlap
                "distance": distance from origin of tracker to origin of overlapped geometry

        Returns:
            Dict[str, Dict[str, float]]: overlapped geometry and metadata
        """
        return self._data

    def to_string(self):
        msg = f"Tracker: {self.parent.GetPath()}, \tname: {self.name}, \tactive_zones: "
        for key, val in self._data.items():
            duration = "%.4f" % val["duration"]
            distance = "%.4f" % val["distance"]
            msg += f"({key}, duration: {duration}, distance: {distance}), "
        return msg

    def is_overlapping(self):
        return self.overlapping

    def get_active_zones(self) -> List[str]:
        """Returns a list of the prim paths of all the collision meshes the tracker is inside of.

        Returns:
            list(str): prim paths as strings
        """
        return self._active_zones

    def get_entered_zones(self) -> List[str]:
        """Returns a list of the prim paths of all the collision meshes the tracker just entered.

        Returns:
            list(str): prim paths as strings
        """
        return self._entered_zones

    def get_exited_zones(self) -> List[str]:
        """Returns a list of the prim paths of all the collision meshes the tracker just exited.

        Returns:
            list(str): prim paths as strings
        """
        return self._exited_zones


class SensorManager(object):
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(SensorManager, cls).__new__(cls)
            cls.sensors = []
        return cls._instance

    def register_sensor(self, sensor: Sensor):
        self.sensors.append(sensor)

    def clear_sensors(self):
        self.sensors = []

    def update(self):
        for sensor in self.sensors:
            sensor.update()


# Public API:
def register_sensor(sensor: Sensor):
    SensorManager().register_sensor(sensor)


def clear_sensors():
    SensorManager().clear_sensors()


class Extension(omni.ext.IExt):
    def on_startup(self):
        # step the sensor every physics step
        self._sub = get_physx_interface().subscribe_physics_step_events(self._on_update)
        self._sm = SensorManager()  # Store instance to sensor manager singleton

    def on_shutdown(self):
        self._sub = None  # clear subscription
        clear_sensors()  # clear sensors on shutdown
        self._sm = None

    def _on_update(self, dt):
        self._sm.update()
        pass


# Example usage, create a physics scene and a Cube first
# from omni.isaac.proximity_sensor import Sensor, register_sensor, clear_sensors
# import carb
# import omni
# half_extent = carb.Float3(25,25,25)
# stage = omni.usd.get_context().get_stage()
# parent = stage.GetPrimAtPath("/World/Cube")
# s = Sensor(half_extent, parent)
# register_sensor(s)
