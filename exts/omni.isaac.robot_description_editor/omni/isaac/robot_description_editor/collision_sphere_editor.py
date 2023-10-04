# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from omni.isaac.core.objects.sphere import VisualSphere
from omni.isaac.core.utils.prims import is_prim_path_valid, delete_prim
from omni.isaac.core.utils.string import find_unique_string_name

from collections import OrderedDict
import yaml
import numpy as np
import carb
import lula


class CollisionSphereEditor:
    def __init__(self):
        self.path_2_spheres = {}
        self.path_2_sphere_serial_copy = {}

        self._operations = []

        self._redo = []

        self.filter = ""
        self.filter_in_sphere_color = np.array([37, 207, 188]) / 255  # All spheres that match filter
        self.filter_out_sphere_color = np.array([207.0, 184.0, 37.0]) / 255  # All Spheres that don't match filter

        self._preview_color = np.array([1.0, 0, 0])
        self._preview_spheres = []

    def clear_spheres(self, store_op=True):
        sphere_paths = list(self.path_2_spheres.keys())
        if len(sphere_paths) == 0:
            return

        if store_op:
            self.copy_all_sphere_data()
            deleted_spheres = ["DEL"]
            for sphere_path in sphere_paths:
                deleted_spheres.append(self.path_2_sphere_serial_copy[sphere_path])
            self._operations.append(deleted_spheres)

        for sphere_path in sphere_paths:
            self.delete_sphere(sphere_path)

    def clear_link_spheres(self, path, store_op=True):
        path_len = len(path)

        to_delete = []
        if store_op:
            self.copy_all_sphere_data()
        deleted_spheres = ["DEL"]
        for p in self.path_2_spheres.keys():
            if is_prim_path_valid(p) and p[:path_len] == path:
                deleted_spheres.append(self.path_2_sphere_serial_copy[p])
                to_delete.append(p)

        if store_op:
            self._operations.append(deleted_spheres)
        for s in to_delete:
            self.delete_sphere(s)

    def delete_sphere(self, sphere_path):
        if is_prim_path_valid(sphere_path):
            delete_prim(sphere_path)

        if sphere_path in self.path_2_spheres:
            del self.path_2_spheres[sphere_path]

    def set_sphere_colors(self, filter, color_in=None, color_out=None):
        if color_in is not None:
            self.filter_in_sphere_color = color_in
        if color_out is not None:
            self.filter_out_sphere_color = color_out
        self.filter = filter

        for sphere_path in self.path_2_spheres.keys():
            self.set_sphere_color(sphere_path)

    def set_sphere_color(self, sphere_path):
        if not is_prim_path_valid(sphere_path):
            return
        sphere = self.path_2_spheres[sphere_path]
        if sphere_path[: len(self.filter)] == self.filter:
            sphere.get_applied_visual_material().set_color(self.filter_in_sphere_color)
        else:
            sphere.get_applied_visual_material().set_color(self.filter_out_sphere_color)

    def copy_all_sphere_data(self):
        sphere_paths = list(self.path_2_spheres.keys())
        deleted_spheres = ["DEL"]
        for sphere_path in sphere_paths:
            if is_prim_path_valid(sphere_path):
                sphere = self.path_2_spheres[sphere_path]
                color = sphere.get_applied_visual_material().get_color()
                self.path_2_sphere_serial_copy[sphere_path] = {
                    "sphere_path": sphere_path,
                    "center": sphere.get_local_pose()[0],
                    "radius": sphere.get_radius(),
                    "color": color,
                }
            else:
                if sphere_path in self.path_2_sphere_serial_copy:
                    deleted_spheres.append(self.path_2_sphere_serial_copy[sphere_path])
                self.delete_sphere(sphere_path)
        if len(deleted_spheres) > 1:
            self._operations.append(deleted_spheres)

    def undo(self):
        if len(self._operations) == 0:
            return

        last_op = self._operations.pop()
        op_type = last_op[0]
        op = last_op[1:]

        if op_type == "ADD":
            redo = ["ADD"]
            for sphere_path in op:
                if is_prim_path_valid(sphere_path):
                    sphere = self.path_2_spheres[sphere_path]
                    redo.append(
                        {
                            "sphere_path": sphere_path,
                            "center": sphere.get_local_pose()[0],
                            "radius": sphere.get_radius(),
                        }
                    )
                self.delete_sphere(sphere_path)
            self._redo.append(redo)

        elif op_type == "DEL":
            redo = ["DEL"]
            for d in op:
                sphere = VisualSphere(d["sphere_path"], translation=d["center"], radius=d["radius"])
                self.path_2_spheres[d["sphere_path"]] = sphere
                self.set_sphere_color(d["sphere_path"])
                redo.append(d)
            self._redo.append(redo)

        elif op_type == "SCALE":
            redo = ["SCALE"]
            for d in op:
                path = d["sphere_path"]
                rad = d["radius"]
                factor = d["factor"]
                if path in self.path_2_spheres and is_prim_path_valid(path):
                    sphere = self.path_2_spheres[path]
                    sphere.set_radius(rad)
                    redo.append({"sphere_path": path, "radius": factor * rad})
            self._redo.append(redo)

    def redo(self):
        if len(self._redo) == 0:
            return

        last_redo = self._redo.pop()
        op_type = last_redo[0]
        op = last_redo[1:]

        if op_type == "ADD":
            added_spheres = ["ADD"]
            for d in op:
                sphere = VisualSphere(d["sphere_path"], translation=d["center"], radius=d["radius"])
                self.path_2_spheres[d["sphere_path"]] = sphere
                self.set_sphere_color(d["sphere_path"])
                added_spheres.append(d["sphere_path"])
            self._operations.append(added_spheres)

        elif op_type == "DEL":
            deleted_spheres = ["DEL"]
            for d in op:
                self.delete_sphere(d["sphere_path"])
                deleted_spheres.append(d)
            self._operations.append(deleted_spheres)

        elif op_type == "SCALE":
            for d in op:
                path = d["sphere_path"]
                rad = d["radius"]
                if path in self.path_2_spheres and is_prim_path_valid(path):
                    sphere = self.path_2_spheres[path]
                    sphere.set_radius(rad)

    def generate_spheres(self, link_path, points, face_inds, vert_cts, num_spheres, radius_offset, is_preview):
        if not is_preview and self._preview_spheres:
            # If preview spheres exist, change them to permanent spheres
            added_sphere_paths = ["ADD"]
            for preview_sphere in self._preview_spheres:
                if not preview_sphere.prim.IsValid():
                    continue
                sphere_path = self.add_sphere(
                    link_path, preview_sphere.get_local_pose()[0], preview_sphere.get_radius(), store_op=False
                )
                added_sphere_paths.append(sphere_path)
            self._operations.append(added_sphere_paths)
            self.clear_preview()
            return

        if np.unique(vert_cts) != [3]:
            self.clear_preview()
            carb.log_error(
                "Cannot generate collsision spheres for mesh because the specified mesh is not composed of triangles."
            )

        num_points = face_inds.shape[0]

        generator = lula.create_collision_sphere_generator(points, face_inds.reshape((num_points // 3, 3)))
        result = generator.generate_spheres(num_spheres, radius_offset)
        if is_preview:
            self.clear_preview()
            for lula_sphere in result:
                sphere_path = find_unique_string_name(
                    link_path + "/preview_sphere", lambda x: not is_prim_path_valid(x)
                )
                self._preview_spheres.append(
                    VisualSphere(
                        sphere_path,
                        color=self._preview_color,
                        translation=lula_sphere.center,
                        radius=lula_sphere.radius,
                    )
                )
        else:
            added_sphere_paths = ["ADD"]
            for lula_sphere in result:
                sphere_path = self.add_sphere(link_path, lula_sphere.center, lula_sphere.radius, store_op=False)
                added_sphere_paths.append(sphere_path)
            self._operations.append(added_sphere_paths)
            self.clear_preview()

    def clear_preview(self):
        for sphere in self._preview_spheres:
            self.delete_sphere(sphere.prim_path)
        self._preview_spheres = []

    def add_sphere(self, link_path, center, radius, store_op=True):
        if not is_prim_path_valid(link_path):
            carb.log_warn("Attempted to add sphere nested under non-existent path")

        if link_path[-1] == "/":
            link_path = link_path[:-1]

        self._redo = []
        prim_path = find_unique_string_name(link_path + "/collision_sphere", lambda x: not is_prim_path_valid(x))

        sphere = VisualSphere(prim_path, translation=center, radius=radius)
        self.path_2_spheres[sphere.prim_path] = sphere
        self.set_sphere_color(prim_path)
        if store_op:
            self._operations.append(["ADD", sphere.prim_path])

        return prim_path

    def load_spheres(self, robot, robot_description_file_path):
        self.clear_spheres(store_op=False)
        self._redo = []
        self._operations = []

        with open(robot_description_file_path, "r") as stream:
            try:
                parsed_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        sphere_list = parsed_file["collision_spheres"]

        robot_path = robot.prim_path

        added_sphere_paths = ["ADD"]
        for sphere_dict in sphere_list:
            for key, val in sphere_dict.items():
                link_path = robot_path + "/" + key
                if is_prim_path_valid(link_path):
                    for sphere in val:
                        center = np.array(sphere["center"])
                        radius = sphere["radius"]
                        sphere_path = self.add_sphere(link_path, center, radius, store_op=False)
                        added_sphere_paths.append(sphere_path)
                else:
                    carb.log_warn("Could not place sphere from robot description at path: {}".format(link_path))

        self._operations.append(added_sphere_paths)

    def interpolate_spheres(self, path1, path2, num_spheres):
        if not is_prim_path_valid(path1):
            carb.log_warn("{} is not a valid Prim path to a sphere".format(path1))
            return
        elif not is_prim_path_valid(path2):
            carb.log_warn("{} is not a valid Prim path to a sphere".format(path2))
            return

        link_path = self._get_link_path(path1)
        if self._get_link_path(path2) != link_path:
            carb.log_warn(
                "Prim paths {} and {} are not nested under the same link.  They cannot be interpolated.".format(
                    path1, path2
                )
            )

        epsilon = 1e-12

        sphere_1 = self.path_2_spheres[path1]
        sphere_2 = self.path_2_spheres[path2]

        rad_1 = max(sphere_1.get_radius(), epsilon)
        rad_2 = max(sphere_2.get_radius(), epsilon)

        t1 = sphere_1.get_local_pose()[0]
        t2 = sphere_2.get_local_pose()[0]

        d = t2 - t1

        # Assign radii for interpolated spheres according to a geometric sequence.
        rads = np.geomspace(rad_1, rad_2, num=num_spheres + 2)

        # Position spheres so that they're tangent to an enclosing cone.
        #
        # When the two endpoint spheres have the same radius, the interpolated spheres
        # will be evenly spaced.  The general expression below gives this result when
        # the equal-radius limit is taken analytically, but for the purpose of numerical
        # evaluation we must handle this case separately to avoid division by zero.
        if abs((rad_1 - rad_2) / (rad_1 + rad_2)) < epsilon:
            relative_offsets = np.linspace(0.0, 1.0, num=num_spheres + 2)
        else:
            relative_offsets = (rads - rad_1) / (rad_2 - rad_1)

        added_sphere_paths = ["ADD"]
        for i in range(1, num_spheres + 1):
            sphere_path = self.add_sphere(link_path, t1 + relative_offsets[i] * d, rads[i], store_op=False)
            added_sphere_paths.append(sphere_path)
        self._operations.append(added_sphere_paths)

    def scale_spheres(self, path, factor):
        scaled_spheres = ["SCALE"]
        path_len = len(path)

        for p in self.path_2_spheres.keys():
            if is_prim_path_valid(p) and p[:path_len] == path:
                sphere = self.path_2_spheres[p]
                rad = sphere.get_radius()
                sphere.set_radius(factor * rad)
                scaled_spheres.append({"sphere_path": p, "radius": rad, "factor": factor})
        self._operations.append(scaled_spheres)

    def get_sphere_names_by_link(self, link_path):
        sphere_names = []
        for sphere_path in self.path_2_spheres.keys():
            sphere_link_path = self._get_link_path(sphere_path)
            if sphere_link_path == link_path:
                sphere_names.append(sphere_path[len(link_path) :])

        return sphere_names

    def save_spheres(self, robot, f):
        link_to_spheres = OrderedDict()
        robot_path_split = robot.prim_path.split("/")
        for sphere in self.path_2_spheres.values():
            prim_path = sphere.prim_path
            if is_prim_path_valid(prim_path):
                s = prim_path.split("/")
                if s[:-2] != robot_path_split:
                    carb.log_warn(
                        "Not writing sphere at path {} to file because it is not nested under the robot Articulation".format(
                            prim_path
                        )
                    )
                    continue
                link_name = s[-2]
                link_spheres = link_to_spheres.get(link_name, [])
                sphere_pose = self._round_list_floats(sphere.get_local_pose()[0])
                link_spheres.append({"center": sphere_pose, "radius": round(sphere.get_radius(), 3)})
                link_to_spheres[link_name] = link_spheres

        f.write("collision_spheres:\n")
        for link_name, sphere_list in link_to_spheres.items():
            f.write("  - {}:\n".format(link_name))
            for sphere in sphere_list:
                f.write('    - "center": {}\n'.format(sphere["center"]))
                f.write('      "radius": {}\n'.format(sphere["radius"]))

    def _round_list_floats(self, l, decimals=3):
        r = []
        for f in l:
            r.append(round(f, decimals))
        return r

    def _get_link_path(self, sphere_path):
        # Remove last element of Prim path to sphere

        slash_ind = sphere_path.rfind("/")

        link_path = sphere_path[:slash_ind]
        return link_path
