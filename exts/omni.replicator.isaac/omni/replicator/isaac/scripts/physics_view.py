# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import asyncio
from typing import List, Optional, Tuple, Union
import copy

import carb
import numpy as np
import torch
import omni.kit
import omni.timeline
import omni.usd

import omni.graph.core as og

from omni.isaac.core.utils.torch.rotations import get_euler_xyz as quat_to_euler_torch
from omni.isaac.core.utils.numpy.rotations import quats_to_euler_angles as quat_to_euler_numpy

from omni.replicator.core import distribution
from omni.replicator.core.scripts.utils import ReplicatorItem, ReplicatorWrapper, utils
from omni.replicator.isaac.scripts import context
from .context import trigger_randomization
from .attributes import TENDON_ATTRIBUTES

_simulation_context = None
_rigid_prim_views = dict()
_articulation_views = dict()

_simulation_context_initial_values = dict()
_rigid_prim_views_initial_values = dict()
_articulation_views_initial_values = dict()
_current_tendon_properties = dict()

_simulation_context_reset_values = dict()
_rigid_prim_views_reset_values = dict()
_articulation_views_reset_values = dict()


def register_simulation_context(
    simulation_context: Union[omni.isaac.core.SimulationContext, omni.isaac.core.World]
) -> None:
    """ 
        Args:
            simulation_context (Union[omni.isaac.core.SimulationContext, omni.isaac.core.World]): Registering the SimulationContext.
    """
    global _simulation_context
    _simulation_context = simulation_context
    direction, magnitude = _simulation_context.get_physics_context().get_gravity()
    gravity_vector = np.array(direction) * magnitude
    _simulation_context_initial_values["gravity"] = gravity_vector
    _simulation_context_reset_values["gravity"] = copy.deepcopy(gravity_vector)


def register_rigid_prim_view(rigid_prim_view: omni.isaac.core.prims.RigidPrimView) -> None:
    """ 
        Args:
            rigid_prim_view (omni.isaac.core.prims.RigidPrimView): Registering the RigidPrimView to be randomized.
    """
    clone_tensor = rigid_prim_view._backend_utils.clone_tensor
    tensor_cat = rigid_prim_view._backend_utils.tensor_cat
    create_zeros_tensor = rigid_prim_view._backend_utils.create_zeros_tensor
    device = rigid_prim_view._device

    name = rigid_prim_view.name
    _rigid_prim_views[name] = rigid_prim_view
    initial_values = dict()
    pos, quats = rigid_prim_view.get_world_poses()
    initial_values["position"] = pos
    if rigid_prim_view._backend == "torch":
        r, p, y = quat_to_euler_torch(quats)
        initial_values["orientation"] = torch.stack((r, p, y), dim=-1)
    elif rigid_prim_view._backend == "numpy":
        initial_values["orientation"] = quat_to_euler_numpy(quats)

    initial_values["linear_velocity"] = rigid_prim_view.get_linear_velocities()
    initial_values["angular_velocity"] = rigid_prim_view.get_angular_velocities()
    initial_values["velocity"] = tensor_cat(
        [initial_values["linear_velocity"], initial_values["angular_velocity"]], dim=-1
    )
    initial_values["force"] = create_zeros_tensor(
        shape=[initial_values["position"].shape[0], 3], dtype="float32", device=device
    )
    initial_values["mass"] = clone_tensor(rigid_prim_view._physics_view.get_masses(), device="cpu")
    initial_values["inertia"] = clone_tensor(
        rigid_prim_view._physics_view.get_inertias()[:, [0, 4, 8]], device="cpu"  # extract the diagonals
    )
    initial_values["material_properties"] = clone_tensor(
        rigid_prim_view._physics_view.get_material_properties(), device="cpu"
    ).reshape(rigid_prim_view.count, rigid_prim_view._physics_view.max_shapes * 3)
    initial_values["contact_offset"] = clone_tensor(rigid_prim_view._physics_view.get_contact_offsets(), device="cpu")
    initial_values["rest_offset"] = clone_tensor(rigid_prim_view._physics_view.get_rest_offsets(), device="cpu")
    _rigid_prim_views_initial_values[name] = initial_values
    _rigid_prim_views_reset_values[name] = copy.deepcopy(initial_values)


def register_articulation_view(articulation_view: omni.isaac.core.articulations.ArticulationView) -> None:
    """ 
        Args:
            articulation_view (omni.isaac.core.articulations.ArticulationView): Registering the ArticulationView to be randomized.
    """
    clone_tensor = articulation_view._backend_utils.clone_tensor
    tensor_cat = articulation_view._backend_utils.tensor_cat
    create_zeros_tensor = articulation_view._backend_utils.create_zeros_tensor
    device = articulation_view._device

    name = articulation_view.name
    _articulation_views[name] = articulation_view
    initial_values = dict()
    initial_values["stiffness"] = clone_tensor(articulation_view._default_kps, device="cpu")
    initial_values["damping"] = clone_tensor(articulation_view._default_kds, device="cpu")
    initial_values["joint_friction"] = clone_tensor(
        articulation_view._physics_view.get_dof_friction_coefficients(), device="cpu"
    )
    initial_values["position"] = articulation_view._default_state.positions

    quats = articulation_view._default_state.orientations
    if articulation_view._backend == "torch":
        r, p, y = quat_to_euler_torch(quats)
        initial_values["orientation"] = torch.stack((r, p, y), dim=-1)
    elif articulation_view._backend == "numpy":
        initial_values["orientation"] = quat_to_euler_numpy(quats)

    initial_values["linear_velocity"] = articulation_view.get_linear_velocities()
    initial_values["angular_velocity"] = articulation_view.get_angular_velocities()
    initial_values["velocity"] = tensor_cat(
        [initial_values["linear_velocity"], initial_values["angular_velocity"]], dim=-1
    )
    initial_values["joint_positions"] = articulation_view._default_joints_state.positions
    initial_values["joint_velocities"] = articulation_view._default_joints_state.positions
    initial_values["lower_dof_limits"] = clone_tensor(articulation_view.get_dof_limits()[..., 0], device="cpu")
    initial_values["upper_dof_limits"] = clone_tensor(articulation_view.get_dof_limits()[..., 1], device="cpu")
    initial_values["max_efforts"] = clone_tensor(articulation_view.get_max_efforts(), device="cpu")
    initial_values["joint_armatures"] = clone_tensor(articulation_view._physics_view.get_dof_armatures(), device="cpu")
    initial_values["joint_max_velocities"] = clone_tensor(
        articulation_view._physics_view.get_dof_max_velocities(), device="cpu"
    )
    initial_values["joint_efforts"] = create_zeros_tensor(
        shape=[initial_values["stiffness"].shape[0], initial_values["stiffness"].shape[1]],
        dtype="float32",
        device=device,
    )
    initial_values["body_masses"] = clone_tensor(articulation_view._physics_view.get_masses(), device="cpu")
    initial_values["body_inertias"] = clone_tensor(
        articulation_view._physics_view.get_inertias()[:, :, [0, 4, 8]], device="cpu"
    ).reshape(articulation_view.count, articulation_view._physics_view.max_links * 3)
    initial_values["material_properties"] = clone_tensor(
        articulation_view._physics_view.get_material_properties(), device="cpu"
    ).reshape(articulation_view.count, articulation_view._physics_view.max_shapes * 3)
    initial_values["contact_offset"] = clone_tensor(articulation_view._physics_view.get_contact_offsets(), device="cpu")
    initial_values["rest_offset"] = clone_tensor(articulation_view._physics_view.get_rest_offsets(), device="cpu")

    if articulation_view._physics_view.max_fixed_tendons > 0:
        initial_values["tendon_stiffnesses"] = clone_tensor(
            articulation_view._physics_view.get_fixed_tendon_stiffnesses(), device=device
        )
        initial_values["tendon_dampings"] = clone_tensor(
            articulation_view._physics_view.get_fixed_tendon_dampings(), device=device
        )
        initial_values["tendon_limit_stiffnesses"] = clone_tensor(
            articulation_view._physics_view.get_fixed_tendon_limit_stiffnesses(), device=device
        )
        tendon_limits = clone_tensor(articulation_view._physics_view.get_fixed_tendon_limits(), device=device).reshape(
            articulation_view.count, articulation_view._physics_view.max_fixed_tendons, 2
        )
        initial_values["tendon_lower_limits"] = tendon_limits[..., 0]
        initial_values["tendon_upper_limits"] = tendon_limits[..., 1]
        initial_values["tendon_rest_lengths"] = clone_tensor(
            articulation_view._physics_view.get_fixed_tendon_rest_lengths(), device=device
        )
        initial_values["tendon_offsets"] = clone_tensor(
            articulation_view._physics_view.get_fixed_tendon_offsets(), device=device
        )

        for attribute in TENDON_ATTRIBUTES:
            _current_tendon_properties[attribute] = clone_tensor(initial_values[attribute], device=device)

    _articulation_views_initial_values[name] = initial_values
    _articulation_views_reset_values[name] = copy.deepcopy(initial_values)


def step_randomization(reset_inds: Optional[Union[list, np.ndarray, torch.Tensor]] = list()) -> None:
    """ 
        Args:
            reset_inds (Optional[Union[list, np.ndarray, torch.Tensor]]): The indices corresonding to the prims to be reset in the views.
    """
    if torch.is_tensor(reset_inds):
        trigger_randomization(reset_inds.cpu().numpy())
    else:
        trigger_randomization(np.asarray(reset_inds))


@ReplicatorWrapper
def _write_physics_view_node(view, attribute, values, operation, node_type, num_buckets=None):
    node = utils.create_node(node_type)
    node.get_attribute("inputs:attribute").set(attribute)
    node.get_attribute("inputs:prims").set(view)
    node.get_attribute("inputs:operation").set(operation)

    if not isinstance(values, ReplicatorItem):
        values = distribution.uniform(values, values)

    if num_buckets is not None:
        node.get_attribute("inputs:num_buckets").set(num_buckets)
        if values.node.get_node_type().get_node_type() == "omni.replicator.core.OgnSampleUniform":
            node.get_attribute("inputs:distribution").set("uniform")
            values.node.get_attribute("inputs:lower").connect(node.get_attribute("inputs:dist_param_1"), True)
            values.node.get_attribute("inputs:upper").connect(node.get_attribute("inputs:dist_param_2"), True)
        elif values.node.get_node_type().get_node_type() == "omni.replicator.core.OgnSampleNormal":
            node.get_attribute("inputs:distribution").set("gaussian")
            values.node.get_attribute("inputs:mean").connect(node.get_attribute("inputs:dist_param_1"), True)
            values.node.get_attribute("inputs:std").connect(node.get_attribute("inputs:dist_param_2"), True)
        elif values.node.get_node_type().get_node_type() == "omni.replicator.core.OgnSampleLogUniform":
            node.get_attribute("inputs:distribution").set("loguniform")
            values.node.get_attribute("inputs:lower").connect(node.get_attribute("inputs:dist_param_1"), True)
            values.node.get_attribute("inputs:upper").connect(node.get_attribute("inputs:dist_param_2"), True)

    counter = ReplicatorItem(utils.create_node, "omni.replicator.isaac.OgnCountIndices")

    upstream_node = ReplicatorItem.get_context()
    upstream_node.get_attribute("outputs:indices").connect(counter.node.get_attribute("inputs:indices"), True)
    counter.node.get_attribute("outputs:count").connect(values.node.get_attribute("inputs:numSamples"), True)
    upstream_node.get_attribute("outputs:indices").connect(node.get_attribute("inputs:indices"), True)
    upstream_node.get_attribute("outputs:on_reset").connect(node.get_attribute("inputs:on_reset"), True)

    utils.auto_connect(values.node, node)
    return node


@ReplicatorWrapper
def randomize_rigid_prim_view(
    view_name: str,
    operation: str = "direct",
    num_buckets: int = None,
    position: ReplicatorItem = None,
    orientation: ReplicatorItem = None,
    linear_velocity: ReplicatorItem = None,
    angular_velocity: ReplicatorItem = None,
    velocity: ReplicatorItem = None,
    force: ReplicatorItem = None,
    mass: ReplicatorItem = None,
    inertia: ReplicatorItem = None,
    material_properties: ReplicatorItem = None,
    contact_offset: ReplicatorItem = None,
    rest_offset: ReplicatorItem = None,
) -> None:
    """ 
        Args:
            view_name (str): The name of a registered RigidPrimView.
            operation (str): Can be "direct", "additive", or "scaling".
                             "direct" means random values are directly written into the view.
                             "additive" means random values are added to the default values.
                             "scaling" means random values are multiplied to the default values.
            num_buckets (int): Number of buckets to sample values from for material_properties randomization.
            position (ReplicatorItem): Randomizes the position of the prims.
            orientation (ReplicatorItem): Randomizes the orientation of the prims using euler angles (rad).
            linear_velocity (ReplicatorItem): Randomizes the linear velocity of the prims.
            angular_velocity (ReplicatorItem): Randomizes the angular velocity of the prims.
            velocity (ReplicatorItem): Randomizes the linear and angular velocity of the prims.
            force (ReplicatorItem): Applies a random force to the prims.
            mass (ReplicatorItem): Randomizes the mass of the prims. CPU pipeline only.
            inertia (ReplicatorItem): Randomizes the inertia of the prims. Takes in three values for the
                                       replicator distribution, corresponding to the diagonal elements of 
                                       the inertia matrix. CPU pipeline only.
            material_properties (ReplicatorItem): Takes in three values for the replicator distriution,
                                                  corresponding to static friction, dynamic friction,
                                                  and restitution.
            contact_offset (ReplicatorItem): Randomizes the contact offset of the prims.
            rest_offset (ReplicatorItem): Randomizes the rest offset of the prims.
    """

    # check whether randomization occurs within the correct context
    upstream_node_name = ReplicatorItem.get_context().get_node_type().get_node_type()
    if upstream_node_name != "omni.replicator.isaac.OgnIntervalFiltering":
        raise ValueError(
            "randomize_rigid_prim_view() is expected to be called within the omni.replicator.isaac.randomize.on_interval"
            + " or omni.replicator.isaac.randomize.on_env_reset context managers."
        )

    node_type = "omni.replicator.isaac.OgnWritePhysicsRigidPrimView"

    if _rigid_prim_views.get(view_name) is None:
        raise ValueError(f"Expected a registered rigid prim view, but instead received {view_name}")

    if position is not None:
        _write_physics_view_node(view_name, "position", position, operation, node_type)
    if orientation is not None:
        _write_physics_view_node(view_name, "orientation", orientation, operation, node_type)
    if linear_velocity is not None:
        _write_physics_view_node(view_name, "linear_velocity", linear_velocity, operation, node_type)
    if angular_velocity is not None:
        _write_physics_view_node(view_name, "angular_velocity", angular_velocity, operation, node_type)
    if velocity is not None:
        _write_physics_view_node(view_name, "velocity", velocity, operation, node_type)
    if force is not None:
        _write_physics_view_node(view_name, "force", force, operation, node_type)
    if mass is not None:
        _write_physics_view_node(view_name, "mass", mass, operation, node_type)
    if inertia is not None:
        _write_physics_view_node(view_name, "inertia", inertia, operation, node_type)
    if material_properties is not None:
        _write_physics_view_node(
            view_name, "material_properties", material_properties, operation, node_type, num_buckets
        )
    if contact_offset is not None:
        _write_physics_view_node(view_name, "contact_offset", contact_offset, operation, node_type)
    if rest_offset is not None:
        _write_physics_view_node(view_name, "rest_offset", rest_offset, operation, node_type)


@ReplicatorWrapper
def randomize_articulation_view(
    view_name: str,
    operation: str = "direct",
    num_buckets: int = None,
    stiffness: ReplicatorItem = None,
    damping: ReplicatorItem = None,
    joint_friction: ReplicatorItem = None,
    position: ReplicatorItem = None,
    orientation: ReplicatorItem = None,
    linear_velocity: ReplicatorItem = None,
    angular_velocity: ReplicatorItem = None,
    velocity: ReplicatorItem = None,
    joint_positions: ReplicatorItem = None,
    joint_velocities: ReplicatorItem = None,
    lower_dof_limits: ReplicatorItem = None,
    upper_dof_limits: ReplicatorItem = None,
    max_efforts: ReplicatorItem = None,
    joint_armatures: ReplicatorItem = None,
    joint_max_velocities: ReplicatorItem = None,
    joint_efforts: ReplicatorItem = None,
    body_masses: ReplicatorItem = None,
    body_inertias: ReplicatorItem = None,
    material_properties: ReplicatorItem = None,
    tendon_stiffnesses: ReplicatorItem = None,
    tendon_dampings: ReplicatorItem = None,
    tendon_limit_stiffnesses: ReplicatorItem = None,
    tendon_lower_limits: ReplicatorItem = None,
    tendon_upper_limits: ReplicatorItem = None,
    tendon_rest_lengths: ReplicatorItem = None,
    tendon_offsets: ReplicatorItem = None,
) -> None:
    """ 
        Args:
            view_name (str): The name of a registered ArticulationView.
            operation (str): Can be "direct", "additive", or "scaling".
                             "direct" means random values are directly written into the view.
                             "additive" means random values are added to the default values.
                             "scaling" means random values are multiplied to the default values.
            num_buckets (int): Number of buckets to sample values from for material_properties randomization.
            stiffness (ReplicatorItem): Randomizes the stiffness of the joints in the articulation prims.
            damping (ReplicatorItem): Randomizes the damping of the joints in the articulation prims.
            joint_friction (ReplicatorItem): Randomizes the friction of the joints in the articulation prims.
            position (ReplicatorItem): Randomizes the root position of the prims.
            orientation (ReplicatorItem): Randomizes the root orientation of the prims using euler angles (rad).
            linear_velocity (ReplicatorItem): Randomizes the root linear velocity of the prims.
            angular_velocity (ReplicatorItem): Randomizes the root angular velocity of the prims.
            velocity (ReplicatorItem): Randomizes the root linear and angular velocity of the prims.
            joint_positions (ReplicatorItem): Randomizes the joint positions of the articulation prims.
            joint_velocities (ReplicatorItem): Randomizes the joint velocities of the articulation prims.
            lower_dof_limits (ReplicatorItem): Randomizes the lower joint limits of the articulation prims.
            upper_dof_limits (ReplicatorItem): Randomizes the upper joint limits of the articulation prims.
            max_efforts (ReplicatorItem): Randomizes the maximum joint efforts of the articulation prims.
            joint_armatures (ReplicatorItem): Randomizes the joint armatures of the articulation prims.
            joint_max_velocities (ReplicatorItem): Randomizes the maximum joint velocities of the articulation prims.
            joint_efforts (ReplicatorItem): Randomizes the joint efforts of the articulation prims.
            body_masses (ReplicatorItem): Randomizes the mass of each body in the articulation prims. The 
                                          replicator distribution takes in K values, where K is the number of 
                                          bodies in the articulation.
            body_inertias (ReplicatorItem): Randomizes the inertia of each body in the articulation prims. The 
                                            replicator distribution takes in K * 3 values, where K is the number of 
                                            bodies in the articulation.
            material_properties (ReplicatorItem): Randomizes the material properties of the bodies in the articulation.
            tendon_stiffnesses (ReplicatorItem): Randomizes the stiffnesses of the fixed tendons in the articulation.
                                                 The replicator distribution takes in T values, where T is the number
                                                 of tendons in the articulation.
            tendon_dampings (ReplicatorItem): Randomizes the dampings of the fixed tendons in the articulation.
                                              The replicator distribution takes in T values, where T is the number
                                              of tendons in the articulation.
            tendon_limit_stiffnesses (ReplicatorItem): Randomizes the limit stiffnesses of the fixed tendons in 
                                                       the articulation. The replicator distribution takes in T values, 
                                                       where T is the number of tendons in the articulation.
            tendon_lower_limits (ReplicatorItem): Randomizes the lower limits of the fixed tendons in 
                                                  the articulation. The replicator distribution takes in T values, 
                                                  where T is the number of tendons in the articulation.
            tendon_upper_limits (ReplicatorItem): Randomizes the upper limits of the fixed tendons in 
                                                  the articulation. The replicator distribution takes in T values, 
                                                  where T is the number of tendons in the articulation.
            tendon_rest_lengths (ReplicatorItem): Randomizes the rest lengths of the fixed tendons in 
                                                  the articulation. The replicator distribution takes in T values, 
                                                  where T is the number of tendons in the articulation.
            tendon_offsets (ReplicatorItem): Randomizes the offsets of the fixed tendons in 
                                             the articulation. The replicator distribution takes in T values, 
                                             where T is the number of tendons in the articulation.
    """
    # check whether randomization occurs within the correct context
    upstream_node_name = ReplicatorItem.get_context().get_node_type().get_node_type()
    if upstream_node_name != "omni.replicator.isaac.OgnIntervalFiltering":
        raise ValueError(
            "randomize_articulation_view() is expected to be called within the omni.replicator.isaac.randomize.on_interval"
            + " or omni.replicator.isaac.randomize.on_env_reset context managers."
        )

    node_type = "omni.replicator.isaac.OgnWritePhysicsArticulationView"

    if _articulation_views.get(view_name) is None:
        raise ValueError(f"Expected a registered articulation view, but instead received {view_name}")

    tendon_nodes = list()

    if stiffness is not None:
        _write_physics_view_node(view_name, "stiffness", stiffness, operation, node_type)
    if damping is not None:
        _write_physics_view_node(view_name, "damping", damping, operation, node_type)
    if joint_friction is not None:
        _write_physics_view_node(view_name, "joint_friction", joint_friction, operation, node_type)
    if position is not None:
        _write_physics_view_node(view_name, "position", position, operation, node_type)
    if orientation is not None:
        _write_physics_view_node(view_name, "orientation", orientation, operation, node_type)
    if linear_velocity is not None:
        _write_physics_view_node(view_name, "linear_velocity", linear_velocity, operation, node_type)
    if angular_velocity is not None:
        _write_physics_view_node(view_name, "angular_velocity", angular_velocity, operation, node_type)
    if velocity is not None:
        _write_physics_view_node(view_name, "velocity", velocity, operation, node_type)
    if joint_positions is not None:
        _write_physics_view_node(view_name, "joint_positions", joint_positions, operation, node_type)
    if joint_velocities is not None:
        _write_physics_view_node(view_name, "joint_velocities", joint_velocities, operation, node_type)
    if lower_dof_limits is not None:
        _write_physics_view_node(view_name, "lower_dof_limits", lower_dof_limits, operation, node_type)
    if upper_dof_limits is not None:
        _write_physics_view_node(view_name, "upper_dof_limits", upper_dof_limits, operation, node_type)
    if max_efforts is not None:
        _write_physics_view_node(view_name, "max_efforts", max_efforts, operation, node_type)
    if joint_armatures is not None:
        _write_physics_view_node(view_name, "joint_armatures", joint_armatures, operation, node_type)
    if joint_max_velocities is not None:
        _write_physics_view_node(view_name, "joint_max_velocities", joint_max_velocities, operation, node_type)
    if joint_efforts is not None:
        _write_physics_view_node(view_name, "joint_efforts", joint_efforts, operation, node_type)
    if body_masses is not None:
        _write_physics_view_node(view_name, "body_masses", body_masses, operation, node_type)
    if body_inertias is not None:
        _write_physics_view_node(view_name, "body_inertias", body_inertias, operation, node_type)
    if material_properties is not None:
        _write_physics_view_node(
            view_name, "material_properties", material_properties, operation, node_type, num_buckets
        )
    if tendon_stiffnesses is not None:
        tendon_nodes.append(
            _write_physics_view_node(view_name, "tendon_stiffnesses", tendon_stiffnesses, operation, node_type).node
        )
    if tendon_dampings is not None:
        tendon_nodes.append(
            _write_physics_view_node(view_name, "tendon_dampings", tendon_dampings, operation, node_type).node
        )
    if tendon_limit_stiffnesses is not None:
        tendon_nodes.append(
            _write_physics_view_node(
                view_name, "tendon_limit_stiffnesses", tendon_limit_stiffnesses, operation, node_type
            ).node
        )
    if tendon_lower_limits is not None:
        tendon_nodes.append(
            _write_physics_view_node(view_name, "tendon_lower_limits", tendon_lower_limits, operation, node_type).node
        )
    if tendon_upper_limits is not None:
        tendon_nodes.append(
            _write_physics_view_node(view_name, "tendon_upper_limits", tendon_upper_limits, operation, node_type).node
        )
    if tendon_rest_lengths is not None:
        tendon_nodes.append(
            _write_physics_view_node(view_name, "tendon_rest_lengths", tendon_rest_lengths, operation, node_type).node
        )
    if tendon_offsets is not None:
        tendon_nodes.append(
            _write_physics_view_node(view_name, "tendon_offsets", tendon_offsets, operation, node_type).node
        )

    # convert tendon nodes to sequential execution
    if len(tendon_nodes) > 0:
        for node in tendon_nodes:
            upstream_tendon_node = context._context.get_tendon_exec_context()
            context._context.add_tendon_exec_context(node)
            if upstream_tendon_node is not None:
                utils._disconnect(node.get_attribute("inputs:execIn"))
                upstream_tendon_node.get_attribute("outputs:execOut").connect(node.get_attribute("inputs:execIn"), True)


@ReplicatorWrapper
def randomize_simulation_context(operation: str = "direct", gravity: ReplicatorItem = None):
    """ 
        Args:
            operation (str): Can be "direct", "additive", or "scaling".
                             "direct" means random values are directly written into the view.
                             "additive" means random values are added to the default values.
                             "scaling" means random values are multiplied to the default values.
            gravity (ReplicatorItem): Randomizes the gravity vector of the simulation.
    """
    # check whether randomization occurs within the correct context
    upstream_node_name = ReplicatorItem.get_context().get_node_type().get_node_type()
    if upstream_node_name != "omni.replicator.isaac.OgnIntervalFiltering":
        raise ValueError(
            "randomize_simulation_context() is expected to be called within the omni.replicator.isaac.randomize.on_interval"
            + " or omni.replicator.isaac.randomize.on_env_reset context managers."
        )

    node_type = "omni.replicator.isaac.OgnWritePhysicsSimulationContext"

    global _simulation_context
    if _simulation_context is None:
        raise ValueError(f"Expected a registered simulation context")

    if gravity is not None:
        _write_physics_view_node("simulation_context", "gravity", gravity, operation, node_type)
