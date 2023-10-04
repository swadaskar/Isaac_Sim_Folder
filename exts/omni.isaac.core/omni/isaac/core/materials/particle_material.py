# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


from typing import Optional

# omniverse
import omni, carb
from pxr import PhysxSchema, Usd, UsdShade
from omni.isaac.core.simulation_context.simulation_context import SimulationContext

# isaac-core
import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.materials.particle_material_view import ParticleMaterialView


class ParticleMaterial:
    """A wrapper around position-based-dynamics (PBD) material for particles used to
    simulate fluids, cloth and inflatables.

    Note:
        Currently, only a single material per particle system is supported which applies
        to all objects that are associated with the system.
    """

    def __init__(
        self,
        prim_path: str,
        name: Optional[str] = "particle_material",
        friction: Optional[float] = None,
        particle_friction_scale: Optional[float] = None,
        damping: Optional[float] = None,
        viscosity: Optional[float] = None,
        vorticity_confinement: Optional[float] = None,
        surface_tension: Optional[float] = None,
        cohesion: Optional[float] = None,
        adhesion: Optional[float] = None,
        particle_adhesion_scale: Optional[float] = None,
        adhesion_offset_scale: Optional[float] = None,
        gravity_scale: Optional[float] = None,
        lift: Optional[float] = None,
        drag: Optional[float] = None,
    ):
        """Applies the `PhysxSchema.PhysxPBDMaterialAPI` to a material prim.
        Note:
            If a prim does not exist at specified path, then a new UsdShade.Material prim is created.

        Args:
            prim_path (str): The prim path to create/apply PBD material properties.
            friction (float, optional): The friction coefficient.
            particle_friction_scale (float, optional): The coefficient that scales friction for
                solid particle-particle interactions.
            damping (float, optional): The global velocity damping coefficient
            viscosity (float, optional): The viscosity of fluid particles.
            vorticity_confinement (float, optional): The vorticity confinement for fluid particles.
            surface_tension (float, optional): The surface tension.
            cohesion (float, optional): The cohesion for interaction between fluid particles.
            adhesion (float, optional): The adhesion for interaction between particles (solid or fluid),
                and rigid or deformable objects.
            particle_adhesion_scale (float, optional): The coefficient that scales adhesion for solid
                particle-particle interations.
            adhesion_offset_scale (float, optional): The offset scale defines at which adhesion ceases
                to take effect.
            gravity_scale (float, optional): The gravitational acceleration scaling factor. It can be used
                to approximate lighter-than-air inflatables.
            lift (float, optional): The lift coefficient for cloth and inflatable particle objects.
            drag (float, optional): The drag coefficient for cloth and inflatable particle objects.
        """
        stage = omni.usd.get_context().get_stage()
        self._name = name
        self._prim_path = prim_path
        self._prim = stage.GetPrimAtPath(prim_path)

        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._device = SimulationContext.instance().device
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

        if stage.GetPrimAtPath(prim_path).IsValid():
            if not self._prim.IsA(UsdShade.Material):
                raise ValueError(f"A prim at path '{prim_path}' exists but is not a Usd.Material prim.")
            else:
                carb.log_warn(f"A material prim already defined at path: {prim_path}.")
                self._material = UsdShade.Material(stage.GetPrimAtPath(prim_path))
        else:
            self._material = UsdShade.Material.Define(stage, prim_path)

        # set properties
        if friction is not None:
            friction = self._backend_utils.create_tensor_from_list([friction], dtype="float32", device=self._device)
        if particle_friction_scale is not None:
            particle_friction_scale = self._backend_utils.create_tensor_from_list(
                [particle_friction_scale], dtype="float32", device=self._device
            )
        if damping is not None:
            damping = self._backend_utils.create_tensor_from_list([damping], dtype="float32", device=self._device)
        if viscosity is not None:
            viscosity = self._backend_utils.create_tensor_from_list([viscosity], dtype="float32", device=self._device)
        if vorticity_confinement is not None:
            vorticity_confinement = self._backend_utils.create_tensor_from_list(
                [vorticity_confinement], dtype="float32", device=self._device
            )
        if surface_tension is not None:
            surface_tension = self._backend_utils.create_tensor_from_list(
                [surface_tension], dtype="float32", device=self._device
            )
        if cohesion is not None:
            cohesion = self._backend_utils.create_tensor_from_list([cohesion], dtype="float32", device=self._device)
        if adhesion is not None:
            adhesion = self._backend_utils.create_tensor_from_list([adhesion], dtype="float32", device=self._device)
        if particle_adhesion_scale is not None:
            particle_adhesion_scale = self._backend_utils.create_tensor_from_list(
                [particle_adhesion_scale], dtype="float32", device=self._device
            )
        if adhesion_offset_scale is not None:
            adhesion_offset_scale = self._backend_utils.create_tensor_from_list(
                [adhesion_offset_scale], dtype="float32", device=self._device
            )
        if gravity_scale is not None:
            gravity_scale = self._backend_utils.create_tensor_from_list(
                [gravity_scale], dtype="float32", device=self._device
            )
        if lift is not None:
            lift = self._backend_utils.create_tensor_from_list([lift], dtype="float32", device=self._device)
        if drag is not None:
            drag = self._backend_utils.create_tensor_from_list([drag], dtype="float32", device=self._device)

        self._particle_material_view = ParticleMaterialView(
            prim_paths_expr=prim_path,
            name=name,
            frictions=friction,
            particle_friction_scales=particle_friction_scale,
            dampings=damping,
            viscosities=viscosity,
            vorticity_confinements=vorticity_confinement,
            surface_tensions=surface_tension,
            cohesions=cohesion,
            adhesions=adhesion,
            particle_adhesion_scales=particle_adhesion_scale,
            adhesion_offset_scales=adhesion_offset_scale,
            gravity_scales=gravity_scale,
            lifts=lift,
            drags=drag,
        )

    """
    Properties.
    """

    @property
    def prim_path(self) -> str:
        """
        Returns:
            str: The stage path to the material.
        """
        return self._prim_path

    @property
    def prim(self) -> Usd.Prim:
        """
        Returns:
            Usd.Prim: The USD prim present.
        """
        return self._prim

    @property
    def material(self) -> UsdShade.Material:
        """
        Returns:
            UsdShade.Material: The USD Material object.
        """
        return self._material

    @property
    def name(self) -> Optional[str]:
        """
        Returns:
            str: name given to the prim when instantiating it. Otherwise None.
        """
        return self._name

    def initialize(self, physics_sim_view=None) -> None:
        self._particle_material_view.initialize(physics_sim_view=physics_sim_view)
        return

    def is_valid(self) -> bool:
        """
        Returns:
            bool: True is the current prim path corresponds to a valid prim in stage. False otherwise.
        """
        return self._particle_material_view.is_valid()

    def post_reset(self) -> None:
        """Resets the prim to its default state.
        """
        self._particle_material_view.post_reset()
        return

    """
    Operations - Setters.
    """

    def set_friction(self, value: float) -> None:
        """Sets the friction coefficient.

        The friction takes effect in all interactions between particles and rigids or deformables.
        For solid particle-particle interactions it is multiplied by the particle friction scale.

        Args:
            value (float): The friction coefficient.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of friction coefficient is [0. inf).")
        self._particle_material_view.set_frictions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_particle_friction_scale(self, value: float) -> None:
        """Sets the particle friction scale.

        The coefficient that scales friction for solid particle-particle interaction.

        Args:
            value (float): The particle friction scale.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of particle friction scale is [0. inf).")
        self._particle_material_view.set_particle_friction_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_damping(self, value: float) -> None:
        """Sets the global velocity damping coefficient.

        Args:
            value (float): The damping coefficient.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of damping coefficient is [0. inf).")
        self._particle_material_view.set_dampings(self._backend_utils.create_tensor_from_list([value], dtype="float32"))

    def set_viscosity(self, value: float) -> None:
        """Sets the viscosity for fluid particles.

        Args:
            value (float): The viscosity.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of viscosity is [0. inf).")
        self._particle_material_view.set_viscosities(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_vorticity_confinement(self, value: float) -> None:
        """Sets the vorticity confinement for fluid particles.

        This helps prevent energy loss due to numerical solver by adding vortex-like
        accelerations to the particles.

        Args:
            value (float): The vorticity confinement.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of vorticity confinement is [0. inf).")
        self._particle_material_view.set_vorticity_confinements(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_surface_tension(self, value: float) -> None:
        """Sets the surface tension for fluid particles.

        Args:
            value (float): The surface tension.
                Range: [0, inf), Units: 1 / (distance * distance * distance)
        """
        if value < 0:
            carb.log_error("The valid range of damping coefficient is [0. inf).")
        self._particle_material_view.set_surface_tensions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_cohesion(self, value: float) -> None:
        """Sets the cohesion for interaction between fluid particles.

        Args:
            value (float): The cohesion.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of adhesion is [0. inf).")
        self._particle_material_view.set_cohesions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_adhesion(self, value: float) -> None:
        """Sets the adhesion for interaction between particles (solid or fluid), and rigid or deformable objects.

        Note:
            Adhesion also applies to solid-solid particle interactions, but is multiplied with the
            particle adhesion scale.

        Args:
            value (float): The adhesion.
                Range: [0, inf), Units: dimensionless

        """
        if value < 0:
            carb.log_error("The valid range of adhesion is [0. inf).")
        self._particle_material_view.set_adhesions(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_particle_adhesion_scale(self, value: float) -> None:
        """Sets the particle adhesion scale.

        This coefficient scales the adhesion for solid particle-particle interaction.

        Args:
            value (float): The adhesion scale.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of particle adhesion scale is [0. inf).")
        self._particle_material_view.set_particle_adhesion_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_adhesion_offset_scale(self, value: float) -> None:
        """Sets the adhesion offset scale.

        It defines the offset at which adhesion ceases to take effect. For interactions between
        particles (fluid or solid), and rigids or deformables, the adhesion offset is defined
        relative to the rest offset. For solid particle-particle interactions, the adhesion
        offset is defined relative to the solid rest offset.

        Args:
            value (float): The adhesion offset scale.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of adhesion offset scale is [0. inf).")
        self._particle_material_view.set_adhesion_offset_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_gravity_scale(self, value: float) -> None:
        """Sets the gravitational acceleration scaling factor.

        It can be used to approximate lighter-than-air inflatable.
        For example (-1.0 would invert gravity).

        Args:
            value (float): The gravity scale.
                Range: (-inf , inf), Units: dimensionless
        """
        self._particle_material_view.set_gravity_scales(
            self._backend_utils.create_tensor_from_list([value], dtype="float32")
        )

    def set_lift(self, value: float) -> None:
        """Sets the lift coefficient, i.e. basic aerodynamic lift model coefficient.

        It is useful for cloth and inflatable particle objects.

        Args:
            value (float): The lift coefficient.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of lift coefficient is [0. inf).")
        self._particle_material_view.set_lifts(self._backend_utils.create_tensor_from_list([value], dtype="float32"))

    def set_drag(self, value: float) -> None:
        """Sets the drag coefficient, i.e. basic aerodynamic drag model coefficient.

        It is useful for cloth and inflatable particle objects.

        Args:
            value (float): The drag coefficient.
                Range: [0, inf), Units: dimensionless
        """
        if value < 0:
            carb.log_error("The valid range of drag coefficient is [0. inf).")
        self._particle_material_view.set_drags(self._backend_utils.create_tensor_from_list([value], dtype="float32"))

    """
    Operations - Getters.
    """

    def get_friction(self) -> float:
        """
        Returns:
            float: The friction coefficient.
        """
        return self._particle_material_view.get_frictions()[0]

    def get_particle_friction_scale(self) -> float:
        """
        Returns:
            float: The particle friction scale.
        """
        return self._particle_material_view.get_particle_friction_scales()[0]

    def get_damping(self) -> float:
        """
        Returns:
            float: The global velocity damping coefficient.
        """
        return self._particle_material_view.get_dampings()[0]

    def get_viscosity(self) -> float:
        """
        Returns:
            float: The viscosity.
        """
        return self._particle_material_view.get_viscosities()[0]

    def get_vorticity_confinement(self) -> float:
        """
        Returns:
            float: The vorticity confinement for fluid particles.
        """
        return self._particle_material_view.get_vorticity_confinements()[0]

    def get_surface_tension(self) -> float:
        """
        Returns:
            float: The surface tension for fluid particles.
        """
        return self._particle_material_view.get_surface_tensions()[0]

    def get_cohesion(self) -> float:
        """
        Returns:
            float: The cohesion for interaction between fluid particles.
        """
        return self._particle_material_view.get_cohesions()[0]

    def get_adhesion(self) -> float:
        """
        Returns:
            float: The adhesion for interaction between particles (solid or fluid), and rigids or deformables.
        """
        return self._particle_material_view.get_adhesions()[0]

    def get_particle_adhesion_scale(self) -> float:
        """
        Returns:
            float: The particle adhesion scale.
        """
        return self._particle_material_view.get_particle_adhesion_scales()[0]

    def get_adhesion_offset_scale(self) -> float:
        """
        Returns:
            float: The adhesion offset scale.
        """
        return self._particle_material_view.get_adhesion_offset_scales()[0]

    def get_gravity_scale(self) -> float:
        """
        Returns:
            float: The gravitational acceleration scaling factor.
        """
        return self._particle_material_view.get_gravity_scales()[0]

    def get_lift(self) -> float:
        """
        Returns:
            float: The lift coefficient, basic aerodynamic lift model coefficient.
        """
        return self._particle_material_view.get_lifts()[0]

    def get_drag(self) -> float:
        """
        Returns:
            float: The drag coefficient, basic aerodynamic drag model coefficient.
        """
        return self._particle_material_view.get_drags()[0]
