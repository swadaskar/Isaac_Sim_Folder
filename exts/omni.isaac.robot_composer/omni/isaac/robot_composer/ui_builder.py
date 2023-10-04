# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ui as ui
import omni.timeline

import omni.usd
from pxr import Usd

from omni.isaac.ui.element_wrappers import DropDown, FloatField, CollapsableFrame, Button, Frame, CheckBox, TextBlock
from omni.isaac.core.utils.prims import get_prim_object_type
from omni.isaac.ui.ui_utils import get_style

from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats

import omni.kit.commands

import numpy as np
import carb
from omni.isaac.core.utils.stage import update_stage_async

import asyncio

from typing import List

from .robot_composer import RobotComposer


class UIBuilder:
    def __init__(self):
        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self._robot_composer = RobotComposer()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar. 
        This is distinct from the creation of the UI in build_ui()
        because it can happen more than once if the user repeatedly
        closes and reopens the window.

        This callback happens after build_ui() when the extension is first opened
        """
        # Handles the edge case where the user loads their Articulation and
        # presses play before opening this extension
        if self._timeline.is_playing():
            self._repopulate_all_dropdowns()
            self.composition_frame.enabled = True
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """

        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            # Populate Articulation selection menu when the user presses PLAY
            self._repopulate_all_dropdowns()
            self.composition_frame.enabled = True
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.composition_frame.enabled = False
            self.composition_frame.collapsed = True
            self._repopulate_all_dropdowns()

    def on_physics_step(self, step):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing
           
        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        # The user adding or removing an Articulation from the stage would be a stage event
        # print("stage event")
        self._repopulate_all_dropdowns()
        pass

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    ######################################################################################################
    #                                           Build UI
    ######################################################################################################

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.  
        This function will be called once when your extension is opened.  
        Closing and reopening the extension from the toolbar will maintain the state of the UI.
        If the user hot reloads this extension, this function will be called again.
        """
        names = ["Base Robot", "Attach Robot"]
        self._robot_frames = []
        self._robot_control_frames = []
        self._robot_dropdowns = []
        self._articulation_attach_point_dropdowns = []
        self._articulations = [None] * len(names)
        self._collapsable_robot_control_frames = [None] * len(names)

        self.wrapped_ui_elements = []

        self._joint_control_frames = []
        self._joint_position_float_fields = []

        self._articulations_nested = False
        self._composed_robot = None

        for idx in range(len(names)):
            robot_frame = CollapsableFrame(names[idx], collapsed=False)
            self._robot_frames.append(robot_frame)
            with robot_frame:
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    selection_menu = DropDown(
                        "Select Articulation",
                        tooltip="Select from Articulations found on the stage after the timeline has been played.",
                        on_selection_fn=lambda selection, ind=idx: self._on_articulation_selection(ind, selection),
                        keep_old_selections=True,
                        populate_fn=lambda ind=idx: self._dropdown_populate_fn(ind),
                    )
                    self._robot_dropdowns.append(selection_menu)
                    self.wrapped_ui_elements.append(selection_menu)

                    robot_control_frame = Frame(build_fn=lambda idx=idx: self._build_set_robot_position_frame(idx))
                    robot_control_frame.rebuild()
                    self._robot_control_frames.append(robot_control_frame)

        self._make_compose_frame(names)
        self._make_composition_summary_frame()

    def _make_composition_summary_frame(self):
        self.composition_summary_frame = CollapsableFrame("Composition Summary Frame", visible=False)

        with self.composition_summary_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.undo_compose_btn = Button(
                    "Undo Composition",
                    "UNDO COMPOSE",
                    tooltip="Undo the last robot composition",
                    on_click_fn=self._undo_last_compose,
                )

                self.composition_code_summary = TextBlock(
                    "Python Code",
                    "",
                    tooltip="A Code Block that replicates the result produced by the user in this UI tool",
                    num_lines=20,
                )

    def _make_compose_frame(self, names):
        self.composition_frame = CollapsableFrame("Composition Frame", collapsed=True, enabled=False)
        with self.composition_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                # Select Robot Attach Points
                for idx in range(len(names)):
                    self._articulation_attach_point_dropdowns.append(
                        DropDown(
                            f"{names[idx]} Attach Point",
                            tooltip="Select attach point for Articulation",
                            keep_old_selections=False,
                            populate_fn=lambda ind=idx: self._attach_point_populate_fn(ind),
                        )
                    )

                def on_begin_compose_btn_clicked():
                    self._nest_articulations()

                    self._composer_frame.collapsed = False
                    self._composer_frame.visible = True

                    self._begin_compose_btn.enabled = False

                    for robot_frame in self._robot_frames:
                        robot_frame.visible = False

                    self.composition_frame.visible = False

                def on_stop_compose_btn_clicked():
                    async def async_cancel():
                        await self._undo_nest_articulations()
                        await self._put_attached_art_back()

                    asyncio.ensure_future(async_cancel())

                    self._begin_compose_btn.enabled = True
                    self._composer_frame.visible = False

                    for robot_frame in self._robot_frames:
                        robot_frame.visible = True

                    self.composition_frame.visible = True

                def on_compose_btn_clicked(make_single_robot):
                    trans, rot = self._get_relative_attach_transform()

                    async def async_compose():
                        await self._undo_nest_articulations()
                        self._compose_robots(trans, rot, make_single_robot)

                    asyncio.ensure_future(async_compose())

                    self._composer_frame.collapsed = True
                    self._composer_frame.visible = False
                    self._begin_compose_btn.enabled = True

                    for robot_frame in self._robot_frames:
                        robot_frame.visible = True

                    self.composition_frame.visible = True

                self._begin_compose_btn = Button(
                    "Begin Composing Robots",
                    "BEGIN COMPOSE",
                    "Press this to begin composing the selected robots",
                    on_begin_compose_btn_clicked,
                )

        self._composer_frame = CollapsableFrame("Robot Composer", collapsed=True, visible=False)
        with self._composer_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                stop_compose_btn = Button(
                    "Cancel Compose",
                    "CANCEL COMPOSE",
                    "Press this to cancel the composition.",
                    on_stop_compose_btn_clicked,
                )
                select_attach_point_btn = Button(
                    "Select Attach Point Prim",
                    "SELECT ATTACH POINT",
                    "Select the attach point frame of the attach robot in order to specify the relative pose.",
                    self._select_attach_point_prim,
                )
                single_robot_cb = CheckBox(
                    "Compose Into Single Robot",
                    default_value=False,
                    tooltip="When checked, the attached robots will be treated as a single Articulation that can be accessed at the prim path of the base robot."
                    + "  When not checked, the attached robots will still be treated as separate Articulations that are controlled independently.",
                )

                compose_button = Button(
                    "Compose Robots",
                    "COMPOSE",
                    "Compose the selected robots",
                    lambda: on_compose_btn_clicked(single_robot_cb.get_value()),
                )

    def _build_set_robot_position_frame(self, idx):
        if self._collapsable_robot_control_frames[idx] is None:
            collapsed = True
        else:
            collapsed = self._collapsable_robot_control_frames[idx].collapsed

        if self._articulations[idx] is None:
            Frame()
            self._collapsable_robot_control_frames[idx] = None
            return
        articulation = self._articulations[idx]
        num_dof = articulation.num_dof
        dof_names = articulation.dof_names
        joint_positions = articulation.get_joint_positions()

        lower_joint_limits = articulation.dof_properties["lower"]
        upper_joint_limits = articulation.dof_properties["upper"]

        robot_control_frame = CollapsableFrame("Set Robot Position", collapsed=collapsed)
        self._collapsable_robot_control_frames[idx] = robot_control_frame

        with robot_control_frame:
            # Stack the frames vertically so that they don't cover each other
            with ui.VStack(style=get_style(), spacing=6, height=0):

                for i in range(num_dof):
                    field = FloatField(
                        label=f"{dof_names[i]}",
                        tooltip="Set joint position target",
                        default_value=joint_positions[i],
                        lower_limit=lower_joint_limits[i],
                        upper_limit=upper_joint_limits[i],
                    )
                    field.set_on_value_changed_fn(
                        lambda value, index=i, robot_index=idx: self._on_set_joint_position_target(
                            robot_index, index, value
                        )
                    )

    ##########################################################################################
    #                               Composer Summary Frame Functions
    ##########################################################################################

    def setup_composer_summary_frame(self, art_1_path, art_2_path, sel_1, sel_2, rel_trans, rel_orient, single_robot):
        self.undo_compose_btn.enabled = True
        self.composition_summary_frame.visible = True
        self.composition_summary_frame.collapsed = False

        def list_print(l):
            s = "np.array(["
            for i in range(len(l) - 1):
                s += str(round(l[i], 4)) + ","
            s += str(round(l[-1], 4)) + "])"
            return s

        self.composition_code_summary.set_text(
            "from omni.isaac.robot_composer import RobotComposer,ComposedRobot \n"
            + "from omni.isaac.core.articulations import Articulation\n"
            + "import numpy as np\n\n"
            + f'base_robot_path = "{art_1_path}"\n'
            + f'attach_robot_path = "{art_2_path}"\n'
            + f'base_robot_mount_frame = "{sel_1}"\n'
            + f'attach_robot_mount_frame = "{sel_2}"\n'
            + f"fixed_joint_offset = {list_print(rel_trans)}\n"
            + f"fixed_joint_orient = {list_print(rel_orient)}\n"
            + f"single_robot = {single_robot}\n\n"
            + "robot_composer = RobotComposer()\n"
            + "composed_robot = robot_composer.compose(base_robot_path, attach_robot_path, base_robot_mount_frame, attach_robot_mount_frame, fixed_joint_offset, fixed_joint_orient, single_robot=single_robot, mask_all_collisions = True)\n\n"
            + "# The fixed joint in a composed robot can is editable after the fact:\n"
            + "# offset,orient = composed_robot.get_fixed_joint_transform()\n"
            + "# composed_robot.set_fixed_joint_transform(np.array([.3,0,0]),np.array([1,0,0,0]))\n\n"
            + "# And the composed robot can be decomposed, after which point the ComposedRobot object will no longer function.\n"
            + "# composed_robot.decompose()\n\n"
            + "# Controlling the resulting composed robot is different depending on the single_robot flag\n"
            + "if single_robot:\n"
            + "\t# The robots will be considered to be part of a single Articulation at the base robot path\n"
            + "\tcontrollable_single_robot = Articulation(base_robot_path)\n"
            + "else:\n"
            + "\t# The robots are controlled independently from each other\n"
            + "\tbase_robot = Articulation(base_robot_path)\n"
            + "\tattach_robot = Articulation(attach_robot_path)\n"
        )

    def _undo_last_compose(self):
        if self._composed_robot is not None and self._composed_robot.is_composed():
            self._composed_robot.decompose()
            self.undo_compose_btn.enabled = False
            self._repopulate_all_dropdowns()

    ##########################################################################################
    #                              Robot Composer Frame Functions
    ##########################################################################################

    def _nest_articulations(self):
        sel_1 = self._articulation_attach_point_dropdowns[0].get_selection()
        sel_2 = self._articulation_attach_point_dropdowns[1].get_selection()

        art_1 = self._articulations[0]
        art_2 = self._articulations[1]

        self._art_1_path = art_1.prim_path
        self._art_2_path = art_2.prim_path

        self._base_attach_frame = sel_1
        self._attached_art_attach_frame = sel_2

        if art_1 is None or art_2 is None:
            carb.log_error("Begin Compose Button was Clicked before valid articulations were selected")

        self._articulations_nested = True

        self._attached_art_default_pose = XFormPrim(art_2.prim_path).get_world_pose()

        base_path = art_1.prim_path + sel_1
        art_name = art_2.prim_path[art_2.prim_path.rfind("/") :]

        self._nested_articulation_path_from = art_2.prim_path
        self._nested_articulation_path_to = base_path + art_name

        omni.kit.commands.execute("MovePrimCommand", path_from=art_2.prim_path, path_to=base_path + art_name)

        nested_art_xform = XFormPrim(self._nested_articulation_path_to)
        nested_art_xform.set_local_pose(np.zeros(3), np.array([1, 0, 0, 0]))

        self._timeline.stop()

    def _select_attach_point_prim(self):
        omni.kit.commands.execute(
            "SelectPrimsCommand",
            old_selected_paths=[],
            new_selected_paths=[self._nested_articulation_path_to],
            expand_in_stage=False,
        )

    def _get_relative_attach_transform(self):
        sel_1 = self._base_attach_frame
        art_1_path = self._art_1_path

        base_attach_point_xform = XFormPrim(art_1_path + sel_1)

        attach_frame = self._articulation_attach_point_dropdowns[1].get_selection()
        nested_attach_frame_xform = XFormPrim(self._nested_articulation_path_to + attach_frame)

        nested_art_trans, nested_art_rot = nested_attach_frame_xform.get_world_pose()
        base_trans, base_rot = base_attach_point_xform.get_world_pose()
        base_rot, nested_art_rot = quats_to_rot_matrices(np.array([base_rot, nested_art_rot]))

        local_rot = base_rot.T @ nested_art_rot
        local_trans = (base_rot.T @ (nested_art_trans - base_trans)).reshape((3,))

        local_rot = rot_matrices_to_quats(local_rot)

        return local_trans, local_rot

    async def _undo_nest_articulations(self):
        if not self._articulations_nested:
            return

        omni.kit.commands.execute(
            "MovePrimCommand", path_from=self._nested_articulation_path_to, path_to=self._nested_articulation_path_from
        )

        self._timeline.play()

        await update_stage_async()

        self._articulations_nested = False

    async def _put_attached_art_back(self):
        self._repopulate_all_dropdowns()

        self._robot_dropdowns[0].set_selection(self._art_1_path)

        self._robot_dropdowns[1].set_selection(self._art_2_path)
        self._on_articulation_selection(1, self._art_2_path)

        attach_art = self._articulations[1]
        XFormPrim(attach_art.prim_path).set_world_pose(*self._attached_art_default_pose)

    def _compose_robots(self, rel_trans, rel_orient, single_robot):
        sel_1 = self._base_attach_frame
        sel_2 = self._attached_art_attach_frame

        art_1_path = self._art_1_path
        art_2_path = self._art_2_path

        self._composed_robot = self._robot_composer.compose(
            art_1_path,
            art_2_path,
            sel_1,
            sel_2,
            fixed_joint_offset=rel_trans,
            fixed_joint_orient=rel_orient,
            single_robot=single_robot,
        )
        self.setup_composer_summary_frame(art_1_path, art_2_path, sel_1, sel_2, rel_trans, rel_orient, single_robot)

    ############################################################################################
    #                               Composition Frame Functions
    ############################################################################################

    def _attach_point_populate_fn(self, art_ind: int) -> List[str]:
        articulation = self._articulations[art_ind]
        if articulation is None:
            return []
        attach_points = self._get_attach_points(articulation)
        if art_ind == 0:
            attach_points.reverse()
        return attach_points

    def _get_attach_points(self, articulation):
        stage = omni.usd.get_context().get_stage()
        art_path = articulation.prim_path
        paths = []
        if stage and articulation is not None:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(articulation.prim_path)):
                path = str(prim.GetPath())
                obj_type = get_prim_object_type(path)
                sub_path = path[len(art_path) :]

                if sub_path.count("/") == 1 and (obj_type == "rigid_body" or obj_type == "xform"):
                    paths.append(sub_path)
        return paths

    ##########################################################################################
    #                            Robot Selection Frame Functions
    ##########################################################################################

    def _invalidate_articulation(self, art_ind):
        """
        This function handles the event that the existing articulation becomes invalid and there is
        not a new articulation to select.  It is called explicitly in the code when the timeline is
        stopped and when the DropDown menu finds no articulations on the stage.
        """
        self._articulations[art_ind] = None
        self._robot_control_frames[art_ind].rebuild()

    def _on_articulation_selection(self, art_ind: int, selection: str):
        if selection is None or self._timeline.is_stopped():
            self._invalidate_articulation(art_ind)
            return

        articulation = Articulation(selection)
        articulation.initialize()

        self._articulations[art_ind] = articulation

        self._robot_control_frames[art_ind].rebuild()

        self._repopulate_all_dropdowns()

        self._articulation_attach_point_dropdowns[art_ind].repopulate()

    def _on_set_joint_position_target(self, robot_index: int, joint_index: int, position_target: float):
        articulation = self._articulations[robot_index]
        robot_action = ArticulationAction(
            joint_positions=np.array([position_target]),
            joint_velocities=np.array([0]),
            joint_indices=np.array([joint_index]),
        )
        articulation.apply_action(robot_action)

    def _repopulate_all_dropdowns(self):
        for d in self._robot_dropdowns:
            d.repopulate()

        # Repopulating articulation menus will recursively repopulate articulation_attach_point dropdowns

    def _dropdown_populate_fn(self, ind: int) -> List[str]:
        # Pick an articulation from the stage that has not been selected already
        selections = [d.get_selection() for d in self._robot_dropdowns[:ind]]
        articulations = self._find_all_articulations()

        for selection in selections:
            if selection in articulations:
                articulations.remove(selection)

        if len(articulations) == 0:
            self.composition_frame.enabled = False
        else:
            self.composition_frame.enabled = True
        return articulations

    def _find_all_articulations(self):
        items = []
        stage = omni.usd.get_context().get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                # Get prim type get_prim_object_type
                type = get_prim_object_type(path)
                if type == "articulation":
                    items.append(path)
        return items
