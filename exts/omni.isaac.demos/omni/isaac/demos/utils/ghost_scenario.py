# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import random
import time
import os
import sys
from pxr import Sdf, Gf, UsdPhysics
import concurrent.futures
from omni.isaac.demos.utils.franka import Franka, default_config, alternate_config
from omni.isaac.demos.utils.world import World
from omni.isaac.demos.utils.state_machine import *
from omni.isaac.demos.utils.behavior_states import *
from omni.isaac.demos.utils.behavior_helpers import *
from .scenario import (
    Scenario,
    create_rubiks_cube,
    create_solid_franka,
    create_blocks,
    create_ghost_franka,
    create_background,
    setup_physics,
)


class GhostScenario(Scenario):
    """
    This scenario stacks 4 blocks on top of each other
    Ghost robots are used to visualize alternative trajectories that could have been used to pick up a block.
    """

    def __init__(self, dc, mp):
        super().__init__(dc, mp)
        self.on_stop = 0
        self.on_start = 0
        self.reset_time = 100  # reset the scenario every 100 seconds
        self.num_ghosts = 2  # two ghost robots are spawned alongisde the solid robot

    def reset_blocks(self, *args):
        if self._timeline.is_playing():
            for domain in self._domains:
                xnum = [0.70, 0.40]
                ynum = [0.20, -0.20]
                random.shuffle(xnum)
                random.shuffle(ynum)
                domain.block_locations.reset(
                    [
                        ("00_block_red", (xnum[1], ynum[1], 12)),
                        ("00_block_yellow", (xnum[1], ynum[0], 12)),
                        ("00_block_green", (xnum[0], ynum[1], 12)),
                        ("00_block_blue", (xnum[0], ynum[0], 12)),
                    ]
                )

    def stop_tasks(self, *args):
        super().stop_tasks()
        for domain in self._domains:
            domain.stop = True
            for ghost_domain in domain.ghost_domains:
                ghost_domain.stop = True
                ghost_domain.franka.target_visibility = False
        self.reset_blocks()
        self.on_stop = self.on_start = 0

    def step(self, step):
        if self._timeline.is_playing():
            for domain in self._domains:
                for ghost_domain in domain.ghost_domains:
                    ghost_domain.block_locations.update()
                    ghost_domain.franka.update()
                domain.block_locations.update()
                domain.franka.update()
                domain.tick(step)

            if self.on_stop > 0 and time.time() > self.on_stop:
                self.stop_tasks()
                self.on_stop = 0
                self.on_start = time.time() + 4
            if self.on_start > 0 and time.time() > self.on_start:
                self.on_start = 0
                self.perform_tasks()
        else:
            self.stop_tasks()
            self.on_stop = self.on_start = 0

    def create_franka(self, *args):
        super().create_franka()
        index = 0
        for i in range(0, 1):
            for j in range(0, 1):
                env_path = "/environments/env_{}_{}".format(i, j)
                if not self._stage.GetPrimAtPath(env_path):
                    create_rubiks_cube(
                        self._stage, self.rubiks_cube_usd, env_path + "/Rubiks_cube", Gf.Vec3d(-0.10, -0.30, 0.12)
                    )
                    create_solid_franka(self._stage, env_path, self.franka_table_usd, Gf.Vec3d(-i * 2.00, j * 2.00, 0))
                    create_blocks(
                        self._stage,
                        [self.red_cube_usd, self.yellow_cube_usd, self.green_cube_usd, self.blue_cube_usd],
                        [
                            env_path + "/Blocks/block_01",
                            env_path + "/Blocks/block_02",
                            env_path + "/Blocks/block_03",
                            env_path + "/Blocks/block_04",
                        ],
                        [
                            Gf.Vec3d(0.40, 0.15, 0.12),
                            Gf.Vec3d(0.40, -0.15, 0.12),
                            Gf.Vec3d(0.60, 0.15, 0.12),
                            Gf.Vec3d(0.60, -0.15, 0.12),
                        ],
                    )
                    for ghost_index in range(0, self.num_ghosts):
                        create_ghost_franka(self._stage, env_path, self.franka_ghost_usd, ghost_index)

                index = index + 1

        create_background(self._stage, self.background_usd)
        setup_physics(self._stage)

    def register_assets(self, *args):
        self._domains = []
        self._obstacles = []
        block_colors = ["blue", "yellow", "green", "red"]
        index = 0

        for prim in self._stage.GetPrimAtPath("/environments").GetChildren():
            yellow_path = str(prim.GetPath()) + "/Blocks/block_01/Cube"
            red_path = str(prim.GetPath()) + "/Blocks/block_02/Cube"
            green_path = str(prim.GetPath()) + "/Blocks/block_03/Cube"
            blue_path = str(prim.GetPath()) + "/Blocks/block_04/Cube"

            obstacle_path = str(prim.GetPath()) + "/Rubiks_cube"

            handle_1 = self._dc.get_rigid_body(yellow_path)
            handle_2 = self._dc.get_rigid_body(red_path)
            handle_3 = self._dc.get_rigid_body(green_path)
            handle_4 = self._dc.get_rigid_body(blue_path)

            world = World(self._dc, self._mp)
            franka_solid = Franka(
                self._stage,
                self._stage.GetPrimAtPath(str(prim.GetPath()) + "/Franka/panda"),
                self._dc,
                self._mp,
                world,
                default_config,
            )
            franka_ghosts = []
            ghost_domains = []
            for i in range(0, self.num_ghosts):
                ghost_world = World(self._dc, self._mp)

                ghost = Franka(
                    self._stage,
                    self._stage.GetPrimAtPath(str(prim.GetPath()) + "/Ghost/robot_{}/Franka/panda".format(i)),
                    self._dc,
                    self._mp,
                    ghost_world,
                    alternate_config[i % 2],
                    True,  # this is a ghost
                )

                ghost_world.register_object(handle_1, yellow_path, "00_block_yellow")
                ghost_world.register_object(handle_2, red_path, "00_block_red")
                ghost_world.register_object(handle_3, green_path, "00_block_green")
                ghost_world.register_object(handle_4, blue_path, "00_block_blue")
                ghost_world.register_object(0, str(prim.GetPath()) + "/DemoTable/simple_table/CollisionCube", "table")
                ghost_world.register_object(0, obstacle_path, "rubiks_cube")
                ghost_world.make_obstacle("rubiks_cube", 3, (0.072, 0.072, 0.072))

                franka_ghosts.append(ghost)
                ghost_domain = Domain(
                    ghost, None, BlocksWorldSuppressors(ghost, ghost_world, block_colors), block_colors, ghost_world, 30
                )
                ghost_domains.append(ghost_domain)

            world.register_object(handle_1, yellow_path, "00_block_yellow")
            world.register_object(handle_2, red_path, "00_block_red")
            world.register_object(handle_3, green_path, "00_block_green")
            world.register_object(handle_4, blue_path, "00_block_blue")
            world.register_object(0, str(prim.GetPath()) + "/DemoTable/simple_table/CollisionCube", "table")
            world.register_object(0, obstacle_path, "rubiks_cube")

            self._obstacles.append(world.get_object_from_name("rubiks_cube"))

            world.make_obstacle("00_block_yellow", 3, (0.05, 0.05, 0.05))
            world.make_obstacle("00_block_red", 3, (0.05, 0.05, 0.05))
            world.make_obstacle("00_block_green", 3, (0.05, 0.05, 0.05))
            world.make_obstacle("00_block_blue", 3, (0.05, 0.05, 0.05))
            world.make_obstacle("rubiks_cube", 3, (0.072, 0.072, 0.072))

            blocks_world_suppressors = BlocksWorldSuppressors(franka_solid, world, block_colors)

            domain = Domain(franka_solid, ghost_domains, blocks_world_suppressors, block_colors, world, 30)

            self._domains.append(domain)
            index = index + 1

        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=len(self._domains))

    def task(self, domain):
        try:
            start_state = latest_state = Behavior()
            resolve_violations_behavior = Behavior(ResolveViolations(domain))
            latest_state.terminal_transition = NextStateTransition(resolve_violations_behavior)
            latest_state = resolve_violations_behavior

            stack_blocks = StackBlocksEconomical(domain)
            latest_state.terminal_transition = NextStateTransition(stack_blocks)

            # Run the state machine.
            run_state_machine(start_state, domain.step_rate, [domain])
        except Exception as exc:
            print("generated an exception: %s" % (exc))
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)

    def perform_tasks(self, *args):
        super().perform_tasks()
        self.on_stop = time.time() + self.reset_time
        for domain in self._domains:
            domain.stop = False
            for ghost_domain in domain.ghost_domains:
                ghost_domain.stop = False
            self._executor.submit(self.task, domain)
