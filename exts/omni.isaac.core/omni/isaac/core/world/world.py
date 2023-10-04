# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# python
from typing import Optional, List

# omniverse
from pxr import Usd

# isaac-core
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.loggers import DataLogger


class World(SimulationContext):
    """ This class inherits from SimulationContext which provides the following.

        SimulationContext provide functions that take care of many time-related events such as
        perform a physics or a render step for instance. Adding/ removing callback functions that 
        gets triggered with certain events such as a physics step, timeline event 
        (pause or play..etc), stage open/ close..etc.

        It also includes an instance of PhysicsContext which takes care of many physics related
        settings such as setting physics dt, solver type..etc.
        
        In addition to what is provided from SimulationContext, this class allows the user to add a 
        task to the world and it contains a scene object.
        
        To control the default reset state of different objects easily, the object could be added to
        a Scene. Besides this, the object is bound to a short keyword that fascilitates objects retrievals,
        like in a dict.

        Checkout the required tutorials at 
        https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html

        Args:
            physics_dt (Optional[float], optional): dt between physics steps. Defaults to None.
            rendering_dt (Optional[float], optional): dt between rendering steps. Note: rendering means 
                                                       rendering a frame of the current application and not 
                                                       only rendering a frame to the viewports/ cameras. So UI
                                                       elements of Isaac Sim will be refereshed with this dt 
                                                       as well if running non-headless. 
                                                       Defaults to None.
            stage_units_in_meters (Optional[float], optional): The metric units of assets. This will affect gravity value..etc.
                                                       Defaults to None.
            physics_prim_path (Optional[str], optional): specifies the prim path to create a PhysicsScene at, 
                                                 only in the case where no PhysicsScene already defined. 
                                                 Defaults to "/physicsScene".
            set_defaults (bool, optional): set to True to use the defaults settings
                                            [physics_dt = 1.0/ 60.0,
                                            stage units in meters = 0.01 (i.e in cms),
                                            rendering_dt = 1.0 / 60.0,
                                            gravity = -9.81 m / s
                                            ccd_enabled,
                                            stabilization_enabled,
                                            gpu dynamics turned off,
                                            broadcast type is MBP,
                                            solver type is TGS]. Defaults to True.
            backend (str, optional): specifies the backend to be used (numpy or torch). Defaults to numpy.
            device (Optional[str], optional): specifies the device to be used if running on the gpu with torch backend.
        """

    _world_initialized = False

    def __init__(
        self,
        physics_dt: Optional[float] = None,
        rendering_dt: Optional[float] = None,
        stage_units_in_meters: Optional[float] = None,
        physics_prim_path: str = "/physicsScene",
        sim_params: dict = None,
        set_defaults: bool = True,
        backend: str = "numpy",
        device: Optional[str] = None,
    ) -> None:
        SimulationContext.__init__(
            self,
            physics_dt=physics_dt,
            rendering_dt=rendering_dt,
            stage_units_in_meters=stage_units_in_meters,
            physics_prim_path=physics_prim_path,
            sim_params=sim_params,
            set_defaults=set_defaults,
            backend=backend,
            device=device,
        )
        if World._world_initialized:
            return
        World._world_initialized = True
        self._task_scene_built = False
        self._current_tasks = dict()
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self._scene = Scene()
        # if not builtins.ISAAC_LAUNCHED_FROM_TERMINAL:
        #     self.start_simulation()
        set_camera_view(eye=[1.5, 1.5, 1.5], target=[0.01, 0.01, 0.01], camera_prim_path="/OmniverseKit_Persp")
        self._data_logger = DataLogger()
        return

    def __del__(self):
        SimulationContext.__del__(self)
        World._world_initialized = None
        return

    """
    Instance handling.
    """

    @classmethod
    def clear_instance(cls):
        SimulationContext.clear_instance()
        World._world_initialized = None
        return

    """
    Properties.
    """

    @property
    def dc_interface(self) -> _dynamic_control.DynamicControl:
        """[summary]

        Returns:
            _dynamic_control.DynamicControl: [description]
        """
        return self._dc_interface

    @property
    def scene(self) -> Scene:
        """[summary]

        Returns:
            Scene: [description]
        """
        return self._scene

    """
    Operations - Tasks management.
    """

    def add_task(self, task: BaseTask) -> None:
        """Tasks should have a unique name.


        Args:
            task (BaseTask): [description]
        """
        if task.name in self._current_tasks:
            raise Exception("Task name should be unique in the world")
        self._current_tasks[task.name] = task
        return

    def is_tasks_scene_built(self) -> bool:
        return self._task_scene_built

    def get_current_tasks(self) -> List[BaseTask]:
        """[summary]

        Returns:
            List[BaseTask]: [description]
        """
        return self._current_tasks

    def get_task(self, name: str) -> BaseTask:
        if name not in self._current_tasks:
            raise Exception("task name {} doesn't exist in the current world tasks.".format(name))
        return self._current_tasks[name]

    """
    Operations - Tasks state collection.
    """

    def get_observations(self, task_name: Optional[str] = None) -> dict:
        """Gets observations from all the tasks that were added

        Args:
            task_name (Optional[str], optional): [description]. Defaults to None.

        Returns:
            dict: [description]
        """
        if task_name is not None:
            return self._current_tasks[task_name].get_observations()
        else:
            observations = dict()
            for task in self._current_tasks.values():
                observations.update(task.get_observations())
            return observations

    def calculate_metrics(self, task_name: Optional[str] = None) -> None:
        """Gets metrics from all the tasks that were added

        Args:
            task_name (Optional[str], optional): [description]. Defaults to None.

        Returns:
            [type]: [description]
        """
        if task_name is not None:
            return self._current_tasks[task_name].calculate_metrics()
        else:
            metrics = dict()
            for task in self._current_tasks.values():
                metrics.update(task.calculate_metrics())
            return metrics

    def is_done(self, task_name: Optional[str] = None) -> None:
        """[summary]

        Args:
            task_name (Optional[str], optional): [description]. Defaults to None.

        Returns:
            [type]: [description]
        """
        if task_name is not None:
            return self._current_tasks[task_name].is_done()
        else:
            result = [task.is_done() for task in self._current_tasks.values()]
            return all(result)

    """
    Operations - Data logger.
    """

    def get_data_logger(self) -> DataLogger:
        """Returns the data logger of the world.

        Returns:
            DataLogger: [description]
        """
        return self._data_logger

    """
    Operations.
    """

    def initialize_physics(self) -> None:
        """_summary_
        """
        SimulationContext.initialize_physics(self)
        self._scene._finalize(self.physics_sim_view)
        return

    def reset(self, soft: bool = False) -> None:
        """ Resets the stage to its initial state and each object included in the Scene to its default state
            as specified by .set_default_state and the __init__ funcs. 

            Note:
            - All tasks should be added before the first reset is called unless a .clear() was called. 
            - All articulations should be added before the first reset is called unless a .clear() was called. 
            - This method takes care of initializing articulation handles with the first reset called.
            - This will do one step internally regardless
            - calls post_reset on each object in the Scene
            - calls post_reset on each Task

            things like setting pd gains for instance should happend at a Task reset or a Robot reset since
            the defaults are restored after .stop() is called.

        Args:
            soft (bool, optional): if set to True simulation won't be stopped and start again. It only calls the reset on the scene objects. 
        """
        if not self._task_scene_built:
            for task in self._current_tasks.values():
                task.set_up_scene(self.scene)
            self._task_scene_built = True
        if not soft:
            self.stop()
        for task in self._current_tasks.values():
            task.cleanup()
        SimulationContext.reset(self, soft=soft)
        self._scene._finalize(self.physics_sim_view)
        self.scene.post_reset()
        for task in self._current_tasks.values():
            task.post_reset()

    async def reset_async(self, soft: bool = False) -> None:
        """Resets the stage to its initial state and each object included in the Scene to its default state
            as specified by .set_default_state and the __init__ funcs. 

            Note:
            - All tasks should be added before the first reset is called unless a .clear() was called. 
            - All articulations should be added before the first reset is called unless a .clear() was called. 
            - This method takes care of initializing articulation handles with the first reset called.
            - This will do one step internally regardless
            - calls post_reset on each object in the Scene
            - calls post_reset on each Task

            things like setting pd gains for instance should happend at a Task reset or a Robot reset since
            the defaults are restored after .stop() is called.

        Args:
            soft (bool, optional): if set to True simulation won't be stopped and start again. It only calls the reset on the scene objects. 
        """
        if not self._task_scene_built:
            for task in self._current_tasks.values():
                task.set_up_scene(self.scene)
            self._task_scene_built = True
        if not soft:
            await self.stop_async()
        for task in self._current_tasks.values():
            task.cleanup()
        await SimulationContext.reset_async(self, soft=soft)
        self._scene._finalize(self.physics_sim_view)
        self._scene.post_reset()
        for task in self._current_tasks.values():
            task.post_reset()
        return

    def step(self, render: bool = True, step_sim: bool = True) -> None:
        """Steps the physics simulation while rendering or without.

           - Note: task pre_step is called here.

        Args:
            render (bool, optional): Set to False to only do a physics simulation without rendering. Note:
                                     app UI will be frozen (since its not rendering) in this case. 
                                     Defaults to True.

        """
        if self._task_scene_built:
            for task in self._current_tasks.values():
                task.pre_step(self.current_time_step_index, self.current_time)
        if self.scene._enable_bounding_box_computations:
            self.scene._bbox_cache.SetTime(Usd.TimeCode(self._current_time))

        if step_sim:
            SimulationContext.step(self, render=render)
        if self._data_logger.is_started():
            if self._data_logger._data_frame_logging_func is None:
                raise Exception("You need to add data logging function before starting the data logger")
            data = self._data_logger._data_frame_logging_func(tasks=self.get_current_tasks(), scene=self.scene)
            self._data_logger.add_data(
                data=data, current_time_step=self.current_time_step_index, current_time=self.current_time
            )
        return

    def step_async(self, step_size: Optional[float] = None) -> None:
        """Calls all functions that should be called pre stepping the physics

           - Note: task pre_step is called here.

        Args:
            step_size (float): [description]

        Raises:
            Exception: [description]
        """
        if self._task_scene_built:
            for task in self._current_tasks.values():
                task.pre_step(self.current_time_step_index, self.current_time)
        if self.scene._enable_bounding_box_computations:
            self.scene._bbox_cache.SetTime(Usd.TimeCode(self._current_time))
        if self._data_logger.is_started():
            if self._data_logger._data_frame_logging_func is None:
                raise Exception("You need to add data logging function before starting the data logger")
            data = self._data_logger._data_frame_logging_func(tasks=self.get_current_tasks(), scene=self.scene)
            self._data_logger.add_data(
                data=data, current_time_step=self.current_time_step_index, current_time=self.current_time
            )
        return

    def clear(self) -> None:
        """Clears the stage leaving the PhysicsScene only if under /World.
        """
        self.scene.clear(registry_only=False)
        self._current_tasks = dict()
        self._task_scene_built = False
        self._data_logger = DataLogger()
        # clear all prims in the stage.
        SimulationContext.clear(self)
