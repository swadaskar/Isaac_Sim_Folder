# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from ..benchmark_robots.benchmark_robot_registry import BenchmarkRobotRegistry
from ..benchmark_robots.base_benchmark_robot_loader import BaseBenchmarkRobotLoader
from ..benchmark_controllers import franka_controllers

import numpy as np

from typing import Optional
from omni.isaac.franka import Franka

# This class is a singleton, and so it will be shared with the robot_registry that is used in extension.py
robot_registry = BenchmarkRobotRegistry()
"""

Before running this script, enable the robot_benchmark extension in the Isaac Sim UI under "Window->Extensions".
Open "Robot Benchmark" from the toolbar and take a look at the drop-down menu.  You will be able to select different robots
and controllers written for those robots.  Take note of what is already there.

Then, come back to this example, and set RUN_EXAMPLE=True.  This will add options to the drop-down menu.  It will create a clone of the
Franka robot under the name "Example Robot" that floats .1 m above the ground.
It will also create a controller for any Franka robot under the name "Example Controller".
This example_controller will be registered under our "Example Robot" Franka clone, and under the "Franka" robot that already appears in
the drop-down menu.  The "Franka" robot will have this new controller added to its pre-existing list of supported controllers, and the
"Example Robot" will have this controller as its only available controller.

----------------------------

Scrolling down, you will see that this python file is written as a script inside an "if RUN_EXAMPLE==True" block.  This script is always
run on the startup of the robot_benchmark extension because it is listed as a python module in "extension.toml".

The same goes for the empty folder "robot_benchmark/user".
The user should place register any new robots or controllers in the "robot_benchmark/user" folder by copying the structure found in this template 

-----------------------------

Note that there is further explanation at the bottom of this script that is meant to be read after reading through the script.

"""

RUN_EXAMPLE = False

# Here we create a RobotLoader that we can register in the robot_registry to have it become discoverable by the robot_benchmark extension


class ExampleRobotLoader(BaseBenchmarkRobotLoader):
    def __init__(self, name, **robot_kwargs):
        BaseBenchmarkRobotLoader.__init__(self, name, **robot_kwargs)

        # This is redundant with BaseBenchmarkRobotLoader.__init__ and is written here for clarity
        self._robot_kwargs = robot_kwargs

        # This is specific to this particular BaseBenchmarkRobotLoader
        self._articulation = None

    def load_robot(
        self,
        prim_path: str,
        name: str = "Franka",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ):
        """
        The optional arguments after name are not explicitly passed to the load_robot function.  They are provided in the 
        __init__() function to the ExampleRobotLoader class.  They are listed as optional arguments here for clarity to show
        what some of the robot_kwargs may be for this specific load_function.
        """

        self._articulation = Franka(prim_path, name, **self._robot_kwargs)

        return self._articulation

    def get_robot_articulation(self):
        return self._articulation

    # Other functions used in this example are already implemented in the BaseBenchmarkRobotLoader class.
    # See omni.isaac.robot_benchmark.benchmark_robots.base_benchmark_robot_loader.py for documentation


if RUN_EXAMPLE:

    """
    To make our example robot appear on the drop-down menu, we need to register it.
    Note that we can write the kwargs that we want to pass to the ExampleRobotLoader.load_robot() function 
    when we register the robot.  In this case, the Example Robot is set to float .1 m above the ground.
    """

    # print(robot_registry.get_robot_options()) -> prints ["Franka"]

    example_robot_loader = ExampleRobotLoader(
        "Example Robot", position=np.array([0, 0, 0.1])
    )  # This string is the name of the robot in the benchmark log headers
    robot_registry.register_robot(
        "Example Robot", example_robot_loader
    )  # This string is the name of the robot as it appears in the drop-down menu

    # print(robot_registry.get_robot_options()) -> prints ["Franka", "Example Robot"]

    """
    This is how you add a controller an existing robot
    """

    def example_controller_load_fun(**kwargs):
        # Implement a controller that fulfills the robot_benchmark.benchmark_controllers.base_benchmark_controller interface
        controller = franka_controllers.RMPFlowBenchmarkController(
            kwargs["controller_name"], kwargs["robot_loader"].get_robot_articulation()
        )
        return controller

    # This gets the Franka robot loader that is already registered in the robot registry.  We can now register our example controller with the Franka
    franka_loader = robot_registry.get_robot_loader("Franka")
    franka_loader.register_controller(
        "Example Controller",
        example_controller_load_fun,
        controller_name="Example Controller",
        robot_loader=franka_loader,
    )

    # Add the controller to our example robot:
    example_robot_loader.register_controller(
        "Example Controller",
        example_controller_load_fun,
        controller_name="Example Controller",
        robot_loader=example_robot_loader,
    )


"""
You may be wondering a few things about this example script:

What is a RobotLoader, and why is it necessary?
    A RobotLoader exists so that you can register a robot without initializing it until the correct moment.
    Isaac Sim has a few annoying properties around Articulations.  When you load an Articulation, it gets added to 
    the USD stage.  You have to do this BEFORE pressing "play".  But if you try to initialize the Articulation with 
    Articulation.initialize() (which is necessary to control the robot), it will fail with confusing errors unless "play"
    has already been pressed.  
    
    By filling in a BaseBenchmarkRobotLoader interface, the robot_benchmark extension takes care of timing these things 
    appropriately.  Additionally, you can write many robot_loaders, but only the robot_loader that is selected in the 
    drop-down menu will actually place a robot on the stage.  

Why is the controller registered the way it is? 
    The controller is registered with a function to be called later because, once again, the timing needs to be correct to
    have everything get initialized without errors.  In robot_benchmark.py, the controller load_function is called
    after the robot Articulation has been initialized.  This structure allows for multiple controllers to be written for a robot,
    but only one controller will ever be initialized at a time.
    
    When writing a load_function for a controller, you may take in kwargs that you supply to the register_controller() function.
    These kwargs are stored internally to be passed along later when the load function is called.  
    
    Notice in the example_controller_load_fun that one kwarg was the robot_loader object.  
    The RMPFlow controller requires an initialized robot Articulation object to function.  It would fail if,
    instead of passing the robot_loader as a kwarg, we passed an argument "robot_articulation = robot_loader.get_robot_articulation()".
    This is because at the time this script is run, robot_loader.get_robot_articulation() returns None because the robot 
    has not been loaded yet.  But, by the time that robot_benchmark.py calls the controller_load_function, the robot Articulation
    will have been initialized.

What do you mean when you mention logs in robot_benchmark.py?
    When robot_benchmark is initialized, it can be set to log data to a json file.  This is pointless when running it from the UI, but
    there is a provided standalone script for running the (environment,robot,policy) pairs of your choice with data logging.  See the 
    top of the standalone_benchmark_runner.py script for a more detailed explanation.
"""
