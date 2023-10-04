import os
import json
import carb
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from typing import List

"""This InterfaceLoader makes it trivial to load a valid config for supported interface implementations
For example, RMPflow has a collection of robot-specific config files stored in the motion_generation extension.
This loader makes it simple to load RMPflow for the Franka robot using load_supported_motion_policy_config("Franka","RMPflow")
"""


def get_supported_robot_policy_pairs() -> dict:
    """Get a dictionary of MotionPolicy names that are supported for each given robot name

    Returns:
        supported_policy_names_by_robot (dict): dictionary mapping robot names (keys) to a list of supported MotionPolicy config files (values)
    """
    mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
    policy_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    with open(os.path.join(policy_config_dir, "policy_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    supported_policy_names_by_robot = dict()
    for k, v in policy_map.items():
        supported_policy_names_by_robot[k] = list(v.keys())

    return supported_policy_names_by_robot


def get_supported_robots_with_lula_kinematics() -> List[str]:
    # Currently just uses robots that have RmpFlow supported
    robots = []

    pairs = get_supported_robot_policy_pairs()
    for k, v in pairs.items():
        if "RMPflow" in v:
            robots.append(k)
    return robots


def get_supported_robot_path_planner_pairs() -> dict:
    """Get a dictionary of PathPlanner names that are supported for each given robot name

    Returns:
        supported_planner_names_by_robot (dict): dictionary mapping robot names (keys) to a list of supported PathPlanner config files (values)
    """
    mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
    policy_config_dir = os.path.join(mg_extension_path, "path_planner_configs")
    with open(os.path.join(policy_config_dir, "path_planner_map.json")) as planner_map:
        planner_map = json.load(planner_map)

    supported_planner_names_by_robot = dict()
    for k, v in planner_map.items():
        supported_planner_names_by_robot[k] = list(v.keys())

    return supported_planner_names_by_robot


def load_supported_lula_kinematics_solver_config(robot_name: str, policy_config_dir=None) -> dict:
    """Load lula kinematics solver for a supported robot.
    Use get_supported_robots_with_lula_kinematics() to get a list of robots with supported kinematics.

    Args:
        robot_name (str): name of robot
    
    Returns:
        solver_config (dict): a dictionary whose keyword arguments are sufficient to load the lula kinematics solver.
            e.g. lula.LulaKinematicsSolver(**load_supported_lula_kinematics_solver_config("Franka"))

    """
    policy_name = "RMPflow"

    if policy_config_dir is None:
        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        policy_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    with open(os.path.join(policy_config_dir, "policy_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    if robot_name not in policy_map:
        carb.log_error(
            "Unsupported robot passed to InterfaceLoader.  Use get_supported_robots_with_lula_kinematics() to get a list of robots with supported kinematics"
        )
        return None
    if policy_name not in policy_map[robot_name]:
        carb.log_error(
            robot_name
            + " does not have supported lula kinematics.  Use get_supported_robots_with_lula_kinematics() to get a list of robots with supported kinematics"
        )
        return None

    config_path = os.path.join(policy_config_dir, policy_map[robot_name][policy_name])
    rmp_config = _process_policy_config(config_path)

    kinematics_config = dict()
    kinematics_config["robot_description_path"] = rmp_config["robot_description_path"]
    kinematics_config["urdf_path"] = rmp_config["urdf_path"]
    return kinematics_config


def load_supported_motion_policy_config(robot_name: str, policy_name: str, policy_config_dir: str = None) -> dict:
    """Load a MotionPolicy object by specifying the robot name and policy name
    For a dictionary mapping supported robots to supported policies on those robots,
    use get_supported_robot_policy_pairs()

    To use this loader for a new policy, a user may copy the config file structure found under /motion_policy_configs/
    in the motion_generation extension, passing in a path to a directory containing a "policy_map.json"

    Args:
        robot_name (str): name of robot
        policy_name (str): name of MotionPolicy
        policy_config_dir (str): path to directory where a policy_map.json file is stored,
            defaults to ".../omni.isaac.motion_generation/motion_policy_configs"
    
    Returns:
        policy_config (dict): a dictionary whose keyword arguments are sufficient to load the desired motion policy
            e.g. lula.motion_policies.RmpFlow(**load_supported_motion_policy_config("Franka","RMPflow"))
    """

    if policy_config_dir is None:
        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        policy_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    with open(os.path.join(policy_config_dir, "policy_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    if robot_name not in policy_map:
        carb.log_error(
            "Unsupported robot passed to InterfaceLoader.  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies"
        )
        return None
    if policy_name not in policy_map[robot_name]:
        carb.log_error(
            'Unsupported policy name passed to InterfaceLoader for robot "'
            + robot_name
            + '".  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies'
        )
        return None

    config_path = os.path.join(policy_config_dir, policy_map[robot_name][policy_name])
    config = _process_policy_config(config_path)

    return config


def load_supported_path_planner_config(robot_name: str, planner_name: str, policy_config_dir: str = None) -> dict:

    if policy_config_dir is None:
        mg_extension_path = get_extension_path_from_name("omni.isaac.motion_generation")
        policy_config_dir = os.path.join(mg_extension_path, "path_planner_configs")
    with open(os.path.join(policy_config_dir, "path_planner_map.json")) as policy_map:
        policy_map = json.load(policy_map)

    if robot_name not in policy_map:
        carb.log_error(
            "Unsupported robot passed to InterfaceLoader.  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies"
        )
        return None
    if planner_name not in policy_map[robot_name]:
        carb.log_error(
            'Unsupported policy name passed to InterfaceLoader for robot "'
            + robot_name
            + '".  Use get_supported_robot_policy_pairs() to see supported robots and their corresponding supported policies'
        )
        return None

    config_path = os.path.join(policy_config_dir, policy_map[robot_name][planner_name])
    config = _process_policy_config(config_path)

    return config


def _process_policy_config(mg_config_file):
    mp_config_dir = os.path.dirname(mg_config_file)
    with open(mg_config_file) as config_file:
        config = json.load(config_file)
    rel_assets = config.get("relative_asset_paths", {})
    for k, v in rel_assets.items():
        config[k] = os.path.join(mp_config_dir, v)
    del config["relative_asset_paths"]
    return config
