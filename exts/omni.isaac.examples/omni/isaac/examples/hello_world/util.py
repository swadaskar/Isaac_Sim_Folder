import carb
import numpy as np
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.universal_robots import KinematicsSolver
from omni.isaac.motion_generation import WheelBasePoseController
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController

from omni.isaac.core.prims import GeometryPrim, XFormPrim

class CustomDifferentialController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.125
        self._wheel_base = 1.152
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0, 0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[2] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[3] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)
    
    def turn(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0, 0.0, 0.0]
        joint_velocities[0] = ((2 * command[0][0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0][1]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[2] = ((2 * command[0][2]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[3] = ((2 * command[0][3]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

class Utils:
    def __init__(self) -> None:
        self.world = None

        self.delay = 0
        
        self.path_plan_counter = 0
        self.motion_task_counter = 0
        self.motion_task_counterl = 0

        self.id = None

        # Engine cell set up ----------------------------------------------------------------------------
        # bring in moving platforms 
        self.moving_platform = None

        self._my_custom_controller = CustomDifferentialController()
        self._my_controller = WheelBasePoseController(name="cool_controller", open_loop_wheel_controller=DifferentialController(name="simple_control", wheel_radius=0.125, wheel_base=0.46), is_holonomic=False)

        self.my_controller = None
        self.screw_my_controller = None

        self.articulation_controller = None
        self.screw_articulation_controller = None

        # Suspension cell set up ------------------------------------------------------------------------

        self.my_controller_suspension = None
        self.screw_my_controller_suspension = None

        self.articulation_controller_suspension = None
        self.screw_articulation_controller_suspension = None


        # Fuel cell set up ---------------------------------------------------------------------------------

        self.my_controller_fuel = None
        self.screw_my_controller_fuel = None

        self.articulation_controller_fuel = None
        self.screw_articulation_controller_fuel = None

        # battery cell set up ---------------------------------------------------------------------------------

        self.my_controller_battery = None
        self.screw_my_controller_battery = None

        self.articulation_controller_battery = None
        self.screw_articulation_controller_battery = None

        # trunk cell set up ---------------------------------------------------------------------------------

        self.my_controller_trunk = None
        self.screw_my_controller_trunk = None

        self.articulation_controller_trunk = None
        self.screw_articulation_controller_trunk = None

        # wheel cell set up ---------------------------------------------------------------------------------

        self.my_controller_wheel = None
        self.screw_my_controller_wheel = None

        self.articulation_controller_wheel = None
        self.screw_articulation_controller_wheel = None

        self.my_controller_wheel_01 = None
        self.screw_my_controller_wheel_01 = None

        self.articulation_controller_wheel_01 = None
        self.screw_articulation_controller_wheel_01 = None

        # lower_cover cell set up ---------------------------------------------------------------------------------

        self.my_controller_lower_cover = None
        self.screw_my_controller_lower_cover = None

        self.articulation_controller_lower_cover = None
        self.screw_articulation_controller_lower_cover = None

        self.my_controller_lower_cover_01 = None
        self.screw_my_controller_lower_cover_01 = None

        self.articulation_controller_lower_cover_01 = None
        self.screw_articulation_controller_lower_cover_01 = None

        self.my_controller_main_cover = None

        self.articulation_controller_main_cover = None
        
        # handle cell set up ---------------------------------------------------------------------------------

        self.my_controller_handle = None
        self.screw_my_controller_handle = None

        self.articulation_controller_handle = None
        self.screw_articulation_controller_handle = None

        # light cell set up --------------------------------------------------------------------------------
        self.my_controller_light = None
        self.screw_my_controller_light = None

        self.articulation_controller_light = None
        self.screw_articulation_controller_light = None

    def give_location(self, prim_path):
        dc=_dynamic_control.acquire_dynamic_control_interface()
        object=dc.get_rigid_body(prim_path)
        object_pose=dc.get_rigid_body_pose(object)
        return object_pose # position: object_pose.p, rotation: object_pose.r
    
    def move_ur10(self, locations, task_name=""):
        print("Motion task counter", self.motion_task_counter)
        target_location = locations[self.motion_task_counter]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        controller_name = getattr(self,"my_controller"+task_name)
        actions, success = controller_name.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        if success:
            print("still homing on this location")
            articulation_controller_name = getattr(self,"articulation_controller"+task_name)
            articulation_controller_name.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

        # check if reached location
        curr_location = self.give_location(f"/World/UR10{task_name}/ee_link")
        print("Curr:",curr_location.p)
        print("Goal:", target_location["goal_position"])
        print(np.mean(np.abs(curr_location.p - target_location["goal_position"])))
        diff = np.mean(np.abs(curr_location.p - target_location["goal_position"]))
        if diff<0.02:
            self.motion_task_counter+=1
            # time.sleep(0.3)
            print("Completed one motion plan: ", self.motion_task_counter)
        
    def move_ur10_extra(self, locations, task_name=""):
        print("Motion task counter", self.motion_task_counterl)
        target_location = locations[self.motion_task_counterl]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        controller_name = getattr(self,"my_controller"+task_name)
        actions, success = controller_name.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        if success:
            print("still homing on this location")
            articulation_controller_name = getattr(self,"articulation_controller"+task_name)
            articulation_controller_name.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

        # check if reached location
        curr_location = self.give_location(f"/World/UR10{task_name}/ee_link")
        print("Curr:",curr_location.p)
        print("Goal:", target_location["goal_position"])
        print(np.mean(np.abs(curr_location.p - target_location["goal_position"])))
        diff = np.mean(np.abs(curr_location.p - target_location["goal_position"]))
        if diff<0.02:
            self.motion_task_counterl+=1
            # time.sleep(0.3)
            print("Completed one motion plan: ", self.motion_task_counterl)
    
    def do_screw_driving(self, locations, task_name=""):
        print(self.motion_task_counter)
        target_location = locations[self.motion_task_counter]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        controller_name = getattr(self,"screw_my_controller"+task_name)
        actions, success = controller_name.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        if success:
            print("still homing on this location")
            articulation_controller_name = getattr(self,"screw_articulation_controller"+task_name)
            # print(articulation_controller_name, task_name, "screw_articulation_controller"+task_name, self.screw_articulation_controller_wheel)
            # print(self.articulation_controller_wheel_01)
            # print(self.screw_articulation_controller_wheel_01)
            articulation_controller_name.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")
        # check if reached location
        curr_location = self.give_location(f"/World/Screw_driving_UR10{task_name}/ee_link")
        print("Curr:",curr_location.p)
        print("Goal:", target_location["goal_position"])
        print(np.mean(np.abs(curr_location.p - target_location["goal_position"])))
        if np.mean(np.abs(curr_location.p - target_location["goal_position"]))<0.02:
            self.motion_task_counter+=1
            print("Completed one motion plan: ", self.motion_task_counter)

    def do_screw_driving_extra(self, locations, task_name=""):
        print(self.motion_task_counterl)
        target_location = locations[self.motion_task_counterl]
        print("Doing "+str(target_location["index"])+"th motion plan")
        
        controller_name = getattr(self,"screw_my_controller"+task_name)
        actions, success = controller_name.compute_inverse_kinematics(
            target_position=target_location["position"],
            target_orientation=target_location["orientation"],
        )
        if success:
            print("still homing on this location")
            articulation_controller_name = getattr(self,"screw_articulation_controller"+task_name)
            articulation_controller_name.apply_action(actions)
        else:
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")
        # check if reached location
        curr_location = self.give_location(f"/World/Screw_driving_UR10{task_name}/ee_link")
        print("Curr:",curr_location.p)
        print("Goal:", target_location["goal_position"])
        print(np.mean(np.abs(curr_location.p - target_location["goal_position"])))
        if np.mean(np.abs(curr_location.p - target_location["goal_position"]))<0.02:
            self.motion_task_counterl+=1
            print("Completed one motion plan: ", self.motion_task_counterl)

    def transform_for_screw_ur10(self, position):
        position[0]+=0.16171
        position[1]+=0.00752
        position[2]+=-0
        return position

    def transform_for_ur10(self, position):
        position[0]+=0.16171
        position[1]+=0.00752
        position[2]+=-0.00419
        return position    

    def transform_for_screw_ur10_suspension(self, position):
        position[0]-=0
        position[1]+=0
        position[2]+=-0
        return position
    
    def transform_for_screw_ur10_fuel(self, position):
        position[0]+=0.16171
        position[1]+=0.00752
        position[2]+=-0.00419
        return position   
    
    def move_mp_wbpc(self, path_plan_last):
        print("Using wheel base pose controller")
        _, _, goal_position = path_plan_last
        position, orientation = self.moving_platform.get_world_pose()
        # In the function where you are sending robot commands
        print(goal_position)
        action = self._my_controller.forward(start_position=position, start_orientation=orientation, goal_position=goal_position["position"])  # Change the goal position to what you want
        full_action = ArticulationAction(joint_efforts=np.concatenate([action.joint_efforts, action.joint_efforts]) if action.joint_efforts else None, joint_velocities=np.concatenate([action.joint_velocities, action.joint_velocities]), joint_positions=np.concatenate([action.joint_positions, action.joint_positions]) if action.joint_positions else None)
        self.moving_platform.apply_action(full_action)
        print("Current", position)
        print("Goal", goal_position["position"])
        print(np.mean(np.abs(position-goal_position["position"])))
        if np.mean(np.abs(position-goal_position["position"])) <0.033:
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            self.path_plan_counter+=1
    
    def move_mp(self, path_plan):
        if not path_plan:
            return

        if len(path_plan)-1 == self.path_plan_counter and path_plan[self.path_plan_counter][0]!="rotate" and path_plan[self.path_plan_counter][0]!="wait":
            self.move_mp_wbpc(path_plan[self.path_plan_counter])
            return

        current_mp_position, current_mp_orientation = self.moving_platform.get_world_pose()

        move_type, goal = path_plan[self.path_plan_counter][0], path_plan[self.path_plan_counter][1]
        if move_type == "translate":
            goal_pos, axis, reverse = goal
            print(current_mp_position[axis], goal_pos, abs(current_mp_position[axis]-goal_pos))
            if reverse:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[-0.5,0])) # 0.5
            else:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if abs(current_mp_position[axis]-goal_pos)<0.01:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "rotate":
            goal_ori, error_threshold, rotate_right = goal
            if rotate_right:
                self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2])) # 2
            else:
                self.moving_platform.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],-np.pi/2]))
            curr_error = np.mean(np.abs(current_mp_orientation-goal_ori))
            print(current_mp_orientation, goal_ori, curr_error)
            if curr_error< error_threshold:
                self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "wait":
            print("Waiting ...")
            self.moving_platform.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if self.delay>60:
                print("Done waiting")
                self.delay=0
                self.path_plan_counter+=1
            self.delay+=1


    def add_part(self, part_name, prim_name, scale, position, orientation):
        world = self.get_world()
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/mock_robot/platform/{prim_name}") # gives asset ref path
        part= world.scene.add(XFormPrim(prim_path=f'/mock_robot_{self.id}/platform/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)

    def add_part_custom(self, parent_prim_name, part_name, prim_name, scale, position, orientation):
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/{parent_prim_name}/{prim_name}") # gives asset ref path
        part= self.world.scene.add(XFormPrim(prim_path=f'/{parent_prim_name}/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)
        return part
    
    def add_part_without_parent(self, part_name, prim_name, scale, position, orientation):
        world = self.get_world()
        base_asset_path = "/home/lm-2023/Isaac_Sim/isaac sim samples/real_microfactory/Materials/atvsstlfiles/"

        add_reference_to_stage(usd_path=base_asset_path+f"{part_name}/{part_name}.usd", prim_path=f"/World/{prim_name}") # gives asset ref path
        part= world.scene.add(XFormPrim(prim_path=f'/World/{prim_name}', name=f"q{prim_name}")) # declares in the world

        ## add part
        part.set_local_scale(scale)
        part.set_local_pose(translation=position, orientation=orientation)
        return part

    def remove_part(self, parent_prim_name, child_prim_name):

        prim_path = f"/{parent_prim_name}/{child_prim_name}"
        # world = self.get_world()

        prims.delete_prim(prim_path)

    def check_prim_exists(self, prim_path):
        curr_prim = self.world.stage.GetPrimAtPath("/"+prim_path)
        if curr_prim.IsValid():
            return True
        return False

    def move_mp_battery(self, path_plan):
        if not path_plan:
            return
        current_mp_position, current_mp_orientation = self.battery_bringer.get_world_pose()
        move_type, goal = path_plan[self.path_plan_counter]
        if move_type == "translate":
            goal_pos, axis, reverse = goal
            print(current_mp_position[axis], goal_pos, abs(current_mp_position[axis]-goal_pos))
            if reverse:
                self.battery_bringer.apply_action(self._my_custom_controller.forward(command=[-0.5,0]))
            else:
                self.battery_bringer.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if abs(current_mp_position[axis]-goal_pos)<0.01:
                self.battery_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "rotate":
            goal_ori, error_threshold, rotate_right = goal
            if rotate_right:
                self.battery_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            else:
                self.battery_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],-np.pi/2]))
            curr_error = np.mean(np.abs(current_mp_orientation-goal_ori))
            print(current_mp_orientation, goal_ori, curr_error)
            if curr_error< error_threshold:
                self.battery_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "wait":
            print("Waiting ...")
            self.battery_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if self.delay>60:
                print("Done waiting")
                self.delay=0
                self.path_plan_counter+=1
            self.delay+=1

    def move_mp_fuel(self, path_plan):
        if not path_plan:
            return
        current_mp_position, current_mp_orientation = self.fuel_bringer.get_world_pose()
        move_type, goal = path_plan[self.path_plan_counter]
        if move_type == "translate":
            goal_pos, axis, reverse = goal
            print(current_mp_position[axis], goal_pos, abs(current_mp_position[axis]-goal_pos))
            if reverse:
                self.fuel_bringer.apply_action(self._my_custom_controller.forward(command=[-0.5,0]))
            else:
                self.fuel_bringer.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if abs(current_mp_position[axis]-goal_pos)<0.01:
                self.fuel_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "rotate":
            goal_ori, error_threshold, rotate_right = goal
            if rotate_right:
                self.fuel_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            else:
                self.fuel_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],-np.pi/2]))
            curr_error = np.mean(np.abs(current_mp_orientation-goal_ori))
            print(current_mp_orientation, goal_ori, curr_error)
            if curr_error< error_threshold:
                self.fuel_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "wait":
            print("Waiting ...")
            self.fuel_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if self.delay>60:
                print("Done waiting")
                self.delay=0
                self.path_plan_counter+=1
            self.delay+=1

    def move_mp_suspension(self, path_plan):
        if not path_plan:
            return
        current_mp_position, current_mp_orientation = self.suspension_bringer.get_world_pose()
        move_type, goal = path_plan[self.path_plan_counter]
        if move_type == "translate":
            goal_pos, axis, reverse = goal
            print(current_mp_position[axis], goal_pos, abs(current_mp_position[axis]-goal_pos))
            if reverse:
                self.suspension_bringer.apply_action(self._my_custom_controller.forward(command=[-0.5,0]))
            else:
                self.suspension_bringer.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if abs(current_mp_position[axis]-goal_pos)<0.01:
                self.suspension_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "rotate":
            goal_ori, error_threshold, rotate_right = goal
            if rotate_right:
                self.suspension_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            else:
                self.suspension_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],-np.pi/2]))
            curr_error = np.mean(np.abs(current_mp_orientation-goal_ori))
            print(current_mp_orientation, goal_ori, curr_error)
            if curr_error< error_threshold:
                self.suspension_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "wait":
            print("Waiting ...")
            self.suspension_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if self.delay>60:
                print("Done waiting")
                self.delay=0
                self.path_plan_counter+=1
            self.delay+=1

    def move_mp_engine(self, path_plan):
        if not path_plan:
            return
        current_mp_position, current_mp_orientation = self.engine_bringer.get_world_pose()
        move_type, goal = path_plan[self.path_plan_counter]
        if move_type == "translate":
            goal_pos, axis, reverse = goal
            print(current_mp_position[axis], goal_pos, abs(current_mp_position[axis]-goal_pos))
            if reverse:
                self.engine_bringer.apply_action(self._my_custom_controller.forward(command=[-0.5,0]))
            else:
                self.engine_bringer.apply_action(self._my_custom_controller.forward(command=[0.5,0]))
            if abs(current_mp_position[axis]-goal_pos)<0.01:
                self.engine_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "rotate":
            goal_ori, error_threshold, rotate_right = goal
            if rotate_right:
                self.engine_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],np.pi/2]))
            else:
                self.engine_bringer.apply_action(self._my_custom_controller.turn(command=[[0,0,0,0],-np.pi/2]))
            curr_error = np.mean(np.abs(current_mp_orientation-goal_ori))
            print(current_mp_orientation, goal_ori, curr_error)
            if curr_error< error_threshold:
                self.engine_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
                self.path_plan_counter+=1
        elif move_type == "wait":
            print("Waiting ...")
            self.engine_bringer.apply_action(self._my_custom_controller.forward(command=[0,0]))
            if self.delay>60:
                print("Done waiting")
                self.delay=0
                self.path_plan_counter+=1
            self.delay+=1