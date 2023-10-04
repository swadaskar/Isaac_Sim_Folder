


IsaacPickPlaceController
------------------------
    Pick-and-Place Controller for Articulated Robots

    The controller takes a robot, pick-place sequence timing, and pick and place targets

**Inputs**
    - **execIn** (*execution*): The input execution.
    - **targetPrim** (*bundle*): The target robot prim.
    - **usePath** (*bool*): use robot and com path instead of selecting them from stage tree. Default to False.
    - **robotModel** (*string*): type of robot. Options are: UR, Franka, or Dofbot.
    - **robotPrimPath** (*string*): path to the robot articulation root.
    - **pickingPosition** (*double[3]*): XYZ location to pick from. Default to [0.25, 0.25, 0.0].
    - **placingPosition** (*double[3]*): XYZ location to place at. Default to [0.25, -0.25, 0.0].
    - **endEffectorOffset** (*double[3]*): XYZ offset of end-effector from flange. Default to [0.0, 0.0, 0.0].
    - **eventsDT** (*double[]*): timing between pick and place events. Default to [0.01, 0.01, 1.0, 0.01, 0.01, 0.01, 0.01, 0.05, 0.01, 0.08].