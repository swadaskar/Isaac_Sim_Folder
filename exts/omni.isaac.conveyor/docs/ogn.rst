


IsaacConveyor
-------------
    Conveyor Belt


**Inputs**
    - **enabled** (*bool*): node does not execute if disabled. Default to True.
    - **conveyorPrim** (*bundle*): the prim reference to the conveyor.
    - **velocity** (*float*): the velocity of the conveyor unit. Default to 0.0.
    - **direction** (*float[3]*): relative direction to apply velocity. Default to [1.0, 0.0, 0.0].
    - **curved** (*bool*): If the conveyor belt is curved, check this box to apply angular velocities. The rotation origin will be the rigid body origin. Default to False.
    - **animateTexture** (*bool*): If configured, Animates the texture based on the conveyor velocity. may affect performance specially if multiple instances are added to a scene. Default to False.
    - **animateScale** (*float*): how fast to scale animate texture. Default to 1.0.
    - **animateDirection** (*float[2]*): relative direction to apply to UV texture. Default to [1.0, 0.0].
    - **onStep** (*execution*): step to animate textures.
    - **delta** (*float*): time since last step in seconds.