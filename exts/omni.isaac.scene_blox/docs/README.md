Installation
------------

Install scene builder package with python -m pip install tools/scene_blox (using the IsaacSim python or your conda environment, whichever applies).

Note that to run the display, a graphical backend for matplotlib should be installed if you are using the IsaacSim python.

Running
-------

Assuming here <python> is either the IsaacSim python.sh script or the python inside a conda environment
(and proper env sourcing has been done).

To generate some warehouses run

<python> tools/scene_blox/src/scene_blox/generate_scene.py <full_path_to_generated_scene_folder> --grid_config tools/scene_blox/parameters/warehouse/tile_config.yaml --generation_config tools/scene_blox/parameters/warehouse/tile_generation.yaml --cols 15 --rows 11 --constraints_config tools/scene_blox/parameters/warehouse/constraints.yaml --variants 1 --units_in_meters 1.0 --collisions

To generate some labyrinths run

<python> tools/scene_blox/src/scene_blox/generate_scene.py <full_path_to_generated_scene_folder> --grid_config tools/scene_blox/parameters/labyrinth/rules.yaml --generation_config tools/scene_blox/parameters/labyrinth/generation.yaml --cols 9 --rows 9 --constraints_config tools/scene_blox/parameters/labyrinth/constraints.yaml --variants 1

See docs folder for detailed information on the scripts and usage.

Note on server usage
--------------------

If the assets span across different servers (more than the default one), you should add mounted drives to
the command line to be able to find them by appending this to the commands

--/persistent/app/omniverse/mountedDrives=<drive_dictionary_value>

<drive_dictionary_value> where is a copy of the "mountedDrives" key in ~/.local/share/ov/data/Kit/Isaac-Sim/2022.2/user.config.json
