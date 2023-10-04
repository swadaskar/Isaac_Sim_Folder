# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and itslicensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import argparse

from omni.isaac.kit import SimulationApp


def define_arguments():
    """ Define command line arguments. """

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="parameters/warehouse.yaml", help="Path to input parameter file")
    parser.add_argument(
        "--visualize-models",
        "--visualize_models",
        action="store_true",
        help="Output visuals of all object models defined in input parameter file, instead of outputting a dataset.",
    )
    parser.add_argument("--mount", help="Path to mount symbolized in parameter files via '*'.")
    parser.add_argument("--headless", action="store_true", help="Will not launch Isaac SIM window.")
    parser.add_argument("--nap", action="store_true", help="Will nap Isaac SIM after the first scene is generated.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrites dataset in output directory.")
    parser.add_argument("--output", type=str, help="Output directory. Overrides 'output_dir' param.")
    parser.add_argument(
        "--num-scenes", "--num_scenes", type=int, help="Num scenes in dataset. Overrides 'num_scenes' param."
    )
    parser.add_argument(
        "--nucleus-server", "--nucleus_server", type=str, help="Nucleus Server URL. Overrides 'nucleus_server' param."
    )

    return parser


# This allows us to run replicator, which will update the random
# parameters and save out the data for as many frames as listed
def run_orchestrator():
    rep.orchestrator.run()

    # Wait until started
    while not rep.orchestrator.get_is_started():
        kit.update()

    # Wait until stopped
    while rep.orchestrator.get_is_started():
        kit.update()

    rep.BackendDispatch.wait_until_done()

    rep.orchestrator.stop()


if __name__ == "__main__":
    # Create argument parser
    parser = define_arguments()
    args, _ = parser.parse_known_args()

    # Default rendering parameters
    config = {"renderer": "RayTracedLighting", "headless": args.headless, "width": 1280, "height": 720}

    kit = SimulationApp(launch_config=config)

    from omni.replicator.composer import Composer, Visualizer

    # from omni.replicator.composer import Composer
    import omni.replicator.core as rep

    root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")

    if args.visualize_models:
        visualizer = Visualizer(
            input_file_path=args.input,
            output_dir=args.output,
            root_dir=root_dir,
            nucleus_server=args.nucleus_server,
            overwrite=args.overwrite,
            mount=args.mount,
        )

    else:
        # Initialize composer
        composer = Composer(
            input_file_path=args.input,
            output_dir=args.output,
            root_dir=root_dir,
            num_scenes=args.num_scenes,
            nucleus_server=args.nucleus_server,
            overwrite=args.overwrite,
            mount=args.mount,
        )
        composer.generate_scene()

    if args.nap:
        rep.orchestrator.step()
        while True:
            kit.update()
    else:
        # Run replicator
        run_orchestrator()
        kit.update()
        rep.BackendDispatch.wait_until_done()
        kit.close()
