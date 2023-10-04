# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
from datetime import datetime
import omni.stats
import psutil
import nvsmi
import pprint
import numpy as np
import yaml
import copy


def get_memory_stats() -> dict:
    """Returns dictionary with memory usage staticstics in MB for GPU and Host memory

    Returns:
        dict: dictionary with memory usage statistics. The "Total" key contains totals for each category
    """
    memory_usage = {}

    process_id = os.getpid()
    process = psutil.Process(process_id)
    RAM = process.memory_info().rss * 1e-6  # byte to mb
    memory_usage["System Memory"] = {}
    memory_usage["System Memory"]["RAM"] = RAM
    gpu_processes = nvsmi.get_gpu_processes()
    VRAM = 0
    for gpu_process in gpu_processes:
        if gpu_process.pid == process_id:
            VRAM = gpu_process.used_memory
            break
    memory_usage["System Memory"]["VRAM"] = VRAM

    memory_usage["Total"] = {}
    stats_interface = omni.stats.get_stats_interface()
    scopes = stats_interface.get_scopes()
    for scope in scopes:
        stat_dict = {}
        scope_id = scope["scopeId"]
        stats = stats_interface.get_stats(scope_id)
        total = 0
        for s in stats:
            stat_dict[s["name"]] = s["value"]
            total += s["value"]

        memory_usage[scope["name"]] = stat_dict
        memory_usage["Total"][scope["name"]] = total

    return memory_usage


def get_memory_delta(start: dict, end: dict) -> dict:
    """Computes the difference between two memory reports computed using get_memory_stats

    Args:
        start (dict): get_memory_stats from point A in time
        end (dict): get_memory_stats from point B in time

    Returns:
        dict: returns memory report for B - A
    """
    # Create a copy so we don't have to start from scratch
    result = copy.deepcopy(start)
    for k, v in start.items():
        d_start = v
        d_end = end[k]
        d_result = result[k]
        if isinstance(d_start, dict):
            for kk, vv in d_start.items():
                i_start = vv
                i_end = d_end[kk]
                d_result[kk] = i_end - i_start

    return result


def read_log_file(log_file: str) -> dict:
    """Reads a Isaac Statistics log file and outputs it as a dictionary'

    Args:
        log_path (str): Path to log file
    """

    if not os.path.isfile(log_file):
        raise ValueError("Unable to find Isaac Statistics log file: '{}'".format(log_file))

    with open(log_file, "r") as f:
        try:
            print("Parsing Isaac Statistics log file...")
            log = yaml.safe_load(f)
        except:
            raise ValueError("Unable to load Isaac Statistics log file: '{}'".format(log_file))

    return log


def plot_statistics_log(log_file: str) -> None:
    """Parses a Isaac Statistics log file and outputs plot(s).'

    Args:
        log_path (str): Path to log file
    """
    import matplotlib.pyplot as plt

    log = read_log_file(log_file)

    n_samples = len(log)
    if n_samples == 0:
        print("Since Isaac Statistics log file has no entries, no plot will be output.")
        return

    # Sort log entry timestamps
    time_format = "%Y-%m-%d %H:%M:%S.%f"
    timestamps = []
    for timestamp in log.keys():
        timestamps.append(datetime.strptime(timestamp, time_format))
    timestamps.sort()

    # Initialize processed log with all categories
    skip_categories = set(["mode", "step"])
    processed_log = {}
    for timestamp, entry_dict in log.items():
        for category, category_dict in entry_dict.items():
            if category not in skip_categories:
                for metric, _ in category_dict.items():
                    if category not in processed_log:
                        processed_log[category] = {}
                    processed_log[category][metric] = []

    for timestamp in timestamps:
        entry_dict = log[str(timestamp)]
        for category, category_dict in processed_log.items():
            if category in entry_dict.keys():
                # If category exists in log entry, add entry values
                entry_category_dict = entry_dict[category]
                for metric, metric_val in entry_category_dict.items():
                    processed_log[category][metric].append(metric_val)
            else:
                # Else, pad with NaN values
                for metric in category_dict.keys():
                    processed_log[category][metric].append(float("NaN"))

    # Re-organize log categories
    units = {
        "GPU Pipelines": "count",
        "Scene Assets": "count",
        "Scene Triangles & Vertices": "count",
        "Virtual Texturing": "count",
        "Scene Dimensions": "scene units",
        "System Memory": "MB",
        "GPU Memory": "MB",
        "Other": "",
    }
    remapped_log = dict([(key, {}) for key in units])
    for category, category_dict in processed_log.items():
        for metric, metric_list in category_dict.items():

            if category == "GPU Pipelines":
                remapped_log["GPU Pipelines"][metric] = metric_list
            elif category == "RTX Scene":
                if "SceneBoundingBox" not in metric:
                    if metric.startswith((("Triangle", "Vertex"))):
                        remapped_log["Scene Triangles & Vertices"][metric] = metric_list
                    else:
                        remapped_log["Scene Assets"][metric] = metric_list
                else:
                    remapped_log["Scene Dimensions"][metric] = metric_list
            elif category == "Virtual Texturing":
                pass
            elif category == "System Memory":
                remapped_log["System Memory"][metric] = metric_list
            elif category == "GPU Memory":
                remapped_log["GPU Memory"][metric] = metric_list
            else:
                pass

    # Output plot per category
    first_timestamp = str(timestamps[0])
    for category, category_dict in remapped_log.items():
        if not category_dict:
            continue

        x_step = float(log[first_timestamp]["step"])
        mode = log[first_timestamp]["mode"]
        x_label = "every {} {}".format(x_step, mode)
        y_label = units[category]

        plt.rcParams.update({"font.size": 13})
        fig = plt.figure(figsize=(16, 8))
        ax = plt.subplot()
        cm = plt.get_cmap("gist_rainbow")
        n_colors = len(category_dict.keys())
        ax.set_prop_cycle("color", [cm(1.0 * i / n_colors) for i in range(n_colors)])

        # Plot metrics
        x = x_step * np.arange(n_samples)
        for label, y in category_dict.items():
            ax.plot(x, y, label=label)

        # Create legend
        handles, labels = ax.get_legend_handles_labels()
        handles, labels = zip(*sorted(list(zip(handles, labels)), key=lambda x: x[1]))
        ax.set_position([0.07, 0.1, 0.72, 0.82])
        ax.legend(handles, labels, loc="upper left", bbox_to_anchor=(1, 1), fontsize=9, frameon=False)
        if category != "Scene Dimensions":
            ax.set_ylim(bottom=0)
        ax.set_xlim(left=0)

        plt.xlabel(x_label, labelpad=14)
        plt.ylabel(y_label, labelpad=10)
        plt.title("{} Statistics".format(category), pad=16)

        # Resolve output filename
        output_dir = os.path.dirname(log_file)
        plot_file = os.path.join("{}/{}_plot.png".format(output_dir, category.lower().replace(" ", "_")))
        print(f"[omni.isaac.statistics_logging] Plotting to file: {plot_file}")
        fig.savefig(plot_file)


def summarize_statistics_log(log_file: str, system_memory_only=True):
    """Parses a Isaac Statistics log file and outputs summary.

    Args:
        log_file (str): Path to log file
    """

    log = list(read_log_file(log_file).items())
    delta = get_memory_delta(log[0][1], log[-1][1])
    if not system_memory_only:
        print("Delta between start and end of log")
        pprint.pprint(delta)
    else:
        print("Change in memory usage in mb")
        print(delta["System Memory"])
