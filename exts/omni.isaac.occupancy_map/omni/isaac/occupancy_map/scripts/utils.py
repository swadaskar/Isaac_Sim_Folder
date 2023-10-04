# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


def update_location(om, start_location, lower_bound, upper_bound):
    om.set_transform(
        (start_location[0], start_location[1], start_location[2]),
        (lower_bound[0], lower_bound[1], lower_bound[2]),
        (upper_bound[0], upper_bound[1], upper_bound[2]),
    )
    om.update()


def compute_coordinates(om, cell_size):
    import numpy as np

    min_b = om.get_min_bound()
    max_b = om.get_max_bound()
    scale = cell_size
    half_w = scale * 0.5
    top_left = (max_b[0] - half_w, min_b[1] + half_w)
    top_right = (min_b[0] + half_w, min_b[1] + half_w)
    bottom_left = (max_b[0] - half_w, max_b[1] - half_w)
    bottom_right = (min_b[0] + half_w, max_b[1] - half_w)

    image_coords = np.matrix([[0, 1], [-1, 0]]) * np.matrix([[-top_left[0]], [-top_left[1]]])

    return top_left, top_right, bottom_left, bottom_right, image_coords


def generate_image(om, occupied_col, unknown_col, freespace_col):
    buffer = om.get_buffer()
    dims = om.get_dimensions()
    image = unknown_col * dims[0] * dims[1]
    idx = 0
    for b in buffer:
        if b == 1.0:
            image[idx * 4 + 0] = occupied_col[0]
            image[idx * 4 + 1] = occupied_col[1]
            image[idx * 4 + 2] = occupied_col[2]
            image[idx * 4 + 3] = occupied_col[3]
        if b == 0.0:
            image[idx * 4 + 0] = freespace_col[0]
            image[idx * 4 + 1] = freespace_col[1]
            image[idx * 4 + 2] = freespace_col[2]
            image[idx * 4 + 3] = freespace_col[3]
        idx += 1
    return image
