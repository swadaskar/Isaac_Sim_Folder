import numpy as np


def as_type(data, dtype):
    if dtype == "float32":
        return data.astype(np.float32)
    elif dtype == "bool":
        return data.astype(np.bool)
    elif dtype == "int32":
        return data.astype(np.int32)
    elif dtype == "int64":
        return data.to(np.int64)
    elif dtype == "long":
        return data.to(np.long)
    else:
        print(f"Type {dtype} not supported.")


def convert(data, device=None):
    return np.asarray(data)


def create_zeros_tensor(shape, dtype, device=None):
    return as_type(np.zeros(shape), dtype)


def create_tensor_from_list(data, dtype, device=None):
    return as_type(np.array(data), dtype)


def clone_tensor(data, device=None):
    return np.copy(data)


def resolve_indices(indices, count, device=None):
    result = indices
    if isinstance(indices, list):
        result = np.array(indices)
    if indices is None:
        result = np.arange(count)
    return result


def move_data(data, device=None):
    return data


def tensor_cat(data, dim=-1):
    return np.concatenate(data, axis=dim)


def expand_dims(data, axis):
    return np.expand_dims(data, axis)


def pad(data, pad_width, mode="constant", value=None):
    if mode == "constant" and value is not None:
        return np.pad(data, pad_width, mode, constant_values=value)
    if mode == "linear_ramp" and value is not None:
        return np.pad(data, pad_width, mode, end_values=value)
    return np.pad(data, pad_width, mode)


def tensor_stack(data, dim=0):
    return np.stack(data, axis=dim)
