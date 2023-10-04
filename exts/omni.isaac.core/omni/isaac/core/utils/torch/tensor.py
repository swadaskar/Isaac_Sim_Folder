import torch


def as_type(data, dtype):
    if dtype == "float32":
        return data.to(torch.float32)
    elif dtype == "bool":
        return data.to(torch.bool)
    elif dtype == "int32":
        return data.to(torch.int32)
    elif dtype == "int64":
        return data.to(torch.int64)
    elif dtype == "long":
        return data.to(torch.long)
    else:
        print(f"Type {dtype} not supported.")


def convert(data, device):
    if not isinstance(data, torch.Tensor):
        return torch.tensor(data, device=device)
    else:
        return data.to(device=device)


def create_zeros_tensor(shape, dtype, device=None):
    return as_type(torch.zeros(shape, device=device), dtype)


def create_tensor_from_list(data, dtype, device=None):
    return as_type(torch.tensor(data, device=device), dtype=dtype)


def clone_tensor(data, device):
    data = data.to(device=device)
    return torch.clone(data)


def resolve_indices(indices, count, device):
    result = indices
    if isinstance(indices, list):
        result = torch.tensor(indices, dtype=torch.long, device=device)
    if indices is None:
        result = torch.arange(count, device=device)
    return result.to(dtype=torch.long, device=device)


def move_to_gpu(data):
    if isinstance(data, torch.Tensor) and not data.is_cuda:
        return data.cuda()
    if not isinstance(data, torch.Tensor):
        return torch.tensor(data).cuda()
    else:
        return data


def move_data(data, device):
    return data.to(device=device)


def tensor_cat(data, dim=-1):
    return torch.cat(data, dim=dim)


def expand_dims(data, axis):
    return torch.unsqueeze(data, axis)


def pad(data, pad_width, mode="constant", value=None):
    return torch.nn.functional.pad(data, pad_width, mode, value)


def tensor_stack(data, dim=0):
    return torch.stack(data, dim=dim)
