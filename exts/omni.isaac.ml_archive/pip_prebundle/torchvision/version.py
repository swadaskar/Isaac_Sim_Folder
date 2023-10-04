__version__ = '0.14.0+cu117'
git_version = '5ce4506ac43c8b1dc1736ed9e51c58e0e29f5237'
from torchvision.extension import _check_cuda_version
if _check_cuda_version() > 0:
    cuda = _check_cuda_version()
