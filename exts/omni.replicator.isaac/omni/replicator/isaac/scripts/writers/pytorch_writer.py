# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from omni.replicator.core import Writer, AnnotatorRegistry, BackendDispatch, WriterRegistry
from omni.replicator.isaac.scripts.writers.pytorch_listener import PytorchListener
import torch
import warp as wp

__version__ = "0.0.1"


class PytorchWriter(Writer):
    """ A custom writer that uses omni.replicator API to retrieve RGB data via render products
        and formats them as tensor batches. The writer takes a PytorchListener which is able 
        to retrieve pytorch tensors for the user directly after each writer call. 

    Args:
        listener (PytorchListener): A PytorchListener that is sent pytorch batch tensors at each write() call.
        output_dir (str): directory in which rgb data will be saved in PNG format by the backend dispatch. 
                          If not specified, the writer will not write rgb data as png and only ping the 
                          listener with batched tensors.
        device (str): device in which the pytorch tensor data will reside. Can be "cpu", "cuda", or any 
                      other format that pytorch supports for devices.    
    """

    def __init__(self, listener: PytorchListener, output_dir: str = None, device: str = "cpu"):
        # If output directory is specified, writer will write annotated data to the given directory
        if output_dir:
            self.backend = BackendDispatch({"paths": {"out_dir": output_dir}})
            self._backend = self.backend
            self._output_dir = self.backend.output_dir
        else:
            self._output_dir = None
        self._frame_id = 0

        self.annotators = [AnnotatorRegistry.get_annotator("LdrColor", device="cuda")]
        self.listener = listener
        self.device = device
        self.version = __version__

    def write(self, data: dict) -> None:
        """ Sends data captured by the attached render products to the PytorchListener and will write data to 
            the output directory if specified during initialization.
        
            Args:
                data (dict): Data to be pinged to the listener and written to the output directory if specified.
        """
        if self._output_dir:
            # Write RGB data to output directory as png
            self._write_rgb(data)
        pytorch_rgb = self._convert_to_pytorch(data)
        self.listener.write_data({"pytorch_rgb": pytorch_rgb, "device": self.device})
        self._frame_id += 1

    def _write_rgb(self, data: dict) -> None:
        for annotator in data.keys():
            if annotator.startswith("LdrColor"):
                render_product_name = annotator.split("-")[-1]
                file_path = f"rgb_{self._frame_id}_{render_product_name}.png"
                img_data = data[annotator]
                if isinstance(img_data, wp.types.array):
                    img_data = img_data.numpy()
                self._backend.write_image(file_path, img_data)

    def _convert_to_pytorch(self, data: dict) -> torch.Tensor:
        if data is None:
            raise Exception("Data is Null")

        data_tensor = None
        for annotator in data.keys():
            if annotator.startswith("LdrColor"):
                if data_tensor is None:
                    data_tensor = torch.tensor(data[annotator], dtype=torch.int32, device=self.device).unsqueeze(0)
                else:
                    rgb_tensor = torch.tensor(data[annotator], dtype=torch.int32, device=self.device).unsqueeze(0)
                    data_tensor = torch.cat((data_tensor, rgb_tensor), dim=0)
        return data_tensor


WriterRegistry.register(PytorchWriter)
