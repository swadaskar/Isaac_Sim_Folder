# Copyright (c) 2020-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.usd
import carb.events
from .base_reset_node import BaseResetNode
from pxr import Usd
import omni.replicator.core as rep
from typing import List, Union
import copy


class WriterRequest:
    def __init__(self, writer: rep.Writer, render_product_path: Union[str, List[str]], activate: bool = True):
        self.writer = writer
        self.render_product_path = render_product_path
        self.activate = activate

    def __repr__(self) -> str:
        output = f"{self.writer.node_type_id}\n\t{self.writer._kwargs}\n\t{self.render_product_path}\n\tAnnotators:\n"
        for a in self.writer._annotators:
            output = output + f"\t\t{a}\n"
        return output


class BaseWriterNode(BaseResetNode):
    """
        Base class for nodes that automatically reset when stop is pressed.
    """

    def __init__(self, initialize: bool = False):
        self._writers = []
        self._requests = []
        self._event_stream = None
        super().__init__(initialize=False)

    def custom_reset(self):
        for w in self._writers:
            self._append_request(WriterRequest(w, None, False))
        self._writers = []
        self.initialized = False

    def append_writer(self, writer):
        self._writers.append(copy.deepcopy(writer))

    def attach_writers(self, render_product_path):
        for w in self._writers:
            self._append_request(WriterRequest(w, render_product_path, True))

    def _append_request(self, request: WriterRequest):
        self._requests.append(request)
        if self._event_stream is None:
            self._event_stream = (
                omni.kit.app.get_app()
                .get_update_event_stream()
                .create_subscription_to_pop(self._process_acivation_requests)
            )

    def _process_acivation_requests(self, event):
        stage = omni.usd.get_context().get_stage()
        if not stage:
            self._event_stream = None
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for request in self._requests:
                try:
                    if request.activate:
                        request.writer.attach(request.render_product_path)
                        self.post_attach(request.writer, request.render_product_path)
                        carb.log_info(f"Attaching:\n{request}")
                    else:
                        request.writer.detach()
                except Exception as e:
                    carb.log_error(
                        f"Could not process writer attach request {request.writer, request.render_product_path}, {e}"
                    )
            # Stop processing additional requests until another one is appended
            self._requests = []
            self._event_stream = None

    # Defined by subclass
    def post_attach(self, writer, render_product):
        pass
