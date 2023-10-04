# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ext
import carb
import sys
from prompt_toolkit import print_formatted_text
from ptpython.repl import embed
from prompt_toolkit.contrib.telnet.server import TelnetServer
from prompt_toolkit.eventloop.utils import get_event_loop


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        if sys.platform == "win32":
            carb.log_error("Currently doesn't support windows")
        else:
            settings = carb.settings.get_settings()
            host = settings.get("/exts/omni.isaac.repl/host")
            port = settings.get("/exts/omni.isaac.repl/port")

            # Start embedded interpreter
            async def interact(connection=None):
                carb.log_info(f"Opening new connection {connection.conn}")
                global_dict = {**globals(), "print": print_formatted_text}
                await embed(return_asyncio_coroutine=True, globals=global_dict)
                carb.log_info(f"Closing connection {connection.conn}")

            self.telnet_server = TelnetServer(interact=interact, host=host, port=port)
            self.telnet_server.start()

            carb.log_info(f"Running telnet server on host {host} port {port}...")

    def on_shutdown(self):
        if sys.platform == "win32":
            return
        carb.log_info("Closing all telnet connections")
        # This forces all connections to close on the client side
        for c in self.telnet_server.connections:
            c.close()
        # Wait for Telnet server to shutdown
        loop = get_event_loop()
        loop.run_until_complete(self.telnet_server.stop())
        self.telnet_server = None
        carb.log_info("Telnet server shutdown complete")

        pass
