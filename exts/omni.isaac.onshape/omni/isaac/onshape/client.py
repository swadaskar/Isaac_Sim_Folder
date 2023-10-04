# Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni
import carb
import weakref
import gc

try:
    # from onshape_client import Client
    # import onshape_client
    from omni.isaac.onshape.onshape_client import Client
    import omni.isaac.onshape.onshape_client as onshape_client
except ImportError:
    carb.log_warn("onshape dependencies not found. attempting to install...")
    # the package name and module names are different, so install at runtime and ignore the import check.
    omni.kit.pipapi.install(
        "requests-oauthlib", version="1.3.0", extra_args=["--no-dependencies"], ignore_import_check=True
    )
    omni.kit.pipapi.install("ruamel.yaml", version="0.17.16", extra_args=["--no-dependencies"])
    # this module cannot be directly imported
    omni.kit.pipapi.install(
        "ruamel.yaml.clib", version="0.2.6", extra_args=["--no-dependencies"], ignore_import_check=True
    )
    omni.kit.pipapi.install("nulltype", version="2.3.1", extra_args=["--no-dependencies"])
    # omni.kit.pipapi.install("onshape_client", version="1.6.3", extra_args=["--no-dependencies"])
    # from onshape_client import Client
    # import onshape_client
    from omni.isaac.onshape.onshape_client import Client
    import omni.isaac.onshape.onshape_client as onshape_client

import omni.ui as ui
from concurrent.futures import ThreadPoolExecutor

import json

import webbrowser
from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, HTTPServer

from threading import Lock


class TimeoutException(Exception):
    pass


from .scripts.definitions import (
    USE_ONSHAPE_KEY,
    DEFAULT_ONSHAPE_KEY,
    DEFAULT_ONSHAPE_SECRET,
    ONSHAPE_BASE_URL,
    ONSHAPE_AUTH_URL,
    ONSHAPE_TOKEN_URL,
)


# class ThreadWithException(threading.Thread):
#     def __init__(self, bucket):
#         threading.Thread.__init__(self)


def set_api_keys(key, secret):
    carb.settings.get_settings().set(DEFAULT_ONSHAPE_KEY, key)
    carb.settings.get_settings().set(DEFAULT_ONSHAPE_SECRET, secret)


def do_auth():
    try:
        d = OnshapeClient.get().documents_api.get_documents()
        if d:
            OnshapeClient.set_authenticated(True)

    except Exception as e:
        carb.log_error("Onshape Authentication Error: {}".format(e))
        OnshapeClient.__stop_request = True
        return False


def callback(q=None):
    OnshapeClient.auth_callback()


class AuthWindow(ui.Widget):
    def __init__(self, parent):
        self.parent = parent
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="onshape_authentitator_pool")

        self.build_widget()

    def start_auth(self, callback_fn):
        # Process(target = auth_method, args=(self.queue,))

        OnshapeClient.__stop_request = False
        OnshapeClient.set_auth_callback(lambda a=self, c=callback_fn: a.done_auth(c))
        self.task = self.executor.submit(do_auth)

        self.task.add_done_callback(callback)
        # do_auth(OnshapeClient.get())
        # callback(0)

    def build_widget(self):
        with ui.VStack(alignment=(ui.Alignment.CENTER)):
            ui.Label("Onshape Authentication in Progress", alignment=(ui.Alignment.CENTER))
            self.cancel_btn = ui.Button("Cancel", clicked_fn=lambda: weakref.proxy(self).cancel_auth())
            ui.Spacer(height=5)

    def cancel_auth(self):
        # self.process.terminate()
        self.parent.visible = False
        OnshapeClient.__stop_request = True
        OnshapeClient.stop_httpServer()
        # for pid, process in self.executor._processes.items():
        #     process.terminate()
        self.executor.shutdown(wait=False)
        OnshapeClient.set_auth_callback(None)
        # self.queue.put(False)

    def done_auth(self, callback):
        # r = self.queue.get()
        # done = False
        # if len(r) > 1:
        #     done = r[1]
        self.parent.visible = False
        if OnshapeClient.is_authenticated():
            callback(True)

    def __del__(self):
        self.cancel_auth()


class OnshapeAuthServer(BaseHTTPRequestHandler):
    code = None
    state = None
    params = None

    def do_GET(self):
        try:
            parsed_url = urlparse(self.path)
            OnshapeAuthServer.params = self.path.split(" ")[-1]
            qs = parse_qs(parsed_url.query)
            if "code" in qs:
                OnshapeAuthServer.code = qs["code"][0]
            if "state" in qs:
                OnshapeAuthServer.state = qs["state"][0]
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(bytes("<html><head><title>Onshape Importer: Succes</title></head>", "utf-8"))
            self.wfile.write(bytes("<body>", "utf-8"))
            self.wfile.write(
                bytes(
                    "<p>You have successfully authorized access to your Onshape account. <br>You can continue to work in your application.</p>",
                    "utf-8",
                )
            )
            OnshapeClient.set_authenticated(True)
            OnshapeClient.set_handled_request(True)
            self.wfile.write(bytes("</body></html>", "utf-8"))
        except Exception as e:
            carb.log_error("Error handling auth callback GET: " + str(e))

    def log_message(self, format, *args):
        return


class OnshapeClient(object):
    __onshape_client = None
    __user_mats_lib = None
    __authenticated = False
    __auth_callback = False
    __lock = Lock()
    __webServer = None
    __stop_request = False
    __handled_request = False
    __cleared_client = False

    @staticmethod
    def set_handled_request(value):
        OnshapeClient.__handled_request = value

    @staticmethod
    def is_authenticated():
        return OnshapeClient.__authenticated

    @staticmethod
    def createHttpServer(hostname, serverPort):
        if OnshapeClient.__webServer:
            OnshapeClient.stop_httpServer()

        OnshapeClient.__webServer = HTTPServer((hostname, serverPort), OnshapeAuthServer)

        return OnshapeClient.__webServer

    @staticmethod
    def get_httpServer():
        return OnshapeClient.__webServer

    @staticmethod
    def stop_httpServer():
        if OnshapeClient.__webServer:
            # time.sleep(3)
            OnshapeClient.__webServer.server_close()
            OnshapeClient.__webServer = None

    @staticmethod
    def set_auth_callback(callback):
        OnshapeClient.__auth_callback = callback

    @staticmethod
    def set_authenticated(auth):
        OnshapeClient.__authenticated = auth

    @staticmethod
    def clear_client():
        if OnshapeClient.__onshape_client:
            OnshapeClient.__onshape_client = None
            del Client.singleton_instace
            Client.clear_client()
            OnshapeClient.__cleared_client = True
            gc.collect()

    @staticmethod
    def auth_callback():
        ret = None
        if OnshapeClient.__auth_callback:
            ret = OnshapeClient.__auth_callback()
            OnshapeClient.__auth_callback = None
        return ret

    @staticmethod
    def get_oauth_client():
        hostName = "localhost"
        serverPort = 4518
        # ID and secret are naming conventions, this is not treated as a secret
        client_id = "7XVZWE3MDZOCYSXEXUJLN4LNHADB42ASGPUPV6Y="
        client_secret = "QZIMKKPIZYQO473R72QEU333XY33NSOXSOMMRUOJQ7HEHRDSPMBA===="

        def auth_callback(url, fetch_token):
            # print("Auth callback")
            if not OnshapeClient.__stop_request:
                qs = parse_qs(urlparse(url).query)
                state = None
                if "state" in qs:
                    state = qs["state"][0]
                webServer = OnshapeClient.createHttpServer(hostName, serverPort)
                # Credentials you get from registering a new application

                webbrowser.get().open(url)
                webServer.timeout = 1
                OnshapeClient.__handled_request = False
                while not (OnshapeClient.__handled_request or OnshapeClient.__stop_request):
                    webServer.handle_request()

                webServer.server_close()
                if OnshapeClient.__handled_request and OnshapeAuthServer.params and not OnshapeClient.__stop_request:
                    # thread = threading.Thread(target=webServer.handle_request)
                    # thread.run()
                    if state == OnshapeAuthServer.state:
                        code = OnshapeAuthServer.code
                    response_uri = "https://{}:{}/oauth-redirect{}".format(
                        hostName, serverPort, OnshapeAuthServer.params[1:]
                    )
                    # print(response_uri)
                    Client.get_client().set_grant_authorization_url_response(response_uri)
                # else:
                #     response_uri = "https://{}:{}/oauth-redirect".format(hostName, serverPort)
                #     try:
                #         Client.get_client().set_grant_authorization_url_response(response_uri)
                #     except Exception:
                #         carb.log_warn("cancelled auth")
                # fetch_token()

            # except Exception as e:
            #     OnshapeClient.__stop_request = True
            #     carb.log_error("error attempting to open Onshape Authentication: " + str(e))

        base_url = carb.settings.get_settings().get(ONSHAPE_BASE_URL)
        OnshapeClient.__onshape_client = Client(
            keys_file=None,
            open_authorize_grant_callback=auth_callback,
            configuration={
                "client_id": client_id,
                "client_secret": client_secret,
                "base_url": base_url,
                "host": base_url,
                "oauth_authorization_method": "python_callback",
                "pool_connections": 100,
                "pool_maxsize": 10000,
                "connection_pool_maxsize": 10000,
            },
        )
        OnshapeClient.__onshape_client.configuration.host = base_url

    @staticmethod
    def authenticate(authenticated_callback):
        OnshapeClient.get()
        if not OnshapeClient.__authenticated:
            auth_popup = ui.Window(
                "Onshape Authentication",
                width=300,
                height=50,
                flags=ui.WINDOW_FLAGS_NO_RESIZE
                | ui.WINDOW_FLAGS_NO_SCROLLBAR
                | ui.WINDOW_FLAGS_NO_TITLE_BAR
                | ui.WINDOW_FLAGS_MODAL,
            )

            def callback(authenticated):
                if authenticated:
                    OnshapeClient.__authenticated = True
                    authenticated_callback()

            with auth_popup.frame:
                window = AuthWindow(auth_popup)
            # window.queue.put(OnshapeClient.get())
            window.start_auth(callback)
        return OnshapeClient.__authenticated

    @staticmethod
    def get():
        with OnshapeClient.__lock:
            if not OnshapeClient.__onshape_client:
                if Client.singleton_instance:
                    OnshapeClient.__onshape_client = onshape_client.client.get_client()
                else:
                    use_api_key = carb.settings.get_settings().get(USE_ONSHAPE_KEY)
                    api_key = carb.settings.get_settings().get(DEFAULT_ONSHAPE_KEY)
                    api_secret = carb.settings.get_settings().get(DEFAULT_ONSHAPE_SECRET)
                    base_url = carb.settings.get_settings().get(ONSHAPE_BASE_URL)
                    auth_url = carb.settings.get_settings().get(ONSHAPE_AUTH_URL)
                    token_url = carb.settings.get_settings().get(ONSHAPE_TOKEN_URL)
                    if use_api_key and api_key and api_secret:
                        OnshapeClient.__onshape_client = Client(
                            keys_file=None,
                            configuration={
                                "access_key": api_key,
                                "secret_key": api_secret,
                                "base_url": base_url,
                                "host": base_url,
                                "authorization_uri": auth_url,
                                "token_uri": token_url,
                            },
                        )
                        OnshapeClient.__onshape_client.configuration.host = base_url
                        OnshapeClient.__authenticated = True
                    else:
                        if OnshapeClient.__cleared_client:
                            OnshapeClient.__onshape_client = Client(
                                keys_file=None, configuration={"base_url": base_url}
                            )
                            OnshapeClient.__onshape_client.configuration.host = base_url
                            OnshapeClient.__cleared_client = False
                        OnshapeClient.get_oauth_client()

                    # Override the API accept map to workaround the API bug
                    OnshapeClient.__onshape_client.assemblies_api.get_features.headers_map["accept"] = [
                        "application/vnd.onshape.v1+json;charset=UTF-8;qs=0.1"
                    ]
            # print(OnshapeClient.__onshape_client.get_client().configuration.host)
            return OnshapeClient.__onshape_client.get_client()

    @staticmethod
    def get_material_library(did, eid):
        base_url = carb.settings.get_settings().get(ONSHAPE_BASE_URL)
        url = "{}/api/materials/libraries/d/{}/e/{}".format(base_url, did, eid)
        r = OnshapeClient.get().api_client.request("GET", url, _preload_content=False, query_params={})
        if r.status == 200:
            return json.loads(r.data)
        return False

    @staticmethod
    def get_default_material_library():
        return OnshapeClient.get_material_library("2718281828459eacfeeda11f", "6bbab304a1f64e7d640a2d7d")

    @staticmethod
    def get_default_material_libraries(update=False):
        if update or not OnshapeClient.__user_mats_lib:
            user_settings = OnshapeClient.get().users_api.get_user_settings_current_logged_in_user()
            libraries = user_settings["material_library_settings"]
            OnshapeClient.__user_mats_lib = [
                OnshapeClient.get_material_library(libs["document_id"], libs["element_id"])
                for libs in libraries["libraries"] + libraries["company_libraries"]
            ]
        return OnshapeClient.__user_mats_lib

    @staticmethod
    def update_metadata(did, wdid, wid, eid, pid, body):
        base_url = carb.settings.get_settings().get(ONSHAPE_BASE_URL)
        url = "{}/api/metadata/d/{}/{}/{}/e/{}/p/{}".format(base_url, did, wdid, wid, eid, pid)
        headers = {
            "accept": "application/vnd.onshape.v1+json;charset=UTF-8;qs=0.1",
            "Content-Type": "application/json;charset=UTF-8; qs=0.09",
            "content-length": str(len(body)),
        }
        r = OnshapeClient.get().api_client.request(
            method="POST", url=url, body=json.loads(body), headers=headers, _preload_content=False, query_params={}
        )
        return r
