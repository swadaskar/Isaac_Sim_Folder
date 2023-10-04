# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
import omni.kit.pipapi


class TestPipArchive(omni.kit.test.AsyncTestCase):
    # import all packages to make sure dependencies were not missed
    async def test_import_all(self):
        import scipy
        import quaternion
        import numba
        import webbot
        import selenium
        import urllib3
        import requests
        import charset_normalizer
        import construct
        import llvmlite
        import nest_asyncio
        import jinja2
        import markupsafe
        import matplotlib
        import pyparsing
        import cycler
        import kiwisolver
        import packaging
        import pint
        import requests_oauthlib
        import oauthlib
        import yaml
        import osqp
        import qdldl
        import nvsmi
        import bezier
        import boto3
        import s3transfer

        self.assertIsNotNone(scipy)
        self.assertIsNotNone(quaternion)
        self.assertIsNotNone(numba)
        self.assertIsNotNone(webbot)
        self.assertIsNotNone(selenium)
        self.assertIsNotNone(urllib3)
        self.assertIsNotNone(requests)
        self.assertIsNotNone(charset_normalizer)
        self.assertIsNotNone(construct)
        self.assertIsNotNone(llvmlite)
        self.assertIsNotNone(nest_asyncio)
        self.assertIsNotNone(jinja2)
        self.assertIsNotNone(markupsafe)
        self.assertIsNotNone(matplotlib)
        self.assertIsNotNone(pyparsing)
        self.assertIsNotNone(cycler)
        self.assertIsNotNone(kiwisolver)
        self.assertIsNotNone(packaging)
        self.assertIsNotNone(pint)
        self.assertIsNotNone(requests_oauthlib)
        self.assertIsNotNone(oauthlib)
        self.assertIsNotNone(yaml)
        self.assertIsNotNone(osqp)
        self.assertIsNotNone(qdldl)
        self.assertIsNotNone(nvsmi)
        self.assertIsNotNone(bezier)
        self.assertIsNotNone(boto3)
        self.assertIsNotNone(s3transfer)
