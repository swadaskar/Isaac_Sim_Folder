# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import json
import numpy as np
import os


class Test:
    def __init__(self):
        self.header = None
        self.frames = []

    def log_frame(self, **kwargs):
        self.frames.append(kwargs)

    def log_header(self, **kwargs):
        self.header = kwargs

    def has_header(self):
        return self.header is not None

    def to_json_dict(self):
        # convert object to dict

        body = []

        for f in self.frames:
            frame = f.copy()
            for k, v in frame.items():
                try:
                    frame[k] = np.array(v).tolist()
                except:
                    pass
            body.append(frame)

        if self.header is not None:
            header = self.header.copy()
            for k, v in header.items():
                try:
                    header[k] = np.array(v).tolist()
                except:
                    pass
        else:
            header = None

        return {"header": header, "body": body}


class BenchmarkLogger:
    def __init__(self, out_file):
        self.out_file = out_file
        self._tests = []
        self._curr_test = None

    def set_output_file(self, out_file):
        self.out_file = out_file

    def log_header(self, **kwargs):
        if self._curr_test is None:
            self.new_test()
        self._curr_test.log_header(**kwargs)

    def log_frame(self, **kwargs):
        if self._curr_test is None:
            self.new_test()
        self._curr_test.log_frame(**kwargs)

    def new_test(self):
        self._curr_test = Test()
        self._tests.append(self._curr_test)

    def to_json_dict(self, skip_headerless_tests=True):
        """
        Return the logged test in a json-ready format (all lists and dictionaries of primitive types). 
        """
        tests = []
        for test in self._tests:
            if skip_headerless_tests and not test.has_header():
                continue
            tests.append(test.to_json_dict())
        return tests

    def write_to_json(self, append=False, skip_headerless_tests=True):
        """
        filename: full path to file
        append: if True, append to file if it already exists
                        if False, overwrite the file if it already exists
        skip_headerless_tests: don't write tests that lack a header
            (headers are written at the ends of tests)
        """
        tests = self.to_json_dict(skip_headerless_tests=skip_headerless_tests)

        if append:
            flag = "a"
        else:
            flag = "w"

        os.makedirs(os.path.dirname(self.out_file), exist_ok=True)
        print("Writing test to ", self.out_file)
        with open(self.out_file, flag) as file:
            json.dump(tests, file)
