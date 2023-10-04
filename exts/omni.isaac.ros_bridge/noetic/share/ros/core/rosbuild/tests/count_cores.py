#!/usr/bin/env python3

import os
print(os.sysconf(os.sysconf_names['SC_NPROCESSORS_ONLN']))
