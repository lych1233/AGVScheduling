# Copyright (c) 2023, Yancheng Liang, lych1233@gmail.com
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
from setuptools import setup
from Cython.Build import cythonize

import numpy as np

setup(
    ext_modules=cythonize(["path_table.pyx", "graph.pyx", "sipp.pyx", "mapf_lns2.pyx", "cbs.pyx"]),
    include_dirs=[np.get_include()]
)
