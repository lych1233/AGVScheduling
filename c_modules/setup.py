from setuptools import setup
from Cython.Build import cythonize

import numpy as np

setup(
    ext_modules=cythonize(["path_table.pyx", "graph.pyx", "sipp.pyx", "mapf_lns2.pyx", "cbs.pyx"]),
    include_dirs=[np.get_include()]
)
