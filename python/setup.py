# Copyright (c) 2026 Leaf. D-Robotics.
# SPDX-License-Identifier: MIT
import os, glob, shutil
from setuptools import setup, find_packages
from setuptools.command.build_py import build_py as _build_py

def get_version():
    here = os.path.abspath(os.path.dirname(__file__))
    vf = os.path.join(here, '..', 'version')
    if os.path.exists(vf):
        with open(vf) as f:
            return f.read().strip()
    return '0.0.0'

class BuildPyCommand(_build_py):
    def run(self):
        core_lib = os.path.join(os.path.dirname(__file__), '..', 'core', 'lib')
        if not os.path.isdir(core_lib):
            raise FileNotFoundError(
                f"Core library directory not found: {core_lib}\n"
                "Please build the core library first (e.g., run 'make lib' in ../core)."
            )
        found = glob.glob(os.path.join(core_lib, 'librdkimu.so*'))
        if not found:
            raise FileNotFoundError(
                f"No librdkimu.so* files found in {core_lib}\n"
                "Please build the core library first."
            )
        dest = os.path.join(self.build_lib, 'rdkimu')
        os.makedirs(dest, exist_ok=True)
        for src in found:
            real = os.path.realpath(src) if os.path.islink(src) else src
            shutil.copy2(real, os.path.join(dest, os.path.basename(src)))
        super().run()

setup(
    name='rdkimu',
    version=get_version(),
    packages=find_packages(),
    description='SDK for RDK IMU Module',
    python_requires='>=3.4',
    has_ext_modules=lambda: True,
    package_data={'rdkimu': ['librdkimu.so*']},
    include_package_data=True,
    cmdclass={'build_py': BuildPyCommand},
)