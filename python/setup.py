import os
from setuptools import setup, find_packages

def get_version():
    """Read version from ../version file, fallback to 0.0.0."""
    here = os.path.abspath(os.path.dirname(__file__))
    version_file = os.path.join(here, '..', 'version')
    if os.path.exists(version_file):
        with open(version_file, 'r') as f:
            return f.read().strip()
    return '0.0.0'

setup(
    name='rdkimu',
    version=get_version(),
    packages=find_packages(),
    description='SDK for RDK IMU Module',
    python_requires='>=3.4',
    # Force platform‑specific wheel naming (not “any”)
    has_ext_modules=lambda: True,
)