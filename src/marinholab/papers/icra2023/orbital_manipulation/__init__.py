from marinholab.papers.icra2023.orbital_manipulation._orbital_manipulation import *

# https://setuptools-git-versioning.readthedocs.io/en/stable/runtime_version.html
from importlib.metadata import version, PackageNotFoundError
try:
    __version__ = version("marinholab-papers-icra2023-orbitalmanipulation")
except PackageNotFoundError:
    # package is not installed
    pass