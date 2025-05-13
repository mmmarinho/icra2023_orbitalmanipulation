"""
This module encloses the main functions of the orbital manipulation paper.

    Y. Koyama, M. M. Marinho and K. Harada, "Vitreoretinal Surgical Robotic System with Autonomous Orbital
    Manipulation using Vector-Field Inequalities," 2023 IEEE International Conference on Robotics and
    Automation (ICRA), London, United Kingdom, 2023, pp. 4654-4660, doi: 10.1109/ICRA48891.2023.10160795.

See _orbital_manipulation.py for more details.
"""
from marinholab.papers.icra2023.orbital_manipulation._orbital_manipulation import *

# https://setuptools-git-versioning.readthedocs.io/en/stable/runtime_version.html
from importlib.metadata import version, PackageNotFoundError
try:
    __version__ = version("marinholab-papers-icra2023-orbitalmanipulation")
except PackageNotFoundError:
    # package is not installed
    pass