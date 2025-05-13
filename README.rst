ICRA2023: Autonomous Orbital Manipulation using Vector-Field Inequalities Example
============================================================================

.. toctree::
   :maxdepth: 2
   :hidden:

   apidocs/index

.. tip::

   If you're reading this page directly on GitHub, it might be better visualised in
   https://mmmarinho.github.io/icra2023_orbitalmanipulation/

.. code-block:: console

    python3 -m pip install marinholab-papers-icra2023-orbitalmanipulation

Orbital manipulation code and a minimal example for
`our ICRA2023 paper <http://doi.org/10.1109/ICRA48891.2023.10160795>`_.

.. code-block:: bibtex

    @InProceedings{koyama2023vitreoretinal,
      author       = {Koyama, Y. and Marinho, M. M. and Harada, K.},
      title        = {Vitreoretinal Surgical Robotic System with Autonomous Orbital Manipulation using Vector-Field Inequalities},
      booktitle    = {2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
      year         = {2023},
      month        = may,
      organization = {IEEE},
      pages        = {4654--4660},
      doi          = {10.1109/ICRA48891.2023.10160795}
    }

(See https://arxiv.org/pdf/2302.05567.pdf for the green open-access file.)

In this example, I implemented the main code of the paper in `marinholab.papers.icra2023.orbital_manipulation._orbital_manipulation`.

By moving `xd1` and `xd2`,
the user can see the controller handling the orbital manipulation constraint at execution time.

To simplify this example to users outside my lab, I used the `KUKA LBR4+` robots available by default in CoppeliaSim. 

The "eye" is in this simulation is much bigger than in the real experiments for easier visualization and operation on CoppeliaSim.

.. image:: https://user-images.githubusercontent.com/46012516/217511663-ccbacfbe-aeff-4b75-9588-16fb2ecc443e.png
  :width: 500
  :alt: Simulation screenshot

Code documentation
------------------

https://mmmarinho.github.io/icra2023_orbitalmanipulation/apidocs/index.html

Changelog
---------

2025.05
+++++++

- Package structure was modified to be compliant with the new ``marinholab.papers`` namespace in PyPI.
- Given that `DQ_VrepInterface <https://github.com/dqrobotics/cpp-interface-vrep>`_ was archived, this code was modified 
  to be compliant with `DQ_CoppeliaSimInterfaceZMQ <https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq>`_.
- Adjusted API documentation and created webpage via ``sphinx`` and ``autodoc2``.

Configuration
-------------

Supposing you have a suitable Python3 installation, do as follows.

1. Download this repository::

    git clone https://github.com/mmmarinho/icra2023_orbitalmanipulation.git
    cd icra2023_orbitalmanipulation

2. Create a virtual environment::

    python3 -m venv venv
    source venv/bin/activate

* These instructions are for ``bash``-enabled systems, for other terminal programs, check the venv documentation linked below.
* Remember to always activate this virtual env again when you want to reuse it.
* For more info on how to use venv `click here <https://docs.python.org/3/tutorial/venv.html>`_.

3. Install the package::

    python3 -m pip install marinholab-papers-icra2023-orbitalmanipulation

Running this code
-----------------

1. Open the ``orbital_manipulation_403_rev3.ttt`` scene in `CoppeliaSim <https://www.coppeliarobotics.com/downloads>`_

2. Run the ``main`` script. Example::

    marinholab_papers_icra2023_orbitalmanipulation

If at this stage you have problems connecting to CoppeliaSim, check the `DQRobotics CoppeliaSim Interface webpage <https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html#interface-with-coppeliasim-formely-v-rep>`_ for possible fixes and tips.

TODO
----

- Add the motion of the eye in the simulation. The real eye will naturally move in reaction to the motion enacted by the moving RCM points of each instrument.


