# ICRA2023: Autonomous Orbital Manipulation using Vector-Field Inequalities Example

Orbital manipulation code and a minimal example for [our ICRA2023 paper](http://doi.org/10.1109/ICRA48891.2023.10160795).
(https://arxiv.org/pdf/2302.05567.pdf for the green open-access link)

```bib
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
```

In this example, I have implemented the main code of the paper in `orbital_manipulation.py`. 
By moving `xd1` and `xd2`, the user can see the optimization handling the orbital manipulation constraint in real time.

To simplify this example to users outside my lab, I decided to use the `KUKA LBR4+` robots available by default in CoppeliaSim. 

The "eye" is in this simulation is much bigger than in the real experiments for easier visualization and operation on CoppeliaSim.

<img width="500" alt="Screenshot 2023-02-08 at 20 01 44" src="https://user-images.githubusercontent.com/46012516/217511663-ccbacfbe-aeff-4b75-9588-16fb2ecc443e.png">

## Changelog

#### 2025.05

- Package structure was modified to be compliant with the new `marinholab.papers` namespace in PyPI.
- Given that [`DQ_VrepInterface`](https://github.com/dqrobotics/cpp-interface-vrep) was archived, this code was modified to be compliant with [`DQ_CoppeliaSimInterfaceZMQ`](https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq).

## Configuration

Supposing you have a suitable Python3 installation, do as follows.

1. Download this repository

```commandline
git clone https://github.com/mmmarinho/icra2023_orbitalmanipulation.git
cd icra2023_orbitalmanipulation
```

2. Create a virtual environment

```commandline
python3 -m venv venv
source venv/bin/activate
```

- These instructions are for `bash`-enabled systems, for other terminal programs, check the venv documentation linked below.
- Remember to always activate this virtual env again when you want to reuse it.
- For more info on how to use venv [[click here]](https://docs.python.org/3/tutorial/venv.html).

3. Install the package

```commandline
python3 -m pip install marinholab-papers-icra2023-orbitalmanipulation
```

## Running this code

1. Open the `orbital_manipulation_403_rev3.ttt` scene in [CoppeliaSim](https://www.coppeliarobotics.com/downloads)

2. Run the `main` script. Example

```commandline
marinholab_papers_icra2023_orbitalmanipulation
```

- If at this stage you have problems connecting to CoppeliaSim, check the [DQRobotics CoppeliaSim Interface webpage](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html#interface-with-coppeliasim-formely-v-rep) for possible fixes and tips.

3. The simulation will start automatically. You can click and drag the `xd1` and `xd2` to change each robot's setpoint. Then, the robots will move accordingly to keep the orbital manipulation constraint.

4. You can stop the script with `CTRL+C`.

## TODO

- Add the motion of the eye in the simulation. The real eye will naturally move in reaction to the motion enacted by the moving RCM points of each instrument.


