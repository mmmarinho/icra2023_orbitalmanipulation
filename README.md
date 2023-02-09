# ICRA2023 Autonomous Orbital Manipulation using Vector-Field Inequalities Example

Orbital manipulation code and minimal example for the ICRA2023 paper: 

```bib
@InProceedings{koyama2023vitreoretinal,
  author       = {Koyama, Y. and Marinho, M. M. and Harada, K.},
  title        = {Vitreoretinal Surgical Robotic System with Autonomous Orbital Manipulation using Vector-Field Inequalities},
  booktitle    = {2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year         = {2023},
  month        = may,
  organization = {IEEE},
  pages        = {1--7},
}
```

In this example, I have implemented the main code of the paper in `orbital_manipulation.py`. 
By moving `xd1` and `xd2`, the user can see the optimization handling the orbital manipulation constraint in real time.

To make this example simpler to users outside my lab, I have decided to use the `KUKA LBR4+` robots available by default in CoppeliaSim. 

The "eye" is in this simulation is much bigger than in the real experiments for easier visualization and operation on CoppeliaSim.

<img width="500" alt="Screenshot 2023-02-08 at 20 01 44" src="https://user-images.githubusercontent.com/46012516/217511663-ccbacfbe-aeff-4b75-9588-16fb2ecc443e.png">

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

3. Install the prerequisites

```commandline
python3 -m pip install -r requirements.txt
```

## Running this code

1. Open the `orbital_manipulation.ttt` scene in [CoppeliaSim](https://www.coppeliarobotics.com/downloads)

2. Run the `main.py` script. Example

```commandline
python3 main.py
```

- If at this stage you have problems connecting to CoppeliaSim, check the [DQRobotics CoppeliaSim Interface webpage](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/python.html#interface-with-coppeliasim-formely-v-rep) for possible fixes and tips.

3. The simulation will start automatically. You can click and drag the `xd1` and `xd2` to change each robot's setpoint. Then, the robots will move accordingly to keep the orbital manipulation constraint.

4. You can stop the script with `CTRL+C`.

## TODO

- Add the motion of the eye in the simulation. The real eye will naturally move according to the motion enacted by the moving RCM points of each instrument.


